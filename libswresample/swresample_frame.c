/*
 * Copyright (c) 2014 Luca Barbato <lu_zero@gentoo.org>
 * Copyright (c) 2014 Michael Niedermayer <michaelni@gmx.at>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "swresample_internal.h"
#include "libavutil/channel_layout.h"
#include "libavutil/frame.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

/* An input format change replaces the whole conversion state (filter history,
 * buffered input, resampler phase). Everything the old state still owes has to
 * leave the conversion first, otherwise the old tail is lost. Since delivering
 * it may take more output space than the caller offers in a single call, and
 * because the samples must keep their place even if the new format turns out
 * to be unusable, they are staged in a FIFO.
 *
 * Outside of such a handover the conversion writes straight into the caller's
 * frame exactly as before, and the samples remain retrievable through
 * swr_convert() as documented. */

void swri_frame_free(SwrContext *s)
{
    av_audio_fifo_free(s->frame_fifo);
    s->frame_fifo      = NULL;
    s->frame_fifo_fmt  = AV_SAMPLE_FMT_NONE;
    s->frame_fifo_ch   = 0;
    s->frame_fifo_rate = 0;
    s->frame_drained   = 0;

    av_freep(&s->frame_tmp.data);
    memset(&s->frame_tmp, 0, sizeof(s->frame_tmp));
}

static int frame_staged(const SwrContext *s)
{
    return s->frame_fifo && av_audio_fifo_size(s->frame_fifo) > 0;
}

/* Make sure the staging FIFO matches the current output configuration. Staged
 * samples cannot be converted to a different output format/layout/rate. */
static int frame_fifo_init(SwrContext *s)
{
    if (s->frame_fifo) {
        if (s->frame_fifo_fmt  == s->out_sample_fmt &&
            s->frame_fifo_ch   == s->out_ch_layout.nb_channels &&
            s->frame_fifo_rate == s->out_sample_rate)
            return 0;
        swri_frame_free(s);
    }

    if (s->out_ch_layout.nb_channels <= 0 ||
        av_get_bytes_per_sample(s->out_sample_fmt) <= 0)
        return AVERROR(EINVAL);

    s->frame_fifo = av_audio_fifo_alloc(s->out_sample_fmt,
                                        s->out_ch_layout.nb_channels, 1024);
    if (!s->frame_fifo)
        return AVERROR(ENOMEM);

    s->frame_fifo_fmt  = s->out_sample_fmt;
    s->frame_fifo_ch   = s->out_ch_layout.nb_channels;
    s->frame_fifo_rate = s->out_sample_rate;
    return 0;
}

/* Point the scratch buffer at the current output configuration, discarding
 * any allocation made for a different one. */
static void set_frame_tmp_fmt(SwrContext *s)
{
    AudioData *a = &s->frame_tmp;
    int ch_count = s->out_ch_layout.nb_channels;
    int bps      = av_get_bytes_per_sample(s->out_sample_fmt);
    int planar   = av_sample_fmt_is_planar(s->out_sample_fmt) || ch_count == 1;

    if (a->data && (a->ch_count != ch_count || a->bps != bps ||
                    a->planar != planar)) {
        av_freep(&a->data);
        memset(a, 0, sizeof(*a));
    }

    a->ch_count = ch_count;
    a->bps      = bps;
    a->planar   = planar;
    a->fmt      = s->out_sample_fmt;
}

/* Pull everything the current conversion state can still produce - the
 * resampler filter tail as well as any input buffered internally - into the
 * staging FIFO. */
static int frame_drain(SwrContext *s)
{
    int ret, n;

    if (s->frame_drained)
        return 0;

    if ((ret = frame_fifo_init(s)) < 0)
        return ret;

    if (!swr_is_initialized(s)) {
        s->frame_drained = 1;
        return 0;
    }

    set_frame_tmp_fmt(s);
    for (;;) {
        int chunk = swr_get_out_samples(s, 0);
        if (chunk < 0)
            return chunk;
        chunk = FFMAX(chunk, 4096);

        if ((ret = swri_realloc_audio(&s->frame_tmp, chunk)) < 0)
            return ret;

        n = swr_convert(s, s->frame_tmp.ch, chunk, NULL, 0);
        if (n < 0)
            return n;
        if (!n)
            break;

        if ((ret = av_audio_fifo_write(s->frame_fifo,
                                       (void **)s->frame_tmp.ch, n)) < 0)
            return ret;
    }

    s->frame_drained = 1;
    return 0;
}

/* Convert one input frame into the staging FIFO. */
static int frame_convert_staged(SwrContext *s, const AVFrame *in)
{
    int ret, max, n;

    if ((ret = frame_fifo_init(s)) < 0)
        return ret;

    /* A previous drain flushed the filter state. Resuming with fresh input
     * needs a clean state so the flushed history is not used again. */
    if (s->frame_drained) {
        if ((ret = swr_init(s)) < 0)
            return ret;
        s->frame_drained = 0;
    }

    max = swr_get_out_samples(s, in->nb_samples);
    if (max < 0)
        return max;
    max = FFMAX(max, 1);

    set_frame_tmp_fmt(s);
    if ((ret = swri_realloc_audio(&s->frame_tmp, max)) < 0)
        return ret;

    n = swr_convert(s, s->frame_tmp.ch, max,
                    (const uint8_t **)in->extended_data, in->nb_samples);
    if (n < 0)
        return n;

    if (n) {
        if ((ret = av_audio_fifo_write(s->frame_fifo,
                                       (void **)s->frame_tmp.ch, n)) < 0)
            return ret;
    }

    return 0;
}

/* Move up to the space available in @p out from the staging FIFO. */
static int frame_deliver(SwrContext *s, AVFrame *out)
{
    int ret, n;

    if (!s->frame_fifo || !out)
        return 0;

    /* The staged samples can only be delivered as they were converted. */
    if (out->format != s->frame_fifo_fmt ||
        out->ch_layout.nb_channels != s->frame_fifo_ch ||
        out->sample_rate != s->frame_fifo_rate)
        return AVERROR_OUTPUT_CHANGED;

    n = FFMIN(out->nb_samples, av_audio_fifo_size(s->frame_fifo));
    if (n <= 0) {
        out->nb_samples = 0;
        return 0;
    }

    ret = av_audio_fifo_read(s->frame_fifo, (void **)out->extended_data, n);
    if (ret < 0) {
        out->nb_samples = 0;
        return ret;
    }
    out->nb_samples = ret;
    return 0;
}

/* Apply only the input side of the configuration. The output configuration is
 * kept, so samples already staged in the FIFO remain deliverable. */
static int config_input(SwrContext *s, const AVFrame *in)
{
    AVChannelLayout ch_layout = { 0 };
    int ret;

    if ((ret = av_channel_layout_copy(&ch_layout, &in->ch_layout)) < 0)
        goto fail;
    if ((ret = av_opt_set_chlayout(s, "ichl", &ch_layout, 0)) < 0)
        goto fail;
    if ((ret = av_opt_set_int(s, "isf", in->format, 0)) < 0)
        goto fail;
    if ((ret = av_opt_set_int(s, "isr", in->sample_rate, 0)) < 0)
        goto fail;

    ret = swr_init(s);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Failed to apply the new input configuration\n");
    } else {
        /* The new conversion state starts empty. */
        s->frame_drained = 0;
    }

fail:
    av_channel_layout_uninit(&ch_layout);
    return ret;
}

int swr_config_frame(SwrContext *s, const AVFrame *out, const AVFrame *in)
{
    AVChannelLayout ch_layout = { 0 };
    int ret;

    swr_close(s);

    if (in) {
        if ((ret = av_channel_layout_copy(&ch_layout, &in->ch_layout)) < 0)
            goto fail;
        if ((ret = av_opt_set_chlayout(s, "ichl", &ch_layout, 0)) < 0)
            goto fail;
        if ((ret = av_opt_set_int(s, "isf", in->format, 0)) < 0)
            goto fail;
        if ((ret = av_opt_set_int(s, "isr", in->sample_rate, 0)) < 0)
            goto fail;
    }

    if (out) {
        if ((ret = av_channel_layout_copy(&ch_layout, &out->ch_layout)) < 0)
            goto fail;
        if ((ret = av_opt_set_chlayout(s, "ochl", &ch_layout, 0)) < 0)
            goto fail;
        if ((ret = av_opt_set_int(s, "osf", out->format,  0)) < 0)
            goto fail;
        if ((ret = av_opt_set_int(s, "osr", out->sample_rate, 0)) < 0)
            goto fail;
    }

    ret = 0;
fail:
    if (ret < 0)
        av_log(s, AV_LOG_ERROR, "Failed to set option\n");
    av_channel_layout_uninit(&ch_layout);
    return ret;
}

/* Report whether @p frame's layout differs from the configured one. The
 * configured layout may have been replaced by a default one when an
 * unspecified layout was set, so a frame matching either representation is
 * accepted. */
static int layout_changed(const AVChannelLayout *user, const AVChannelLayout *effective,
                          const AVFrame *frame)
{
    return av_channel_layout_compare(user, &frame->ch_layout) &&
           av_channel_layout_compare(effective, &frame->ch_layout);
}

/* Compare the frames against the configuration. The two kinds of change are
 * reported separately because AVERROR_INPUT_CHANGED and
 * AVERROR_OUTPUT_CHANGED, although documented as OR-able, cannot be told
 * apart once combined. */
static int config_changed(SwrContext *s,
                          const AVFrame *out, const AVFrame *in,
                          int *input_changed, int *output_changed)
{
    *input_changed  = 0;
    *output_changed = 0;

    if (in &&
        (layout_changed(&s->user_in_chlayout, &s->in_ch_layout, in) ||
         s->in_sample_rate != in->sample_rate ||
         s->in_sample_fmt  != in->format))
        *input_changed = 1;

    if (out &&
        (layout_changed(&s->user_out_chlayout, &s->out_ch_layout, out) ||
         s->out_sample_rate != out->sample_rate ||
         s->out_sample_fmt  != out->format))
        *output_changed = 1;

    return 0;
}

static inline int available_samples(AVFrame *out)
{
    int bytes_per_sample = av_get_bytes_per_sample(out->format);
    int samples = out->linesize[0] / bytes_per_sample;

    if (av_sample_fmt_is_planar(out->format)) {
        return samples;
    } else {
        int channels = out->ch_layout.nb_channels;
        return samples / channels;
    }
}

/* Conversion that writes straight into the caller's frame, letting samples
 * that do not fit in the offered space stay in the context's own FIFO. */
static int convert_frame(SwrContext *s, AVFrame *out, const AVFrame *in)
{
    int ret;
    uint8_t **out_data = NULL;
    const uint8_t **in_data = NULL;
    int out_nb_samples = 0, in_nb_samples = 0;

    if (out) {
        out_data       = out->extended_data;
        out_nb_samples = out->nb_samples;
    }

    if (in) {
        in_data       = (const uint8_t **)in->extended_data;
        in_nb_samples = in->nb_samples;
    }

    ret = swr_convert(s, out_data, out_nb_samples, in_data, in_nb_samples);

    if (ret < 0) {
        if (out)
            out->nb_samples = 0;
        return ret;
    }

    if (out)
        out->nb_samples = ret;

    return 0;
}

/* Prepare the caller's output frame. For a frame without buffers, size it to
 * the samples the staged FIFO holds; for a buffered frame without an explicit
 * nb_samples, derive it from the available space. */
static int prepare_out_staged(SwrContext *s, AVFrame *out)
{
    int ret;

    if (!out)
        return 0;

    if (!out->linesize[0]) {
        int available = s->frame_fifo ? av_audio_fifo_size(s->frame_fifo) : 0;

        if (available <= 0) {
            out->nb_samples = 0;
            return 0;
        }

        out->nb_samples = available;
        if ((ret = av_frame_get_buffer(out, 0)) < 0) {
            out->nb_samples = 0;
            /* The converted samples stay staged so the caller can retry. */
            return ret;
        }
    } else if (!out->nb_samples) {
        out->nb_samples = available_samples(out);
    }

    return 0;
}

int swr_convert_frame(SwrContext *s,
                      AVFrame *out, const AVFrame *in)
{
    int ret, input_changed = 0, output_changed = 0;

    if (!swr_is_initialized(s)) {
        if (in) {
            if ((ret = swr_config_frame(s, out, in)) < 0)
                return ret;
            if ((ret = swr_init(s)) < 0)
                return ret;
            s->frame_drained = 0;
        } else if (!frame_staged(s)) {
            /* Nothing configured and nothing staged: nothing to deliver. */
            if (out)
                out->nb_samples = 0;
            return 0;
        }
    } else {
        if ((ret = config_changed(s, out, in,
                                  &input_changed, &output_changed)) < 0)
            return ret;

        if (output_changed) {
            if (out)
                out->nb_samples = 0;
            if (input_changed)
                return AVERROR_OUTPUT_CHANGED | AVERROR_INPUT_CHANGED;
            return AVERROR_OUTPUT_CHANGED;
        }

        if (input_changed) {
            /* Hand the old tail over to the staging FIFO before the state it
             * belongs to is replaced. */
            if ((ret = frame_drain(s)) < 0) {
                if (out)
                    out->nb_samples = 0;
                return ret;
            }

            /* Setting up the new input may still fail. The samples staged
             * above belong to the old segment and stay deliverable. */
            if ((ret = config_input(s, in)) < 0) {
                if (out)
                    out->nb_samples = 0;
                return AVERROR_INPUT_CHANGED;
            }
        }
    }

    if (frame_staged(s) || (in && s->frame_drained)) {
        /* A handover is in progress: convert into the staging FIFO and hand
         * out what fits, so old and new segments keep their order. */
        if (in) {
            if ((ret = frame_convert_staged(s, in)) < 0)
                goto fail;
        } else if (swr_is_initialized(s)) {
            if ((ret = frame_drain(s)) < 0)
                goto fail;
        }

        if ((ret = prepare_out_staged(s, out)) < 0)
            return ret;
        return frame_deliver(s, out);
    }

    /* Plain conversion: unchanged behaviour, feeding the context's own
     * buffers directly. */
    if (out) {
        if (!out->linesize[0]) {
            out->nb_samples = swr_get_delay(s, s->out_sample_rate) + 3;
            if (in)
                out->nb_samples += in->nb_samples * (int64_t)s->out_sample_rate /
                                   s->in_sample_rate;
            if ((ret = av_frame_get_buffer(out, 0)) < 0) {
                out->nb_samples = 0;
                return ret;
            }
        } else if (!out->nb_samples) {
            out->nb_samples = available_samples(out);
        }
    }

    return convert_frame(s, out, in);

fail:
    if (out)
        out->nb_samples = 0;
    return ret;
}
