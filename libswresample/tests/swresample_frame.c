/*
 * Exercise swr_convert_frame() across an input format change.
 *
 * The output configuration is fixed; the input switches its channel layout,
 * sample format and sample rate mid-stream. The test checks the properties the
 * frame API promises, comparing runs inside the process so the result does not
 * depend on floating point details of a particular build:
 *
 *  - the delivered sample stream must not depend on how much output space the
 *    caller offers per call;
 *  - a rejected new configuration must not swallow the samples already
 *    accepted (the total must match a run without the rejected segment);
 *  - accepted transitions (layout, rate, sample format) must produce the
 *    expected number of output samples for their input.
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "libavutil/channel_layout.h"
#include "libavutil/error.h"
#include "libavutil/frame.h"
#include "libavutil/mem.h"
#include "libavutil/samplefmt.h"
#include "libswresample/swresample.h"

static const AVChannelLayout OUT_LAYOUT  = AV_CHANNEL_LAYOUT_STEREO;
static const enum AVSampleFormat OUT_FMT = AV_SAMPLE_FMT_FLTP;
static const int OUT_RATE                 = 44100;

/* A segment's description: layout, sample format, rate and value ramp. */
typedef struct Segment {
    AVChannelLayout     layout;
    enum AVSampleFormat fmt;
    int                 rate;
    int                 frames;
    int                 frame_size;
    float               base;
    int                 invalid_layout; ///< attach this layout only after allocating
} Segment;

/* Write a distinguishable ramp into one plane of a frame. */
static void fill_plane(void *dst, enum AVSampleFormat fmt, int ch, int i,
                       const Segment *seg)
{
    double v = seg->base + 0.125 * ch + 0.0001 * (i % 89);
    switch (fmt) {
    case AV_SAMPLE_FMT_FLTP: ((float   *)dst)[i] = (float)v;                         break;
    case AV_SAMPLE_FMT_DBLP: ((double  *)dst)[i] = v;                                break;
    case AV_SAMPLE_FMT_S16P: ((int16_t *)dst)[i] = (int16_t)lrint(v * 32767);        break;
    case AV_SAMPLE_FMT_S32P: ((int32_t *)dst)[i] = (int32_t)llrint(v * 2147483647.0); break;
    default: break;
    }
}

static AVFrame *make_input(const Segment *seg)
{
    AVFrame *f = av_frame_alloc();
    int ret;
    if (!f)
        return NULL;
    f->format      = seg->fmt;
    f->sample_rate = seg->rate;
    f->nb_samples  = seg->frame_size;
    /* An invalid layout cannot be allocated with. Allocate for a valid layout
     * of the same channel count and attach the bad one afterwards. */
    av_channel_layout_copy(&f->ch_layout,
                           seg->invalid_layout ? &OUT_LAYOUT : &seg->layout);
    if ((ret = av_frame_get_buffer(f, 0)) < 0) {
        av_frame_free(&f);
        return NULL;
    }
    if (seg->invalid_layout) {
        av_channel_layout_uninit(&f->ch_layout);
        av_channel_layout_copy(&f->ch_layout, &seg->layout);
    }

    for (int ch = 0; ch < f->ch_layout.nb_channels; ch++)
        for (int i = 0; i < seg->frame_size; i++)
            fill_plane(f->data[ch], seg->fmt, ch, i, seg);
    return f;
}

static AVFrame *make_output(int space)
{
    AVFrame *f = av_frame_alloc();
    if (!f)
        return NULL;
    f->format      = OUT_FMT;
    f->sample_rate = OUT_RATE;
    av_channel_layout_copy(&f->ch_layout, &OUT_LAYOUT);
    if (space > 0) {
        f->nb_samples = space;
        if (av_frame_get_buffer(f, 0) < 0) {
            av_frame_free(&f);
            return NULL;
        }
    }
    return f;
}

static void reset_output(AVFrame *f, int space)
{
    av_frame_unref(f);
    f->format      = OUT_FMT;
    f->sample_rate = OUT_RATE;
    av_channel_layout_copy(&f->ch_layout, &OUT_LAYOUT);
    if (space > 0) {
        f->nb_samples = space;
        av_frame_get_buffer(f, 0);
    }
}

/* Accumulate a running checksum over everything delivered. */
typedef struct Collector {
    long   samples;
    double sum;
    double energy;
} Collector;

static void collect(Collector *c, const AVFrame *f)
{
    if (f->nb_samples <= 0)
        return;
    for (int ch = 0; ch < f->ch_layout.nb_channels; ch++) {
        const float *p = (const float *)f->data[ch];
        for (int i = 0; i < f->nb_samples; i++) {
            c->sum    += p[i];
            c->energy += (double)p[i] * p[i];
        }
    }
    c->samples += f->nb_samples;
}

/* Feed the listed segments, optionally draining after each one (what a caller
 * does after a rejected configuration), then flush. Returns 0 or an AVERROR
 * other than AVERROR_INPUT_CHANGED. */
static int run(Segment *segs, int nsegs, int space, int drain_each,
               Collector *out, int *rejected)
{
    SwrContext *swr = swr_alloc();
    AVFrame *frame  = make_output(space);
    int ret = 0;

    if (!swr || !frame) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    *rejected = 0;

    for (int s = 0; s < nsegs && !ret; s++) {
        for (int i = 0; i < segs[s].frames && !ret; i++) {
            AVFrame *in = make_input(&segs[s]);
            if (!in) {
                ret = AVERROR(ENOMEM);
                break;
            }
            ret = swr_convert_frame(swr, frame, in);
            av_frame_free(&in);
            if (ret < 0) {
                /* Only a rejected input configuration is expected here; the
                 * samples accepted so far must remain deliverable. */
                if (ret == AVERROR_INPUT_CHANGED) {
                    (*rejected)++;
                    ret = 0;
                    reset_output(frame, space);
                } else {
                    break;
                }
            } else {
                collect(out, frame);
                reset_output(frame, space);
            }
        }

        if (drain_each && !ret) {
            for (;;) {
                ret = swr_convert_frame(swr, frame, NULL);
                if (ret < 0)
                    break;
                int got = frame->nb_samples;
                collect(out, frame);
                reset_output(frame, space);
                if (got <= 0)
                    break;
            }
        }
    }

    if (!ret) {
        for (;;) {
            ret = swr_convert_frame(swr, frame, NULL);
            if (ret < 0)
                break;
            int got = frame->nb_samples;
            collect(out, frame);
            reset_output(frame, space);
            if (got <= 0)
                break;
        }
    }

end:
    av_frame_free(&frame);
    swr_free(&swr);
    return ret;
}

static int same_stream(const Collector *a, const Collector *b)
{
    if (a->samples != b->samples)
        return 0;
    if (fabs(a->sum - b->sum) > 1e-3 * (1.0 + fabs(a->sum)))
        return 0;
    if (fabs(a->energy - b->energy) > 1e-3 * (1.0 + fabs(a->energy)))
        return 0;
    return 1;
}

int main(void)
{
    Segment a_stereo_48k = { AV_CHANNEL_LAYOUT_STEREO, AV_SAMPLE_FMT_FLTP, 48000, 30, 441, 0.20f, 0 };
    Segment b_mono_48k   = { AV_CHANNEL_LAYOUT_MONO,   AV_SAMPLE_FMT_FLTP, 48000, 20, 441, 0.60f, 0 };
    Segment b_stereo_32k = { AV_CHANNEL_LAYOUT_STEREO, AV_SAMPLE_FMT_FLTP, 32000, 20, 320, 0.60f, 0 };
    Segment b_s16p_48k   = { AV_CHANNEL_LAYOUT_STEREO, AV_SAMPLE_FMT_S16P, 48000, 20, 441, 0.60f, 0 };
    Segment bad_layout   = { { 0 }, AV_SAMPLE_FMT_FLTP, 48000, 1, 128, 0.0f, 1 };

    Collector ref, alt, with_bad;
    Segment seq[3];
    int ret, rejected, ok;
    int failures = 0;

    if (av_channel_layout_custom_init(&bad_layout.layout, 2) < 0)
        return 1;
    bad_layout.layout.u.map[0].id = AV_CHAN_NONE; /* invalid -> rejected */

    /* Reference run: stereo 48k then mono 48k, auto-sized output frames. */
    seq[0] = a_stereo_48k;
    seq[1] = b_mono_48k;
    ref = (Collector){ 0 };
    ret = run(seq, 2, 0, 0, &ref, &rejected);

    printf("format-change: ret=%d rejected=%d samples=%ld\n",
           ret, rejected, ref.samples);
    ok = !ret && !rejected && ref.samples > 0;
    failures += !ok;
    printf("format-change accepted: %s\n", ok ? "yes" : "no");

    /* Batch size must not change the stream. */
    for (int space = 37; space <= 4096; space *= 4) {
        alt = (Collector){ 0 };
        ret = run(seq, 2, space, 0, &alt, &rejected);
        ok = !ret && !rejected && same_stream(&ref, &alt);
        failures += !ok;
        printf("batch-size-independent (space=%d): %s\n", space, ok ? "yes" : "no");
    }

    /* Rejected configuration in the middle: the accepted samples of the old
     * segment must still be delivered, so the total matches the run that never
     * saw the bad segment. */
    seq[0] = a_stereo_48k;
    seq[1] = bad_layout;
    seq[2] = b_mono_48k;
    with_bad = (Collector){ 0 };
    ret = run(seq, 3, 128, 1, &with_bad, &rejected);
    ok = !ret && rejected == 1 &&
         with_bad.samples == ref.samples &&
         fabs(with_bad.sum - ref.sum) <= 1e-3 * (1.0 + fabs(ref.sum));
    failures += !ok;
    printf("rejected-format-keeps-accepted: ret=%d rejected=%d samples=%ld ref=%ld -> %s\n",
           ret, rejected, with_bad.samples, ref.samples, ok ? "yes" : "no");

    /* Layout + rate change is accepted. The delivered count follows the total
     * duration at the output rate, whatever the intermediate sample rate. */
    seq[0] = a_stereo_48k;
    seq[1] = b_stereo_32k;
    alt = (Collector){ 0 };
    ret = run(seq, 2, 128, 0, &alt, &rejected);
    {
        double dur_a = (double)a_stereo_48k.frames * a_stereo_48k.frame_size / a_stereo_48k.rate;
        double dur_b = (double)b_stereo_32k.frames * b_stereo_32k.frame_size / b_stereo_32k.rate;
        long wanted  = (long)llrint((dur_a + dur_b) * OUT_RATE);
        long tolerate = 64;
        ok = !ret && !rejected && labs(alt.samples - wanted) <= tolerate;
        failures += !ok;
        printf("rate-change: ret=%d samples=%ld expected~%ld -> %s\n",
               ret, alt.samples, wanted, ok ? "yes" : "no");
    }
    /* Sample format change is accepted. */
    seq[0] = a_stereo_48k;
    seq[1] = b_s16p_48k;
    alt = (Collector){ 0 };
    ret = run(seq, 2, 128, 0, &alt, &rejected);
    ok = !ret && !rejected && alt.samples == ref.samples;
    failures += !ok;
    printf("format-change-samplefmt: ret=%d samples=%ld ref=%ld -> %s\n",
           ret, alt.samples, ref.samples, ok ? "yes" : "no");

    av_channel_layout_uninit(&bad_layout.layout);
    return failures ? 1 : 0;
}
