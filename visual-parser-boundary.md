# MPEG-4 Visual 解析器的帧边界与输入消费分析

分析对象：FFmpeg 5 源码树中的 MPEG-4 Visual (ISO/IEC 14496-2) 解析器。

涉及文件（行号均对应当前工作树）：

| 文件 | 作用 |
|---|---|
| `libavcodec/mpeg4video_parser.c` | MPEG-4 Visual 专用 parser：找帧边界、解析 VOP 头 |
| `libavcodec/parser.c` | 通用 parser 框架：`av_parser_parse2()`、`ff_combine_frame()` |
| `libavcodec/parser.h` / `parser_internal.h` | `ParseContext` 状态结构、`END_NOT_FOUND` 常量 |
| `libavcodec/mpeg4videodefs.h` | 起始码常量（`VOP_STARTCODE` 等） |
| `libavcodec/mpeg4videodec.c` | `ff_mpeg4_parse_picture_header()`、`decode_vop_header()`，真正读 `vop_coding_type` |
| `libavcodec/mpegvideo_dec.c` | `ff_mpv_frame_start()`，I 帧打 `AV_FRAME_FLAG_KEY` |
| `libavformat/demux.c` | `parse_packet()`，把任意大小输入块喂给 parser、EOF flush、`AV_PKT_FLAG_KEY` 判定 |

---

## 1. 数据结构与起始码

### 1.1 解析器私有状态

`mpeg4video_parser.c:32-36`：

```c
struct Mp4vParseContext {
    ParseContext pc;                 // 通用帧拼接状态，必须在首位
    Mpeg4DecContext dec_ctx;         // 内嵌一份解码器上下文，仅用于解析头
    int first_picture;
};
```

通用状态 `ParseContext`（`parser.h:28-38`）中与边界直接相关的字段：

```c
typedef struct ParseContext {
    uint8_t *buffer;       // 跨调用累积不完整帧的堆缓冲
    int      index;        // buffer 中已累积的字节数
    int      last_index;
    uint32_t state;        // 最近若干字节，MSB 顺序组成的滑动窗口
    int      frame_start_found;  // 是否已经在流中看到过 VOP 起始码
    int      overread, overread_index;
    uint64_t state64;
} ParseContext;
```

`state` 是整个边界机器的核心：每读一个字节执行 `state = (state << 8) | byte`，因此任何时刻 `state` 的低 32 位就是“最近 4 个字节”的拼接。起始码前缀 `00 00 01` 加上 1 字节起始码标识，恰好 4 字节，可一次比较命中。

### 1.2 起始码常量（`mpeg4videodefs.h:55-61`）

| 宏 | 值（32 位 state 形式） | 字节序列 | 含义 |
|---|---|---|---|
| `VOS_STARTCODE` | `0x000001B0` | `00 00 01 B0` | Visual Object Sequence |
| `GOP_STARTCODE` | `0x000001B3` | `00 00 01 B3` | Group of VOP |
| `VISUAL_OBJ_STARTCODE` | `0x000001B5` | `00 00 01 B5` | Visual Object |
| `VOP_STARTCODE` | `0x000001B6` | `00 00 01 B6` | **Video Object Plane（一帧画面）开始** |
| `SLICE_STARTCODE` | `0x000001B7` | `00 00 01 B7` | video object plane 内的 slice/resync |
| `EXT_STARTCODE` | `0x000001B8` | `00 00 01 B8` | extension（FGS 等） |
| `0x100..0x11F` | — | `00 00 01 00..1F` | video object start codes |
| `0x120..0x12F` | — | `00 00 01 20..2F` | VOL start codes |

parser 不区分大部分起始码的语义；它只区分三类：**VOP（帧的开始）**、**slice/extension（帧内部，忽略）**、**其他一切起始码（帧的结束）**。

---

## 2. 帧边界状态机：`mpeg4_find_frame_end()`

完整代码在 `mpeg4video_parser.c:42-80`。机器分两个阶段，由 `pc->frame_start_found` 区分。

### 2.1 阶段 A：尚未进入任何 VOP（`vop_found == 0`）

```c
for (i = 0; i < buf_size; i++) {
    state = (state << 8) | buf[i];
    if (state == VOP_STARTCODE) {     // 恰好等于 0x000001B6
        i++;
        vop_found = 1;
        break;
    }
}
```

- 在输入里逐字节扫描，直到滑动窗口拼出完整的 `00 00 01 B6`。
- 命中时 `i++` 跳过 `B6` 这一个字节，`break` 后落入阶段 B；此时 `i` 指向 **VOP 起始码之后的第一个字节**（即 VOP 头的第一个比特所在字节）。
- 初始 `state` 为 0（`av_mallocz` 见 `parser.c:60-64`），所以流最前面的 `00 00` 不会误触发。
- 这一阶段遇到 VOS/VOL/GOP/Visual Object 等起始码都不停止——它们被当作“VOP 之前的前导数据”跳过，不产生输出。

### 2.2 阶段 B：已在 VOP 内部，寻找本帧结束（`vop_found == 1`）

```c
if (buf_size == 0)
    return 0;                        // EOF：见第 5 节
for (; i < buf_size; i++) {
    state = (state << 8) | buf[i];
    if ((state & 0xFFFFFF00) == 0x100) {          // 命中任意 00 00 01 xx 起始码
        if (state == SLICE_STARTCODE || state == EXT_STARTCODE)
            continue;                              // B7/B8 是帧内码，不算结束
        pc->frame_start_found = 0;
        pc->state             = -1;
        return i - 3;                              // 关键：返回前缀起点
    }
}
```

判定与返回值的字节语义（可复算）：

1. `(state & 0xFFFFFF00) == 0x00000100` 等价于“最近 4 个字节是 `00 00 01 xx`”，即命中任意 MPEG-4 起始码（含 VO/VOL `0x100–0x12F`、VOS `0x1B0`、GOP `0x1B3`、下一个 VOP `0x1B6` 等）。
2. `00 00 01 B7`（slice / resync marker）和 `00 00 01 B8`（extension）显式 `continue`：它们属于**当前 VOP 内部**，不能结束帧。数据分区（data partitioning）和带 resync marker 的码流因此能正确保持在同一帧内。
3. 其他任何起始码都意味着当前 VOP 结束。设起始码 4 字节在**本次输入缓冲区**中的下标为 `i-3, i-2, i-1, i`（因为 `buf[i]` 是标识字节 `xx`，前缀 `00 00 01` 从 `i-3` 开始），函数返回 **`i - 3`**。

> **边界定义（最重要的一句）**：返回值 `next = i - 3` 是“下一帧起始码前缀 `00 00 01` 的第一个字节”在当前缓冲区中的下标。当前帧的输出范围是 `buf[0 .. next-1]`；从 `buf[next]` 开始的字节（即整个 `00 00 01 xx`）属于下一帧，本次不消费。

4. 命中结束码后 `pc->state = -1`（即 `0xFFFFFFFF`）、`frame_start_found = 0`。重置为全 1 是为了让阶段 A 的扫描不会把结束码残留字节与后续字节错误拼成起始码——全 1 窗口至少需要 3 个新的零字节才可能再次出现 `000001` 前缀。
5. 扫描到缓冲区末尾仍未发现结束码：保存 `frame_start_found` 与 `state` 后返回 `END_NOT_FOUND`（值为 `-100`，`parser.h:40`），等待后续输入。

### 2.3 跨块拼接：状态机为什么不怕任意切分

`state` 是持久化在 `ParseContext` 中的 32 位滑动窗口，每次调用结束时写回（`mpeg4video_parser.c:77-78`）。因此起始码 `00 00 01 B6` 被切成任意块都能在下一次调用中重组：

- 例如块边界落在 `00 | 00 01 B6`：第一次调用后 `state = 0x00000000`；第二次调用依次移入 `00, 01, B6`，第三次移位后 `state = 0x000001B6`，命中。
- `frame_start_found` 同样持久化，阶段 A/B 的身份跨调用保持。

---

## 3. 一次 `av_parser_parse2()` 调用的完整消费账

入口 `mpeg4video_parse()`（`mpeg4video_parser.c:133-157`）：

```c
if (s->flags & PARSER_FLAG_COMPLETE_FRAMES) {
    next = buf_size;                       // 调用方保证整帧输入，不做边界扫描
} else {
    next = mpeg4_find_frame_end(pc, buf, buf_size);
    if (ff_combine_frame(pc, next, &buf, &buf_size) < 0) {
        *poutbuf = NULL;
        *poutbuf_size = 0;
        return buf_size;                   // 没有完整帧，全部字节暂存，消费整个输入
    }
}
mpeg4_decode_header(s, avctx, buf, buf_size);
*poutbuf      = buf;
*poutbuf_size = buf_size;
return next;
```

### 3.1 `ff_combine_frame()` 的三种情形（`parser.c:213-298`）

设本次输入 `buf[0 .. buf_size-1]`。

**情形 1：`next == END_NOT_FOUND`（帧未结束）**

- 输入整体 `memcpy` 进 `pc->buffer` 尾部（`parser.c:237-251`），`pc->index += buf_size`，函数返回 `-1`。
- parser 向 `av_parser_parse2` 返回 `buf_size`：**本次输入被全部消费，但没有任何帧输出**（`poutbuf_size = 0`）。
- 注意这里返回 `-1` 只是 parser 内部信号；`mpeg4video_parse` 把它翻译成“返回输入长度”。

**情形 2：`next >= 0`（在本块内找到帧结束）且 `pc->index == 0`（没有历史缓存）**

- `parser.c:256-257`：`*buf_size = pc->overread_index = pc->index + next = next`。输出直接指向调用方缓冲区的 `buf[0 .. next-1]`，零拷贝。
- parser 返回 `next`：**只消费前 `next` 个字节**，`buf[next .. ]`（下一帧的起始码起）原样留给下次调用。

**情形 3：`next >= 0` 但 `pc->index > 0`（帧跨了多个输入块）**

- `parser.c:260-277`：把本次输入的前 `next` 字节追加到 `pc->buffer`，拼成完整帧，然后令 `*buf = pc->buffer`、`*buf_size = pc->index + next`。
- 输出帧的字节范围 = **历史缓存的全部 `pc->index` 字节 + 本次输入的前 `next` 字节**。
- parser 仍返回 `next`：本次调用只“消费”新输入中的 `next` 字节；历史缓存的消费发生在之前的调用里（每次都已返回当时的 `buf_size`）。
- `pc->index` 清零，缓冲供下一帧复用。

**负 `next` / overread（`parser.c:279-288`）**：当帧结束码的标识字节 `xx` 落在新输入块的前 3 个字节内（即 4 字节起始码的 `00 00 01` 前缀跨在上一块尾部），`i-3` 为负，`find_frame_end` 返回 `-1..-3`。这不是异常，而是跨块起始码的正常情形，3.3 节有完整复算。`ff_combine_frame` 对 `next < -8` 还有一道截断（`parser.c:279-282`），MPEG-4 的取值范围达不到；负 `next` 统一通过 overread 挂账并在下一次调用开头回收（`parser.c:223-225`）。

### 3.2 框架层的偏移记账（`parser.c:186-200`）

```c
if (*poutbuf_size) {
    s->frame_offset      = s->next_frame_offset;   // 已交付帧在源流中的起点
    s->next_frame_offset = s->cur_offset + index;  // 下一帧起点 = 当前消费位置 + next
    s->fetch_timestamp   = 1;
}
...
s->cur_offset += index;   // 源流读指针推进 index 字节
return index;
```

- `index` 就是 parser 返回的 `next`（或整段 `buf_size`）。
- `cur_offset` 始终等于“已从源流消费的总字节数”；`next_frame_offset` 是下一次将交付帧的源字节偏移。两者之差就是 parser 内部暂存但尚未交付的字节数。

### 3.3 可复算的数字例子（含跨块起始码的 overread 路径）

设源流字节布局（`SC` = 4 字节 VOP 起始码 `00 00 01 B6`，`.` = VOP 负载字节）：

```
源流偏移:  0  1  2  3  4 ... 26 | 27 28 29 30 | 31 ... 39
字节:      00 00 01 B6 .  ... . | 00 00 01 B6 | .  ... .
           |<---- 帧 #1：源偏移 [0, 27)，共 27 字节 --->|<-- 帧 #2 ...
           4 字节 VOP 码 + 23 字节负载      下一 VOP 码从偏移 27 开始
```

调用方故意用 3 个不等长块喂入：块 A = 源 `[0,10)`（10 字节）、块 B = 源 `[10,30)`（20 字节）、块 C = 源 `[30,40)`（10 字节）。初始 `state=0, frame_start_found=0, index=0`。

**调用 1：块 A（10 字节）**

| 步骤 | 结果 |
|---|---|
| 阶段 A 扫到本块下标 3：移入 `B6` 后 `state=0x000001B6`，命中 VOP，`vop_found=1`，`i=4` | 进入阶段 B |
| 阶段 B 扫下标 4..9，无任何起始码 | 返回 `END_NOT_FOUND` |
| `ff_combine_frame` 情形 1：10 字节拷入 `pc->buffer`，`index=10`，返回 -1 | 无帧输出 |
| `mpeg4video_parse` 返回 **10**，`poutbuf_size=0` | 消费 10，累计 `cur_offset=10` |

**调用 2：块 B（20 字节，本块下标 0..19 = 源偏移 10..29）**

| 步骤 | 结果 |
|---|---|
| 持久化的 `vop_found=1`，直接进阶段 B。源偏移 27..29 的 `00 00 01` 位于本块下标 17..19，依次移入后 `state=0x00000001`，尚未凑出 4 字节码 | 扫到块尾仍返回 `END_NOT_FOUND` |
| 20 字节追加进 `pc->buffer`，`index=30` | 无帧输出 |
| 返回 **20**，累计 `cur_offset=30` | 源偏移 [0,30) 全部在内部缓冲，无字节丢失 |

**调用 3：块 C（10 字节，本块下标 0 = 源偏移 30 = `B6`）**

| 步骤 | 结果 |
|---|---|
| 移入本块下标 0 的 `B6`：`state = 0x000001B6`，命中起始码且非 B7/B8。此时循环变量 `i=0` | `find_frame_end` 返回 **`next = i - 3 = -3`**（负下标表示帧界落在本块起点之前 3 字节处，即前缀 `00 00 01` 在上一块的尾部） |
| `ff_combine_frame`（`parser.c:254-288`）：`next=-3 >= -8` 不触发 `-8` 截断；先执行 `parser.c:256-257`：`*buf_size = pc->overread_index = pc->index + next = 30 + (-3) = 27` | 输出长度先定为 **27** |
| 因 `index=30>0`，重分配缓冲（大小 `next+index+64`）并 `memcpy(&pc->buffer[30], 块C, next+64=61 字节)`（`parser.c:272-274`）。这里能读满 61 字节依赖 API 契约：送入 parser 的输入缓冲末尾必须附带 `AV_INPUT_BUFFER_PADDING_SIZE=64` 字节可读 padding（`defs.h:40`），demuxer/decoder 分配的包均满足；`pc->index=0`，`*buf=pc->buffer` | 缓冲逻辑视图含帧 #1 的 30 字节 + 块 C 内容，输出指针指向它 |
| `for(; next<0; next++)` 执行 3 次（`parser.c:284-288`）：依次回读 `pc->buffer[last_index-3..-1]`，即源偏移 27..29 的 `00 00 01`，重建 `state=0x00000001`，`pc->overread=3`，`next` 归 0 | 借读登记完成，`buf_size` 保持 27 不变 |
| 回到 `mpeg4video_parse`：`*poutbuf = pc->buffer`，`*poutbuf_size = 27` | **输出帧 #1 = `pc->buffer[0..26]` = 源偏移 [0,27) 的 27 字节，恰好是帧 #1 的真实范围，不含下一帧的任何字节** |
| parser 把 `next=-3` 透传给 `av_parser_parse2`；框架 `if (index<0) index=0`（`parser.c:197-198`） | 返回 **0**，`cur_offset` 不前进 |

记账闭环：输出 27 字节全部来自此前已计入 `cur_offset=30` 的缓冲，而源偏移 [27,30) 的 `00 00 01` 三字节虽物理上已被读入，却通过 `pc->overread=3` 挂账——它们不属于本次输出，也不会被当作已消费而丢弃。

**调用 4：调用方指针不前进，B6 被重新送入**

这是最容易算错的一步，关键在调用方的推进规则（`demux.c:1251-1253`）：

```c
av_assert1(data || !len);
data  = len ? data + len : data;   // len==0（被框架钳零的负 next）时指针原地不动
size -= len;
```

调用 3 返回 0，所以调用方的读指针仍停在源偏移 30 的 `B6` 上；下一次 `av_parser_parse2()` 的输入**从同一个 `B6` 重新开始**（追加其后的新字节，例如送源流 `[30, 64)`）。

进入 `ff_combine_frame` 后先做 overread 回收（`parser.c:223-225`）：

```c
for (; pc->overread > 0; pc->overread--)
    pc->buffer[pc->index++] = pc->buffer[pc->overread_index++];
```

`overread_index` 在调用 3 被设为 27，于是旧缓冲 `pc->buffer[27..29]` 的 `00 00 01` 被复制到新缓冲头部 `pc->buffer[0..2]`，`index=3`、`overread=0`。本次输入从 `B6` 开始，被追加到 `pc->buffer[3..]`——下一帧的 VOP 码 `00 00 01 B6` 在 `pc->buffer` 中**逐字节完整重组**。随后阶段 A 扫描在缓冲内命中这个新 VOP（`frame_start_found` 已在调用 3 被清零），机器重新进入阶段 B。

用同一仿真继续跑到帧 #2 的下一个边界（设帧 #3 的 VOP 码位于源偏移 60），实测：

```
call4 stream[30,64) next=30 ret=30 cur_offset=60 FRAME-OUT len=33
  输出首 4 字节 = 00 00 01 B6
  memcmp(输出, 源流[27,60), 33) == 0   // 与源字节逐字节相同
```

即帧 #2 的交付范围精确等于源偏移 `[27, 60)` 的 33 字节，起始码完整、无重复无缺失。

**源流总账（守恒式，仿真可复算）**

```
帧 #1：源 [0, 27)，长度 27
  调用1 ret=10（cur 10）+ 调用2 ret=20（cur 30）+ 调用3 ret=0（cur 30）
  输出 27 字节，overread 挂账 3 字节（源 27..29）
帧 #2：源 [27, 60)，长度 33
  调用4 输入重读源 30 的 B6，ret=30（cur 60）
  其中源 30..59 共 30 字节新计费，源 27..29 的 3 字节来自 overread 回收（不重复计费）
  30 + 3 = 33 = 帧 #2 长度 ✓
```

每个交付帧的字节范围都与整块喂入时完全一致；overread 机制保证被“借读”的前缀字节既不会丢，也不会被计费两次。

对照实验：同样的源流若一次整块送入 `[0,40)`，阶段 A 在本块下标 3 命中 VOP，阶段 B 在本块下标 30（标识字节 `B6`）命中下一 VOP 码，直接返回 `next = 30-3 = 27`：输出 `buf[0..26]` 即源 `[0,27)` 的 27 字节，`buf[27..]` 原样留待下次，框架记录 `frame_offset=0, next_frame_offset=27, cur_offset=27`。两种喂法对同一帧给出的**源字节范围完全一致**，差别仅在于分块喂入时由 `pc->buffer` + overread 承担跨块重组。

**一般化结论**：

- 每次 `poutbuf_size>0` 交付一帧，该帧在源流中占据 `[parser->frame_offset, parser->next_frame_offset)`；
- 两次交付之间各次调用返回值之和等于该帧长度；若中间出现返回值 0（负 `next` 被钳零），说明发生了跨块起始码，差额以 `pc->overread`（MPEG-4 下为 1..3 字节）挂账，并在下一次调用开头回收，调用方指针因 `len==0` 不前进而重读标识字节；
- 字节不丢不重：暂存于 `pc->buffer` 的部分尚未形成交付，被借读的前缀字节通过 overread 回退并在下一帧缓冲中归位。

---

## 4. 交付给解码器的一帧里有什么：`mpeg4_decode_header()`

`mpeg4video_parser.c:83-120` 在每次成功拼出帧后调用 `ff_mpeg4_parse_picture_header()`（`mpeg4videodec.c:3454`），以 `parse_only=1` 只解析头、不初始化 VLC 表、不解码宏块。

### 4.1 parser 内部的起始码巡游

`ff_mpeg4_parse_picture_header` 在帧缓冲内再次扫描起始码（`mpeg4videodec.c:3477-3597`），按语义处理：

- `0x120–0x12F`：VOL 头 → `decode_vol_header()`，建立尺寸、`time_increment_bits`、`low_delay`、`data_partitioning` 等；
- `0x1B2`：user data（DivX/Xvid 版本探测等）；
- `0x1B3`：GOP 头（时间基准）；
- `0x1B0`：VOS，profile/level；
- `0x1B5`：Visual Object；
- `0x1B6`：**VOP → `break` 退出巡游，进入 `decode_vop_header()`**。

这意味着 parser 交付的一帧并不只是 VOP 负载：它可以包含 VOS/VO/VOL/GOP/user-data 等前导起始码段，解码器靠同一套巡游跳过它们。parser 与解码器对“帧”的定义一致：**从一个 VOP 起始码（含其前导头）到下一个非 slice/ext 起始码之前的全部字节**。

### 4.2 画面类型与关键帧证据

`decode_vop_header()` 开头（`mpeg4videodec.c:3089-3090`）：

```c
h->c.mcsel     = 0;
h->c.pict_type = get_bits(gb, 2) + AV_PICTURE_TYPE_I;  /* I=0, P=1, B=2(, S 复用) */
```

VOP 起始码 `00 00 01 B6` 之后的**前 2 个比特**就是 `vop_coding_type`：

| 2 比特值 | `pict_type` | 含义 |
|---|---|---|
| `00` | `AV_PICTURE_TYPE_I` (=1) | I-VOP，帧内编码 |
| `01` | `AV_PICTURE_TYPE_P` (=2) | P-VOP |
| `10` | `AV_PICTURE_TYPE_B` (=3) | B-VOP |
| `11` | `AV_PICTURE_TYPE_S` (=4) | S(GMC)-VOP |

parser 把它写回公开字段（`mpeg4video_parser.c:117`）：

```c
s1->pict_type = s->pict_type;
```

注意：**MPEG-4 parser 从不写 `s1->key_frame`**，该字段保持 `av_parser_init()` 设置的 `-1`（`parser.c:74`）。于是 demuxer 在 `demux.c:1320-1323` 走后备规则：

```c
if (sti->parser->key_frame == 1 ||
    (sti->parser->key_frame == -1 &&
     sti->parser->pict_type == AV_PICTURE_TYPE_I))
    out_pkt->flags |= AV_PKT_FLAG_KEY;
```

即 **`pict_type == AV_PICTURE_TYPE_I` 是 parser 层面对“可随机访问”的唯一证据**。解码器侧相互印证：`ff_mpv_frame_start()`（`mpegvideo_dec.c:376-379`）仅对 I 帧置 `AV_FRAME_FLAG_KEY`。

调用方可以据此把最后一幅画面标作可随机访问的**充分证据链**：

1. 该帧输出缓冲以 `00 00 01 B6`（或经前导头后包含该码）起始，parser 确实进入过阶段 B——即本次输出不是 EOF 兜底产生的“无 VOP 前导字节”包（见 5.2 第一行）；
2. `AVCodecParserContext.pict_type == AV_PICTURE_TYPE_I`，且该值来自本次成功的 VOP 头解析（等价于 VOP 码后首 2 比特为 `00`），而不是 `av_parser_init()` 留下的默认值——`pict_type` 初值恰好就是 `AV_PICTURE_TYPE_I`（`parser.c:68`），头解析失败时不会被改写，单凭字段相等不足以采信；实践中可要求该帧缓冲内确实扫描到 VOP 起始码且 `ff_mpeg4_parse_picture_header` 返回 0；
3. （流格式约束）该 I-VOP 之前带有可建立解码上下文的 VOL 头——要么在本帧缓冲内（parser 的头巡游会解析它），要么已通过 `extradata` 提供（`mpeg4video_parser.c:94-99` 在首帧时先解析 `avctx->extradata`）。缺少 VOL 上下文的 I-VOP 即使标记为 key，解码器也无法从该点独立起步。

仅有 GOP 起始码 `00 00 01 B3` **不构成**关键帧证据：MPEG-4 的 GOP 头只携带时间与 closed_gop/linked_gop 两个标志位，parser 不据此设置任何 key 标记；MPEG-4 也不保证 GOP 后第一帧一定是 I 帧。反过来，没有 GOP 头的 I-VOP 仍按规则标记为 key。

### 4.3 其他被提取的字段

- 尺寸：`mpeg4_decode_header` 在 VOL 解析得到 `s->width/height` 后调用 `ff_set_dimensions()`（`mpeg4video_parser.c:104-109`），并经 `av_parser_parse2` 的 `FILL` 宏写入 `avctx`（`parser.c:175-180`）。
- 延迟：`avctx->has_b_frames = !s->low_delay`（`mpeg4video_parser.c:103`），`low_delay` 来自 VOL 头（`mpeg4videodec.c:2646`）。
- 时间戳：若 demuxer 置了 `PARSER_FLAG_USE_CODEC_TS`（裸 m4v 等无容器时间戳场景，`demux.c:1519`），parser 用 VOP 头里的 modulo/time_increment 重建 PTS（`mpeg4video_parser.c:110-115`）。
- 未编码 VOP（`vop_coded == 0`）与 DivX 风格空包：头解析返回 `FRAME_SKIPPED`（`mpeg4videodec.c:3185-3189`），parser 仍交付该字节范围，但解码器侧 `ff_h263_decode_frame()` 直接 `return buf_size` 不出图（`h263dec.c:496-497`）。

---

## 5. 截断素材在结束输入（EOF flush）时的行为

### 5.1 parser 层：最后一个 VOP 靠 EOF 强制收口

两条路径汇合到同一语义：

1. demuxer 在 `ff_read_packet()` 返回错误/EOF 时，对每个带 parser 的流调用 `parse_packet(..., flush=1)`，传入空白包（`demux.c:1428-1434`）。
2. `av_parser_parse2()` 对 `buf_size == 0` 填入全零 dummy 缓冲（`parser.c:148-151`），仍调用 parser 的 `parse` 回调。
3. `mpeg4_find_frame_end()` 中，若 `vop_found` 为 1 且 `buf_size == 0`，**直接 `return 0`**（`mpeg4video_parser.c:62-65`，注释原文 “EOF considered as end of frame”）。
4. `ff_combine_frame()` 对“空输入 + `END_NOT_FOUND`”也有等价兜底：`if (!*buf_size && next == END_NOT_FOUND) next = 0;`（`parser.c:231-232`）。

`next = 0` 的含义：输出 `pc->buffer` 中累积的全部字节（`buf_size = pc->index + 0`），本次新输入消费 0 字节。于是**截断点之前最后一个已见 VOP 起始码之后收到的所有字节，无论后面有没有下一个起始码，都会作为一帧交付**。

### 5.2 截断素材能保留什么、丢失什么

设源流在一个 VOP 的宏块数据中途被截断。

| 截断位置 | parser 交付结果 |
|---|---|
| VOP 起始码之前（阶段 A 中 EOF） | `frame_start_found == 0`，普通调用一律返回 `END_NOT_FOUND`、字节全部累积进 `pc->buffer`，无输出；flush 的空输入同样返回 `END_NOT_FOUND`，由 `ff_combine_frame` 的 `!*buf_size && next==END_NOT_FOUND` 兜底（`parser.c:231-232`）归一为 `next=0`，**把缓冲里的 VOS/VOL 等前导字节作为一次输出交出**（仿真：8 字节前导 → flush 输出 8 字节）。`mpeg4_decode_header` 在其中找不到 VOP，巡游到比特末尾返回 `AVERROR_INVALIDDATA`（`mpeg4videodec.c:3486-3487`），但 `mpeg4video_parse` **不检查该返回值**（`mpeg4video_parser.c:152` 直接丢弃），缓冲照常交付。需要警惕的是 `AVCodecParserContext.pict_type` 的初值是 `AV_PICTURE_TYPE_I`（`parser.c:68`），头解析失败时该字段不会被更新，调用方若只看 `pict_type` 可能误判——因此随机访问判定必须要求头解析成功（实际拿到 VOP 码后的 `vop_coding_type`），不能只凭字段默认值。下游解码器收到无 VOP 的数据后在 `ff_h263_decode_frame` 的 `decode_header` 阶段报 “header damaged” 并返回错误（`h263dec.c:500-503`），**不产生画面**。字节层面仍被全部消费，不滞留。 |
| VOP 起始码之后、VOP 头完整、负载中途 | **交付一帧**：范围从该 VOP 码（含前导头）到截断点。仿真：20 字节处截断的流分两块喂入，两次 `END_NOT_FOUND` 累积后 flush 一次性输出全部 20 字节，首 4 字节即 `00 00 01 B6`。parser 头解析成功，`pict_type` 等元数据有效；解码器拿到的是不完整码流，由错误恢复（`ff_er_frame_end`，`h263dec.c:620`） conceal 未解码宏块，仍可输出一幅带损伤的画面，`ff_h263_decode_frame` 默认返回 `buf_size` 而非错误（`h263dec.c:661-664`，除非开了 `AV_EF_EXPLODE`）。 |
| VOP 头本身被截断（`get_bits_left < 3` 等） | parser 的 `decode_vop_header` 返回错误（如 `mpeg4videodec.c:3230-3233`），该帧不具备可信元数据；字节仍已被消费。 |
| 完整帧之后、下一 VOP 码之前 | 最后一个完整帧早已在看到下一起始码时正常交付；EOF 时 `pc->buffer` 为空或仅有新起始码的零碎前缀，flush 无额外有效输出。 |

解码器侧还有一层 EOF 行为（`h263dec.c:455-475`）：以 `buf_size == 0` 调用 `avcodec` 做 drain 时，若流以 NVOP（not-coded VOP）结尾或存在 B 帧重排，缓存的 `next_pic` 会在此时吐出——即 **parser 负责在 EOF 交出最后一个字节范围，解码器负责在 drain 时交出最后一幅显示顺序的画面**。

### 5.3 flush 后状态

demuxer 在 flush 完成后立即 `av_parser_close()` 释放 parser（`demux.c:1337-1340`），`ff_parse_close()` 释放 `pc->buffer`（`parser.c:300-305`）。不存在 flush 后继续喂数据的合法用法；需要重用应重新 `av_parser_init()`，状态从 `state=0, frame_start_found=0, index=0` 开始。

---

## 6. 调用方操作清单（任意大小块喂入的正确姿势）

1. 调 `av_parser_parse2(parser, avctx, &out, &out_size, chunk, chunk_size, pts, dts, pos)`，`chunk_size` 任意（1 字节也合法，状态机全靠持久化的 `state`/`frame_start_found`）。
2. **前进指针只看返回值 `ret`**：下一次送入 `chunk + ret`，剩余长度 `chunk_size - ret`。`ret` 与 `out_size` 没有相等关系——跨块帧的多次调用里 `ret > 0` 而 `out_size == 0` 是常态。
3. `out_size > 0` 时，`out[0 .. out_size-1]` 是一帧的完整字节（可能指向内部 `pc->buffer`，其生命周期到下次调用为止，需持久保存请自行拷贝）；该帧在源流中的起点是 `parser->frame_offset`，下一帧起点是 `parser->next_frame_offset`。
4. 时间戳用 `parser->pts/dts/pos`，不要用输入块自带的时间戳（块与帧不再对齐；框架的 `ff_fetch_timestamp` 按字节区间匹配，`parser.c:89-118`）。
5. 结束输入时再调一次 `av_parser_parse2(..., NULL, 0, ...)`（或经 demuxer 的 flush 路径），逼出 `pc->buffer` 里最后的 VOP；判断最后一幅画面可否随机访问，以 `parser->pict_type == AV_PICTURE_TYPE_I` 为准（demuxer 自动转成 `AV_PKT_FLAG_KEY`，直接用 API 的调用方需自行检查此字段），并确认 VOL 上下文可用（同帧内含 VOL 或 extradata 已提供）。
6. `PARSER_FLAG_COMPLETE_FRAMES` 模式下（容器已分帧，如 MP4/AVI 中的 MPEG-4）边界扫描整体跳过：`next = buf_size`，每进必出，EOF 不会有额外 flush 帧；本文第 2、5 节的边界语义仅适用于默认的裸流解析模式。

---

## 附：关键代码位置索引

| 结论 | 位置 |
|---|---|
| VOP 起始码 4 字节精确匹配 | `mpeg4video_parser.c:52-59` |
| 任意起始码判定 `(state & 0xFFFFFF00) == 0x100` | `mpeg4video_parser.c:68` |
| B7/B8 帧内码豁免 | `mpeg4video_parser.c:69-70` |
| 帧结束返回 `i-3`（下一码前缀首字节下标） | `mpeg4video_parser.c:71-73` |
| EOF 视为帧结束 | `mpeg4video_parser.c:63-65`；`parser.c:231-232` |
| 不完整帧累积、零拷贝与 overread | `parser.c:213-298` |
| 消费偏移记账 `cur_offset/next_frame_offset` | `parser.c:186-200` |
| `vop_coding_type` 2 比特 → `pict_type` | `mpeg4videodec.c:3089-3090` |
| parser 公开 `pict_type`，不写 `key_frame` | `mpeg4video_parser.c:117`；`parser.c:74` |
| I 帧 → `AV_PKT_FLAG_KEY` 的后备规则 | `demux.c:1320-1323` |
| 解码器侧 I 帧 → `AV_FRAME_FLAG_KEY` | `mpegvideo_dec.c:376-379` |
| EOF flush 触发点 | `demux.c:1428-1434` |
| drain 吐出缓存帧/NVOP 末帧 | `h263dec.c:455-475` |
