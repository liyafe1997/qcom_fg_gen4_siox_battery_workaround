# About this KPM

**English** | [简体中文](#简体中文)

A [SukiSU-Ultra](https://github.com/SukiSU-Ultra/SukiSU-Ultra) / [KernelPatch](https://github.com/bmax121/KernelPatch)
**KPM** (Kernel Patch Module) that lowers the Qualcomm **QPNP FG-Gen4**
fuel-gauge discharge **floor** — cutoff `3400→3250`, empty `3100→3000`, and the
*effective shutdown voltage* `3300→3150` — entirely at runtime, in kernel space,
with **no kernel source edits**.

**Applicability.** Developed and verified on the **Redmi K30 Pro (codename
`lmi`)**, and should apply to any device whose kernel drives its fuel gauge with
the same **Qualcomm FG-Gen4** driver, provided the `qpnp-fg-gen4.c` symbols it
hooks are present and named the same. Nothing device-specific is compiled in:
the module locates the driver's parameter block by *shape* and confirms it
against the value the device itself has programmed into FG SRAM (see
[How it works](#how-it-works-device-independent)). Different PMIC/driver
generations (FG-Gen3, QG/QGauge) are **not** supported.

## Why

The stock driver anchors the fuel gauge's **0%** point at "voltage reaches
3.4 V under a 200 mA load". That is fine for a graphite cell (little charge left
below 3.4 V), but a **silicon/carbon (Si/SiOx) anode** cell keeps a long, flat
tail below 3.4 V — roughly **10–15% of nominal capacity**. Under the stock
framework the phone reports 0% / shuts down while that tail is still full, so a
higher-capacity Si/C replacement pack never delivers its rated mAh.

## The floor is a *set* of parameters, not one value

The **effective shutdown voltage is `SHUTDOWN_DELAY_VOL` (stock 3300 mV), not
`cutoff`** — when the computed SOC hits 0 the driver keeps reporting 1% until
the voltage falls below `SHUTDOWN_DELAY_VOL`. So lowering `cutoff` alone barely
moves the real floor; the whole set has to come down together.

**Conservative Si/C tier — everything is done from the KPM, no kernel source
edits:**

| Parameter | Stock | New | Role | How the KPM does it |
|-----------|-------|-----|------|---------------------|
| `cutoff_volt_mv` | 3400 | **3250** | FVSS 0% scale + hardware `msoc=0%` anchor | DT field write + SRAM word 20 |
| `empty_volt_mv` | 3100 | **3000** | vbatt-low IRQ threshold → arms rapid-SOC | DT field write + SRAM word 35 |
| `SHUTDOWN_DELAY_VOL` (effective) | 3300 | **3150** | the **true** shutdown voltage | functional hook on `fg_psy_get_property` |
| `VBAT_CRITICAL_LOW_THR` | 2800 | *(left stock)* | rapid-SOC immediate-trip floor | **subsumed** by the shutdown hook |

Two DT-field parameters are written directly (they are the **root of truth** —
on any FG profile reload the driver re-derives the SRAM anchors from them — and
the KPM also writes SRAM so the change is immediate).

`SHUTDOWN_DELAY_VOL` is a compile-time `#define`, so instead of patching the
opcode the KPM **hooks `fg_psy_get_property`**: when the driver reports
`CAPACITY == 0` while instantaneous vbatt is still above the target floor, the
hook rewrites the result to `1%`. That extends the stock "hang at 1%" window
from 3300 mV down to 3150 mV **functionally** — robust, no instruction-stream
assumptions.

That same hook also **masks a premature rapid-SOC trip** (which forces
`msoc→0`): the report stays 1% until vbatt genuinely drops below the floor. So
the effective floor is 3150 mV regardless of when rapid latches, which is why
lowering `VBAT_CRITICAL_LOW_THR` is unnecessary and that macro is left at stock.

## Low-temperature tier

Cold raises cell impedance, so the **loaded** terminal voltage sags well below
OCV. Holding the warm floor in the cold therefore throws away real charge. Stock
already handles this — below **15.0 °C** it shifts its whole set down 200 mV:

| | warm | cold (< 15.0 °C) |
|---|---|---|
| `cutoff_volt_mv` | 3400 | 3200 |
| `SHUTDOWN_DELAY_VOL` | 3300 | 3100 |
| FVSS entry (`vbatt_scale`) | 3600 | 3400 |

The KPM does the same, but by a **smaller default step (−100 mV)**, applied
uniformly to all three of its floors so the cutoff→shutdown "hang at 1%" window
keeps its width:

| | warm | cold (default `lt=100`) |
|---|---|---|
| cutoff | 3250 | **3150** |
| empty | 3000 | **2900** |
| shutdown floor | 3150 | **3050** |

**Why not simply copy stock's −200 mV.** In the cold the binding constraint is
not cell damage — at these terminal voltages OCV is much higher, so true
depth-of-discharge stays modest — it is **system brown-out**, because the same
impedance that justifies going lower also makes load spikes sag harder. What
matters is headroom above `sys_min_volt_mv` (2800 mV on `lmi`) at the shutdown
floor:

| | headroom over sys_min |
|---|---|
| stock warm 3300 | 500 mV |
| stock cold 3100 | 300 mV |
| ours warm 3150 | 350 mV |
| ours cold `lt=100` → 3050 | **250 mV** |
| ours cold `lt=150` → 3000 | 200 mV (aggressive) |
| ours cold `lt=200` → 2950 | 150 mV — half the OEM's cold margin, avoid |

Si/C is a reason to keep *more* margin, not less: SiOx-blended anodes have
higher impedance and worse cold behaviour than graphite, so they sag harder
exactly when you are closest to the floor. And the tail steepens near empty, so
the extra 100 mV buys less charge than the number suggests.

The tier is applied **whether or not** the driver runs its own
`qcom,cutoff-voltage-adjust-enable` path (`umi` sets it, `lmi` does not). Where
the driver does run it, the KPM wins the race simply by re-asserting on every
voltage read — far more often than the driver's 10 s monitor tick — and
re-asserts SRAM too, not just the C field. Unlike stock, the KPM applies
**hysteresis** (leave the cold tier only above 17.0 °C); stock flips at exactly
15.0 °C on a timer, so a battery resting at the threshold makes it oscillate.

Battery temperature comes from `fg_gen4_get_battery_temp` (deci-degrees C),
sampled at most once every 16 voltage reads. If that symbol is missing the tier
is disabled and the warm floors are used throughout.

## How it works (device-independent)

KPMs run in kernel space and can inline-hook kernel functions. This module
installs two hooks:

**Hook 1 — `fg_get_battery_voltage(struct fg_dev *fg, int *val)`** (locate +
apply cutoff/empty):

1. It is *exported* and called frequently. Because `struct fg_dev fg` is the
   **first member** of `struct fg_gen4_chip`, `arg0 == the chip base pointer` —
   no struct offset needed to obtain `chip`.
2. On the first call, **scans** the chip object for the dt block. `struct
   fg_dt_props` opens with six ints in a fixed order, so the scan matches on
   that *shape* rather than on any device's DT values:

   | idx | field | test |
   |----|-------|------|
   | 0 | `cutoff_volt_mv` | plausible mV (2000–4500) — **written** |
   | 1 | `empty_volt_mv` | plausible mV, `≤` idx0 — **written** |
   | 2 | `sys_min_volt_mv` | plausible mV |
   | 3 | `cutoff_curr_ma` | **positive** mA |
   | 4 | `sys_term_curr_ma` | **negative** mA |
   | 5 | `ffc_sys_term_curr_ma` | negative mA (`-EINVAL` when absent) |

   The candidate is then confirmed against **the device's own data**: the module
   reads back `CUTOFF_VOLT` from FG SRAM via `fg_sram_read`, decodes it to mV,
   and requires `dt.cutoff_volt_mv` to agree (±4 mV covers the ~1 mV round-trip
   quantisation). A candidate is accepted **only if it is unique** in the whole
   window; if two or more survive, all are logged and **nothing is written**.
   All probing uses `probe_kernel_read`, so a bad address returns `-EFAULT`
   instead of panicking.
3. Writes both C fields and encodes+writes both SRAM words.
4. Restores stock values on unload.

**Hook 2 — `fg_psy_get_property(psy, psp, val)`** (shutdown floor): an AFTER
hook that, for `psp == POWER_SUPPLY_PROP_CAPACITY` returning 0, reports 1% while
vbatt is above the floor (see the table above).

Symbols (`probe_kernel_read`, `fg_sram_write`, `fg_sram_read`,
`fg_get_battery_voltage`, `fg_psy_get_property`) are resolved via
`kallsyms_lookup_name`.

`POWER_SUPPLY_PROP_CAPACITY` is an enum ordinal that shifts between trees, so it
is **resolved at runtime by name**, not hard-coded: the kernel's
`power_supply_attrs[]` table is indexed by that same enum and each entry starts
with a `const char *name`, so the module walks it for `"capacity"`. It
self-validates by requiring index 0 to be `"status"` (which also pins down the
`struct device_attribute` stride, since `CONFIG_DEBUG_LOCK_ALLOC` changes it).
If that lookup ever fails it falls back to `44` and says so; `psp=<n>` forces it.

## Build

Requirements:
- an AArch64 compiler — either Android/NDK **clang** (LLVM) or a bare-metal
  **gcc** cross toolchain
- a KernelPatch checkout for its headers — `build.sh` auto-clones
  [SukiSU_KernelPatch_patch](https://github.com/SukiSU-Ultra/SukiSU_KernelPatch_patch),
  the fork whose `kpimg` actually loads the module (override with `KP_REPO=`)

**Clang** (verified with Android `clang-r487747c`, clang 17 — `ld.lld` included):

```bash
CLANG=$HOME/clang-android/clang-r487747c/bin ./build.sh
# or manually:
make CLANG=$HOME/clang-android/clang-r487747c/bin \
     TARGET=aarch64-linux-gnu KP_DIR=/path/to/KernelPatch
```

**GCC**:

```bash
TARGET_COMPILE=aarch64-linux-gnu- ./build.sh
# or manually:
make TARGET_COMPILE=aarch64-linux-gnu- KP_DIR=/path/to/KernelPatch
```

Output: `fg_cutoff.kpm` — an AArch64 relocatable ELF carrying the
`.kpm.info/.init/.ctl0/.exit` sections the KernelPatch loader maps.

### Why `-mgeneral-regs-only` is mandatory

A KPM runs in kernel context, and arm64 Linux does not save/restore FP/SIMD
state on kernel entry -- which is why the kernel itself is built with this flag.
Without it clang happily auto-vectorises ordinary integer code into NEON, which
is both unsound here and triggers a silent relocation bug:

* the vectorised constants land in `.rodata.cst16`, which requires 16-byte
  alignment, and are reached via `R_AARCH64_LDST128_ABS_LO12_NC`;
* kpimg allocates modules with `kp_malloc_exec()`, which only guarantees
  **8-byte** alignment (hence `.text` at `<base>+8`). Section offsets are aligned
  relative to that base, so a 16-byte-aligned section is misaligned absolutely;
* that relocation's `imm12` is scaled by 16, so the low bits are **silently
  dropped** and the load reads 8 bytes short of the constant.

The module still loads and the hooks still fire -- it just compares against
garbage. This cost a full debugging cycle: the symptom was an endless
`fingerprint not found` while the fingerprint data was provably correct.
`build.sh` now fails the build on any over-aligned section or `LDST128`
relocation.

### Undefined symbols

`build.sh` fails the build if the module references a symbol `kpimg` does not
export. This matters because the KPM loader resolves undefined symbols *only*
against its own `KP_EXPORT_SYMBOL` table — the `kallsyms_lookup_name()` fallback
in `simplify_symbols()` is commented out both upstream and in SukiSU's fork.
Pulling in an ordinary kernel function (`snprintf`, `memcpy`, ...) therefore
makes the loader bail out with `unknown symbol: <name>` before `init` ever runs,
which a manager surfaces only as a failed load. Format and copy in-module, or
resolve the symbol yourself at runtime via `kallsyms_lookup_name` (which *is*
exported).

## Load / control / unload

Using the KernelPatch userspace tool `kpm` (SukiSU ships an equivalent; adjust
to your manager). A superkey is required by KernelPatch.

```bash
# load with the defaults (cutoff 3250, empty 3000, shutdown 3150 mV)
kpm load  /data/local/tmp/fg_cutoff.kpm

# load with explicit targets "cutoff[,empty[,shutdown]]" (mV) as module args
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150"

# change targets at runtime (cutoff only, or more)
kpm control battery-fg-cutoff "3200"
kpm control battery-fg-cutoff "3250,3000,3150"

# force the CAPACITY ordinal if the by-name lookup ever fails on your tree
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,psp=44"

# low-temperature tier: step (mV, 0-200) and threshold (deci-degrees C)
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,lt=150"     # aggressive
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,lt=0"       # disable
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,lttemp=100" # cold below 10.0 C

# unload (restores stock cutoff 3400 / empty 3100; shutdown hook removed)
kpm unload battery-fg-cutoff
```

Clamps: `cutoff` **[2800, 3400]**, `empty` **[2500, 3300]**,
`shutdown` **[2800, 3300]** mV.

To make it persistent, load it from a SukiSU/KernelPatch boot service (the same
place you load other KPMs).

## Verify

Watch the kernel log after loading:

```bash
dmesg | grep fg-cutoff-kpm
```

Expected, within a second or two of the first FG voltage poll:

```
[fg-cutoff-kpm] init (event=..., args=) cutoff=3250 empty=3000 shutdown=3150 mV
[fg-cutoff-kpm] syms: pkr=... sram_write=... get_voltage=... get_property=...
[fg-cutoff-kpm] hooks installed; applying on next FG voltage read
[fg-cutoff-kpm] SRAM CUTOFF_VOLT <- 3250 mV (raw ff 33) rc=0
[fg-cutoff-kpm] SRAM VBATT_LOW  <- 3000 mV (raw 40) rc=0
[fg-cutoff-kpm] applied: cutoff 3400->3250, empty 3100->3000, shutdown floor 3150 mV (dt @ ...)
```

If `get_property=0000...` (symbol not found), cutoff/empty still apply but the
shutdown floor stays at the stock 3300 mV.

You can cross-check the live SRAM values if `CONFIG_DEBUG_FS` is enabled on your
build:

```bash
# /sys/kernel/debug/fg/sram
#   word 20 = CUTOFF_VOLT (2 bytes LE)   3250 mV -> ff 33
#   word 35 byte 1 = VBATT_LOW (1 byte)  3000 mV -> 40
```

## Limitations / caveats

- **What is still device-specific.** The dt scan and the CAPACITY ordinal are
  both derived at runtime, but the **FG SRAM layout is not**: `CUTOFF_VOLT`
  (word 20) and `VBATT_LOW` (word 35 byte 1) with their scaling factors are
  taken from the driver's `pm8150b_v1`/`v2` parameter tables — identical in
  both, but a different PMIC could differ. If your PMIC uses another table,
  those four `#define` groups need updating (and the SRAM cross-check would
  simply fail to confirm, so the module refuses to write rather than writing
  somewhere wrong).
- **Failure is safe, not silent.** If the scan finds nothing, or finds more than
  one candidate, the module writes nothing and logs why; a rate-limited
  diagnostic dump prints the candidate windows it saw.
- **`VBAT_CRITICAL_LOW_THR` left at stock (2800).** The shutdown hook already
  masks a premature rapid-SOC 0% down to the floor, so lowering it is redundant
  for the user-visible floor. (Its only residual effect is *when* rapid latches
  and reprograms the SRAM slope/cutoff-current, which self-restores on charge.)
- **Not a battery profile.** Lowering the floor lets the existing (4700 mAh
  graphite) profile run further down the curve; it does **not** correct the
  percentage curve shape for a Si/C cell. For accurate mid-range SOC you still
  need a matching `fg-profile-data`.
- **The cold tier is the least validated part.** The right step depends on your
  cell's actual DCIR at temperature, which varies by pack. Validate `lt=` at the
  coldest temperature you care about *under load* before trusting it; the
  failure mode is a brown-out reboot, not a clean shutdown.
- **Safety.** Do not set the floor below the cell's real spec or the PMIC UVLO.
  Over-discharge damages cells; too low a floor risks brown-out/reboot under
  load (the cell sags below system-min). Validate the aggressive end on-device
  at high load and cold temperature. Clamp floors: cutoff 2800, empty 2500 mV.
- Verified against the FG-Gen4 driver in this tree
  (`drivers/power/supply/qcom/qpnp-fg-gen4.c`, `fg-util.c`, `fg-core.h`) on
  `lmi` / kernel 4.19.325.

## License

GPL-2.0-or-later (matches the Linux kernel and KernelPatch).

---

# 简体中文

[English](#about-this-kpm)

一个 [SukiSU-Ultra](https://github.com/SukiSU-Ultra/SukiSU-Ultra) / [KernelPatch](https://github.com/bmax121/KernelPatch)
的 **KPM**（Kernel Patch Module），用于降低高通 **QPNP FG-Gen4** 电量计的放电**地板**
——cutoff `3400→3250`、empty `3100→3000`、以及*有效关机电压* `3300→3150`——
全部在运行时于内核态完成，**不改任何内核源码**。

**适用范围。** 在 **红米 K30 Pro（代号 `lmi`）** 上开发并验证，同时应该适用于**其它使用
高通 FG-Gen4 电量计的机型**，前提是内核用的是同一套 `qpnp-fg-gen4.c` 驱动、且它 hook
的那几个符号存在且同名。模块内**不写死任何机型相关的值**：它按*结构形状*定位驱动的参数块，
再用设备自己烧进 FG SRAM 的值来确认（见[工作原理](#工作原理与机型无关)）。
不同代的 PMIC / 驱动（FG-Gen3、QG/QGauge）**不支持**。

## 为什么

原厂驱动把电量计的 **0%** 点锚定在“电压在 200 mA 负载下降到 3.4 V”。这对石墨电芯没问题
（3.4 V 以下几乎没电了），但**硅碳（Si/SiOx）负极**电芯在 3.4 V 以下还有一条又长又平的尾巴，
大约占标称容量的 **10–15%**。在原厂框架下，这条尾巴还满着时手机就报 0% / 关机，于是更高容量的
硅碳替换电池永远发挥不出它标称的 mAh。

## 地板是一*组*参数，不是单个值

**有效关机电压是 `SHUTDOWN_DELAY_VOL`（原厂 3300 mV），不是 `cutoff`**——当计算出的 SOC
归零后，驱动会一直报 1%，直到电压跌破 `SHUTDOWN_DELAY_VOL`。所以只降 `cutoff` 几乎不动真正的地板；
整组参数必须一起往下降。

**保守的硅碳档位——全部由 KPM 完成，不改内核源码：**

| 参数 | 原厂 | 新值 | 作用 | KPM 如何实现 |
|-----------|-------|-----|------|---------------------|
| `cutoff_volt_mv` | 3400 | **3250** | FVSS 0% 标度 + 硬件 `msoc=0%` 锚点 | 写 DT 字段 + SRAM word 20 |
| `empty_volt_mv` | 3100 | **3000** | vbatt-low 中断阈值 → 触发 rapid-SOC | 写 DT 字段 + SRAM word 35 |
| `SHUTDOWN_DELAY_VOL`（有效） | 3300 | **3150** | **真正的**关机电压 | 对 `fg_psy_get_property` 做功能性 hook |
| `VBAT_CRITICAL_LOW_THR` | 2800 | *（保持原厂）* | rapid-SOC 立即触发地板 | 被关机 hook **一并覆盖** |

两个 DT 字段参数直接写入（它们是**真值之源**——每次 FG profile 重载时驱动都会从它们重新推导
SRAM 锚点——同时 KPM 也写 SRAM，使改动立即生效）。

`SHUTDOWN_DELAY_VOL` 是一个编译期 `#define`，因此 KPM 不去打补丁改指令，而是 **hook
`fg_psy_get_property`**：当驱动报 `CAPACITY == 0` 而瞬时 vbatt 仍高于目标地板时，hook 把结果改写为
`1%`。这从**功能上**把原厂“卡在 1%”的窗口从 3300 mV 延伸到 3150 mV——稳健，不对指令流做任何假设。

同一个 hook 还**掩盖了 rapid-SOC 的提前触发**（它会强制 `msoc→0`）：报告会一直保持 1%，直到 vbatt
真正跌破地板。所以无论 rapid 何时锁存，有效地板都是 3150 mV，这也是为什么无需降低
`VBAT_CRITICAL_LOW_THR`，该宏保持原厂值。

## 低温档位

低温会抬高电芯内阻，**带载**端电压因此远低于 OCV。此时若仍守着常温地板，就等于白白丢掉真实电量。
原厂本身就处理了这一点——低于 **15.0℃** 时整套下移 200 mV：

| | 常温 | 低温（< 15.0℃） |
|---|---|---|
| `cutoff_volt_mv` | 3400 | 3200 |
| `SHUTDOWN_DELAY_VOL` | 3300 | 3100 |
| FVSS 进入阈值（`vbatt_scale`） | 3600 | 3400 |

本 KPM 做同样的事，但**默认步长更小（−100 mV）**，并且三个地板统一下移，
使 cutoff→shutdown 的「卡在 1%」窗口宽度保持不变：

| | 常温 | 低温（默认 `lt=100`） |
|---|---|---|
| cutoff | 3250 | **3150** |
| empty | 3000 | **2900** |
| 关机地板 | 3150 | **3050** |

**为什么不直接照抄原厂的 −200 mV。** 低温下真正的约束不是电芯损伤——这种端电压下 OCV 高得多，
真实放电深度其实不大——而是**系统掉电重启**：正是那个「值得往下探」的内阻，同时让负载尖峰塌得更狠。
关键指标是关机地板相对 `sys_min_volt_mv`（lmi 上是 2800 mV）的余量：

| | 相对 sys_min 的余量 |
|---|---|
| 原厂常温 3300 | 500 mV |
| 原厂低温 3100 | 300 mV |
| 本模块常温 3150 | 350 mV |
| 本模块低温 `lt=100` → 3050 | **250 mV** |
| 本模块低温 `lt=150` → 3000 | 200 mV（激进） |
| 本模块低温 `lt=200` → 2950 | 150 mV——只有原厂低温余量的一半，不建议 |

硅碳恰恰是**该多留余量**而非少留的理由：掺 SiOx 的负极内阻更高、低温表现比石墨更差，
也就是说你最接近地板时它塌得最狠。而且尾段曲线会变陡，多下探这 100 mV 拿到的电量比数字看上去要少。

无论驱动自身的 `qcom,cutoff-voltage-adjust-enable` 是否生效（umi 有、lmi 没有），这个档位**都会应用**。
在驱动也会改的机型上，KPM 靠「每次读电压都重新写回」赢下这场竞争——频率远高于驱动 10 秒一次的
monitor tick——而且连 SRAM 一起写回，不只是 C 字段。与原厂不同，KPM 带**滞回**
（要高于 17.0℃ 才退出低温档）；原厂在定时器里按 15.0℃ 硬翻转，电池温度正好卡在阈值上时会来回抖。

电池温度取自 `fg_gen4_get_battery_temp`（单位为 0.1℃），最多每 16 次电压读取采样一次。
若该符号不存在，则关闭低温档、全程使用常温地板。

## 工作原理（与机型无关）

KPM 运行在内核态，可以对内核函数做 inline hook。本模块安装两个 hook：

**Hook 1 —— `fg_get_battery_voltage(struct fg_dev *fg, int *val)`**（定位 + 应用 cutoff/empty）：

1. 它是*导出的*，且被频繁调用。由于 `struct fg_dev fg` 是 `struct fg_gen4_chip` 的**第一个成员**，
   所以 `arg0 == chip 基址指针`——无需任何结构体偏移即可拿到 `chip`。
2. 首次调用时**扫描** chip 对象寻找 dt 块。`struct fg_dt_props` 开头是固定顺序的六个 int，
   所以扫描匹配的是这个*形状*，而不是任何机型的具体 DT 值：

   | 序号 | 字段 | 判据 |
   |----|-------|------|
   | 0 | `cutoff_volt_mv` | 合理电压 mV（2000–4500）——**会被写** |
   | 1 | `empty_volt_mv` | 合理电压 mV，且 `≤` 序号 0——**会被写** |
   | 2 | `sys_min_volt_mv` | 合理电压 mV |
   | 3 | `cutoff_curr_ma` | **正**数 mA |
   | 4 | `sys_term_curr_ma` | **负**数 mA |
   | 5 | `ffc_sys_term_curr_ma` | 负数 mA（DT 缺省时为 `-EINVAL`） |

   随后用**设备自己的数据**确认候选：模块通过 `fg_sram_read` 回读 FG SRAM 里的
   `CUTOFF_VOLT`，解码成 mV，要求 `dt.cutoff_volt_mv` 与之吻合（±4 mV，足以覆盖约
   1 mV 的往返量化误差）。只有在整个扫描窗口内**唯一**的候选才会被接受；若有两个以上
   幸存，全部记入日志并且**什么都不写**。所有探测都用 `probe_kernel_read`，因此坏地址
   返回 `-EFAULT` 而不会 panic。
3. 同时写两个 C 字段，并编码+写入两个 SRAM word。
4. 卸载时恢复原厂值。

**Hook 2 —— `fg_psy_get_property(psy, psp, val)`**（关机地板）：一个 AFTER hook，当
`psp == POWER_SUPPLY_PROP_CAPACITY` 返回 0 时，在 vbatt 高于地板期间报告 1%（见上表）。

符号（`probe_kernel_read`、`fg_sram_write`、`fg_sram_read`、`fg_get_battery_voltage`、
`fg_psy_get_property`）通过 `kallsyms_lookup_name` 解析。

`POWER_SUPPLY_PROP_CAPACITY` 是一个会随内核树变化的枚举序号，因此它**在运行时按名字解析**，
而不是写死：内核的 `power_supply_attrs[]` 表正是以该枚举为下标，且每个条目以
`const char *name` 开头，所以模块遍历它查找 `"capacity"`。它会自校验——要求下标 0 必须是
`"status"`（这同时也确定了 `struct device_attribute` 的步长，因为 `CONFIG_DEBUG_LOCK_ALLOC`
会改变它）。万一查找失败，会回退到 `44` 并在日志中说明；用 `psp=<n>` 可强制指定。

## 构建

需求：
- 一个 AArch64 编译器——Android/NDK **clang**（LLVM），或裸机 **gcc** 交叉工具链
- 一个 KernelPatch 检出以获取头文件——`build.sh` 会自动克隆
  [SukiSU_KernelPatch_patch](https://github.com/SukiSU-Ultra/SukiSU_KernelPatch_patch)，
  即真正加载本模块的那个 `kpimg` 分支（可用 `KP_REPO=` 覆盖）

**Clang**（已用 Android `clang-r487747c`，clang 17 验证——自带 `ld.lld`）：

```bash
CLANG=$HOME/clang-android/clang-r487747c/bin ./build.sh
# 或手动：
make CLANG=$HOME/clang-android/clang-r487747c/bin \
     TARGET=aarch64-linux-gnu KP_DIR=/path/to/KernelPatch
```

**GCC**：

```bash
TARGET_COMPILE=aarch64-linux-gnu- ./build.sh
# 或手动：
make TARGET_COMPILE=aarch64-linux-gnu- KP_DIR=/path/to/KernelPatch
```

输出：`fg_cutoff.kpm`——一个携带 `.kpm.info/.init/.ctl0/.exit` 段的 AArch64 可重定位 ELF，
由 KernelPatch 加载器映射。

### 为什么 `-mgeneral-regs-only` 是强制的

KPM 运行在内核上下文中，而 arm64 Linux 在进入内核时不会保存/恢复 FP/SIMD 状态——这正是内核自身
用这个标志编译的原因。不加它，clang 会欣然把普通整数代码自动向量化成 NEON，这既在此处不成立，
还会触发一个静默的重定位 bug：

* 向量化后的常量落进 `.rodata.cst16`，它要求 16 字节对齐，并通过
  `R_AARCH64_LDST128_ABS_LO12_NC` 寻址；
* kpimg 用 `kp_malloc_exec()` 分配模块，只保证 **8 字节**对齐（因此 `.text` 落在 `<base>+8`）。
  段偏移是相对该基址对齐的，所以一个 16 字节对齐的段在绝对地址上是错位的；
* 该重定位的 `imm12` 按 16 缩放，于是低位被**静默丢弃**，加载读到的地址比常量早 8 字节。

模块照样加载、hook 照样触发——只是拿垃圾值做比较。这耗掉了一整个调试周期：症状是无休止的
`fingerprint not found`，而指纹数据明明是正确的。`build.sh` 现在会在出现任何过度对齐的段或
`LDST128` 重定位时让构建失败。

### 未定义符号

如果模块引用了 `kpimg` 未导出的符号，`build.sh` 会让构建失败。这一点很关键，因为 KPM 加载器*只*
用它自己的 `KP_EXPORT_SYMBOL` 表来解析未定义符号——`simplify_symbols()` 里的
`kallsyms_lookup_name()` 兜底在上游和 SukiSU 的分支中都被注释掉了。因此引入一个普通内核函数
（`snprintf`、`memcpy` 等）会让加载器在 `init` 运行前就以 `unknown symbol: <name>` 退出，而管理器
只会显示为加载失败。请在模块内自行格式化和拷贝，或在运行时用 `kallsyms_lookup_name`
（它*是*导出的）自己解析符号。

## 加载 / 控制 / 卸载

使用 KernelPatch 的用户态工具 `kpm`（SukiSU 提供等价工具；请按你的管理器调整）。KernelPatch 需要
superkey。

```bash
# 用默认值加载（cutoff 3250、empty 3000、shutdown 3150 mV）
kpm load  /data/local/tmp/fg_cutoff.kpm

# 以模块参数 "cutoff[,empty[,shutdown]]"（mV）显式指定目标加载
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150"

# 运行时修改目标（仅 cutoff，或更多）
kpm control battery-fg-cutoff "3200"
kpm control battery-fg-cutoff "3250,3000,3150"

# 万一按名字查找在你的树上失败，可强制指定 CAPACITY 序号
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,psp=44"

# 低温档位：步长（mV，0-200）与阈值（0.1℃）
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,lt=150"     # 激进
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,lt=0"       # 关闭低温档
kpm load  /data/local/tmp/fg_cutoff.kpm "3250,3000,3150,lttemp=100" # 低于 10.0℃ 算低温

# 卸载（恢复原厂 cutoff 3400 / empty 3100；移除关机 hook）
kpm unload battery-fg-cutoff
```

限幅：`cutoff` **[2800, 3400]**、`empty` **[2500, 3300]**、`shutdown` **[2800, 3300]** mV。

若要持久化，请在 SukiSU/KernelPatch 的开机服务里加载它（和你加载其它 KPM 的地方一样）。

## 验证

加载后观察内核日志：

```bash
dmesg | grep fg-cutoff-kpm
```

在首次 FG 电压轮询后一两秒内，预期看到：

```
[fg-cutoff-kpm] init (event=..., args=) cutoff=3250 empty=3000 shutdown=3150 mV
[fg-cutoff-kpm] syms: pkr=... sram_write=... get_voltage=... get_property=...
[fg-cutoff-kpm] hooks installed; applying on next FG voltage read
[fg-cutoff-kpm] SRAM CUTOFF_VOLT <- 3250 mV (raw ff 33) rc=0
[fg-cutoff-kpm] SRAM VBATT_LOW  <- 3000 mV (raw 40) rc=0
[fg-cutoff-kpm] applied: cutoff 3400->3250, empty 3100->3000, shutdown floor 3150 mV (dt @ ...)
```

若 `get_property=0000...`（符号未找到），cutoff/empty 仍会应用，但关机地板保持原厂 3300 mV。

如果你的构建启用了 `CONFIG_DEBUG_FS`，可以交叉核对实时 SRAM 值：

```bash
# /sys/kernel/debug/fg/sram
#   word 20 = CUTOFF_VOLT（2 字节 LE）    3250 mV -> ff 33
#   word 35 byte 1 = VBATT_LOW（1 字节）  3000 mV -> 40
```

## 局限 / 注意事项

- **仍然与机型相关的部分。** dt 扫描和 CAPACITY 序号都已改为运行时推导，但 **FG SRAM
  布局没有**：`CUTOFF_VOLT`（word 20）和 `VBATT_LOW`（word 35 byte 1）及其换算系数取自驱动的
  `pm8150b_v1`/`v2` 参数表——两张表里完全一致，但换个 PMIC 就可能不同。如果你的 PMIC 用的是
  另一张表，需要更新那四组 `#define`（而且此时 SRAM 交叉校验会确认不上，模块会拒绝写入，
  而不是写到错误的地方）。
- **失败是安全的，而且不静默。** 如果扫描什么都没找到，或找到多于一个候选，模块不写任何东西
  并记录原因；一个带频率限制的诊断转储会打印它看到的候选窗口。
- **`VBAT_CRITICAL_LOW_THR` 保持原厂（2800）。** 关机 hook 已经把提前的 rapid-SOC 0% 掩盖到地板，
  因此对用户可见的地板而言降低它是多余的。（它唯一残留的作用是影响 rapid *何时*锁存并重编 SRAM
  斜率/cutoff-current，而这会在充电时自恢复。）
- **这不是电池 profile。** 降低地板让现有的（4700 mAh 石墨）profile 沿曲线跑得更深；它**不会**
  为硅碳电芯修正百分比曲线的形状。要获得准确的中段 SOC，仍需一份匹配的 `fg-profile-data`。
- **低温档是验证最少的部分。** 合适的步长取决于你这颗电芯在对应温度下的真实 DCIR，各家电池不同。
  在你关心的最低温度下**带载**实测验证过再信任 `lt=`；失败模式是掉电重启，而不是干净关机。
- **安全。** 不要把地板设到低于电芯真实规格或 PMIC UVLO。过放会损伤电芯；地板过低有在负载下
  掉压/重启的风险（电芯被拉到系统最低电压以下）。请在设备上以高负载和低温验证激进的下限。
  限幅地板：cutoff 2800、empty 2500 mV。
- 已针对本树的 FG-Gen4 驱动验证
  （`drivers/power/supply/qcom/qpnp-fg-gen4.c`、`fg-util.c`、`fg-core.h`），
  机型 `lmi` / 内核 4.19.325。

## 许可证

GPL-2.0-or-later（与 Linux 内核及 KernelPatch 一致）。
