[☕Buy me a coffee](https://ko-fi.com/strawing)

# About this KPM

**English** | [简体中文](#简体中文)

A [SukiSU-Ultra](https://github.com/SukiSU-Ultra/SukiSU-Ultra) / [KernelPatch](https://github.com/bmax121/KernelPatch)
**KPM** (Kernel Patch Module) that lowers the Qualcomm **QPNP FG-Gen4**
fuel-gauge discharge **floor** — cutoff `3400→3200`, empty `3100→3000`, and the
*effective shutdown voltage* `3300→3100` — and makes the driver report the
**voltage-derived SOC it already computes** in the sub-3.6 V tail instead of
letting the gauge's graphite-profile `msoc` veto it — entirely at runtime, in
kernel space, with **no kernel source edits**.

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
| `cutoff_volt_mv` | 3400 | **3200** | FVSS 0% scale + hardware `msoc=0%` anchor | DT field write + SRAM word 20 |
| `empty_volt_mv` | 3100 | **3000** | vbatt-low IRQ threshold → arms rapid-SOC | DT field write + SRAM word 35 |
| `SHUTDOWN_DELAY_VOL` | 3300 | **3100** | the **true** shutdown voltage, and the 30 s warning | `hotpatch()` on its MOVZ + hook on `fg_psy_get_property` |
| `VBAT_CRITICAL_LOW_THR` | 2800 | *(left stock)* | rapid-SOC immediate-trip floor | **subsumed** by the shutdown hook |
| FVSS reported SOC | HW-`msoc`-vetoed | **pure voltage** | what the tail below 3600 mV actually shows | hook on `fg_get_msoc` — see [Pure FVSS](#pure-fvss--report-the-voltage-soc-the-driver-already-computes) |

Two DT-field parameters are written directly (they are the **root of truth** —
on any FG profile reload the driver re-derives the SRAM anchors from them — and
the KPM also writes SRAM so the change is immediate).

`SHUTDOWN_DELAY_VOL` is a compile-time `#define`, so instead of patching the
opcode the KPM **hooks `fg_psy_get_property`**: when the driver reports
`CAPACITY == 0` while instantaneous vbatt is still above the target floor, the
hook rewrites the result to `1%`. That extends the stock "hang at 1%" window
from 3300 mV down to 3100 mV **functionally** — robust, no instruction-stream
assumptions.

That same hook also **masks a premature rapid-SOC trip** (which forces
`msoc→0`): the report stays 1% until vbatt genuinely drops below the floor. So
the effective floor is 3100 mV regardless of when rapid latches, which is why
lowering `VBAT_CRITICAL_LOW_THR` is unnecessary and that macro is left at stock.

### Moving the 30-second warning instead of faking it

Rewriting the reported capacity is enough to stop Android shutting down at 0%,
but it cannot move the driver's **countdown**. That is armed separately:

```c
if (pval->intval == 0) {
    shutdown_voltage = is_low_temp_flag ? 3100 : 3300;
    if (vbatt_uv/1000 > shutdown_voltage && !charging) {
        fg->shutdown_delay = true;   /* userspace shows 30 s, then powers off */
        pval->intval = 1;            /* which is why you never see 0% */
    }
}
```

Note the direction: the flag arms while voltage is **above** the threshold, so
*lowering* `SHUTDOWN_DELAY_VOL` widens the window rather than delaying it, and
no value we control can produce a countdown below 3300.

The stock behaviour is right — capacity hits 0, and if there is still voltage
the user gets a warning before shutdown — it is only calibrated for graphite. So
rather than asserting or suppressing `fg->shutdown_delay` from the hook (which
would replace the driver's logic with our own), the KPM **moves the constant**.
Both `SHUTDOWN_DELAY_VOL` and `SHUTDOWN_DELAY_VOL_lOW_TEMP` compile to a plain
`MOVZ w?, #imm16`, so the module scans `fg_psy_get_property`, and rewrites the
immediate with `hotpatch()` — KernelPatch's sanctioned primitive, which runs
under `stop_machine` and flushes I-cache.

It aims the threshold at **our own floor**, so warm and cold stay in step with
`eff_shutdown_mv()`: the driver picks between the two patched constants using
the same `is_low_temp_flag` the module already mirrors.

Guard rails, because a wrong instruction takes the kernel down:

* the scan bounds the function with `kallsyms_lookup_size_offset` and **refuses
  to patch unless each constant occurs exactly once**;
* only a plain 32-bit `MOVZ` is rewritten — shifted variants and `CMP` forms are
  reported but never touched;
* nothing within the first 64 bytes is patched (that is the hook trampoline);
* the original words are kept and **restored on unload**;
* `sdv=0` opts out entirely and leaves the instruction stream untouched;
  `sdv=<mv>` overrides the target.

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

| | warm | cold (driver says so; default `lt=100`) |
|---|---|---|
| cutoff | 3200 | **3100** |
| empty | 3000 | **2900** |
| shutdown floor | 3100 | **3000** |

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
| ours warm 3100 | 300 mV |
| ours cold `lt=100` → 3000 | **200 mV** |
| ours cold `lt=150` → 2950 | 150 mV |
| ours cold `lt=200` → 2900 | 100 mV — avoid |

The **cell** is not the limit here. Silicon-containing anodes tolerate cut-off
voltages down to ~2.5 V precisely because their discharge curve declines
smoothly, where graphite drops sharply below 3 V and needs ~2.8 V. The limits
are the **system** and **cycle life**:

* **System.** Silicon has lower conductivity and higher impedance than graphite,
  so load spikes sag *harder* — exactly when you are closest to the floor. That
  is why the cold step stays at 100 mV instead of matching stock's 200 mV.
* **Cycle life.** Silicon anodes are commonly held above **~3.0 V** to limit the
  volume expansion/contraction stress that drives their fade. The default cold
  floor lands exactly on that line (3100 − 100 = **3000 mV**), which is the main
  reason not to push `lt=` past 100 on a Si/C pack.

### Who decides it is cold — the driver, always

The KPM has **no temperature threshold of its own**. It mirrors the driver's
`is_low_temp_flag` (a file-scope global, resolved by `kallsyms`), which is that
driver's single source of truth for "cold" — its own cutoff pair and its
`SHUTDOWN_DELAY_VOL_lOW_TEMP` both key off the same flag.

This is deliberate, and it decides the `lmi` case correctly. That flag is only
maintained where `qcom,cutoff-voltage-adjust-enable` is set (`umi` sets it,
`lmi` does not), so on a kernel with no low-temp policy the flag stays false
forever and **the KPM never enters the cold tier either**. That is the right
answer: this module *shifts the driver's floors down*, it does not invent a
policy the driver does not have.

It also survives a different kernel. The driver's threshold is a compile-time
`#define` (`LOW_DISCHARGE_TEMP_TRH`), not a device-tree property, so there is
nothing per-device to read and another tree may well have compiled a different
value. Mirroring the flag means we switch on **whatever** that tree decided,
without knowing what it is.

There is deliberately **no hysteresis** on our side either: any threshold or
damping of our own would only create windows where the driver says cold and we
say warm, or the reverse.

Only the **step size** is ours (`lt=`) — that is the part tuned for brown-out
margin above. `lt=0` disables the shift entirely while still tracking the flag.

Where the driver does manage cutoff, the KPM wins the race simply by
re-asserting on every voltage read — far more often than the driver's 10 s
monitor tick — and re-asserts SRAM too, not just the C field.

## Pure FVSS — report the voltage SOC the driver already computes

Lowering the floors buys the tail. This makes the phone actually *count* it.

Below **3600 mV** (3400 when the driver says cold) the driver enters **FVSS**
(Fast Voltage Slope Scaling): it stops trusting the gauge's coulomb counter and
derives SOC linearly from filtered voltage instead —

```
soc = (vbatt_avg - dt.cutoff_volt_mv) / soc_scale_slope
```

— which is exactly what we want on a Si/C cell, because the profile programmed
into the gauge is still the stock graphite one. But `soc_scale_work()` does not
report that value unconditionally. Its **first** branch is a veto:

```c
if ((prev_soc_scale_msoc - msoc_actual) > soc_thr_percent)   /* thr = 1% */
        soc_scale_msoc = prev_soc_scale_msoc - soc_thr_percent;
```

*Whenever the voltage SOC sits more than 1% **above** the hardware `msoc`, throw
the voltage away and step down 1% per tick.* Feeding a graphite profile from a
Si/C cell keeps that true for the **entire** tail, so the display is walked down
to the gauge's pessimistic `msoc`, hits 1% early, and then sits there for a long
time while the cell still holds a large usable reserve. That is the "stuck at 1%
for an hour" symptom, and no amount of floor-lowering fixes it — the floors
decide when 0% happens, this decides what is shown on the way there.

**The fix.** `chip->msoc_actual` has exactly **one writer**
(`fg_gen4_validate_soc_scale_mode`) and exactly **two readers**: the veto above,
and the charging-side FVSS exit test `msoc_actual >= soc_scale_msoc`. Nothing
else in the driver touches it. So the KPM hooks `fg_get_msoc()` and, **only for
the single call that fills that field**, hands back `prev_soc_scale_msoc`:

| | effect |
|---|---|
| veto branch | `prev - msoc_actual == 0`, so it can never fire again |
| what is reported | the driver's own voltage SOC, with its own smoothing: never rises, and a fast voltage drop is still rate-limited to 1% per tick |
| plugging in | the exit test is now immediately true, so FVSS exits at once — **better than stock**: `fg_gen4_exit_soc_scale()` programs the scaled SOC into the hardware `MONOTONIC_SOC` register on the way out, so the displayed value carries over seamlessly and then climbs with the charge, instead of freezing until an inaccurate `msoc` catches up |
| outside FVSS | **nothing changes** — `msoc_actual` has no other consumer, and every other `fg_get_msoc()` caller (including the one behind `POWER_SUPPLY_PROP_CAPACITY` when FVSS is off) passes a stack local the identification rejects |

### Identifying the field without a struct offset

The KPM never hard-codes an offset. It reads the seven ints around the pointer
the driver just passed and requires the whole neighbourhood to check out:

| | field | test |
|---|---|---|
| −3 | `soc_scale_msoc` | 0–100 |
| −2 | `prev_soc_scale_msoc` | 0–100 — **the value written back** |
| −1 | `soc_scale_slope` | **> 0** (only true once FVSS has been entered) |
| 0 | `msoc_actual` | 0–100 — the target |
| +1 | `vbatt_avg` | plausible mV |
| +2 | `vbatt_now` | plausible mV |
| +3 | `vbatt_res` | **exactly** `vbatt_avg - cutoff_volt_mv` |

The last row is the real fingerprint: the caller runs
`fg_gen4_get_prop_soc_scale()` immediately beforehand, which ends by computing
that identity — an exact arithmetic relation between three words we can read,
**anchored on the cutoff value this module itself wrote**. Same spirit as the
SRAM cross-check that locates the dt block, and again with no per-device
constant of our own. (The ordering is what makes the anchor sound:
`fg_gen4_get_prop_soc_scale()` calls `fg_get_battery_voltage()` — where hook 1
re-asserts our cutoff — *before* it computes `vbatt_res`.)

Requiring `slope > 0` means the field is confirmed on the first tick where the
veto could actually fire, never on a zeroed struct. A tick that fails any test
simply does not confirm and the next one is tried, so losing a race against
another CPU costs nothing. Once identified the offset is pinned; a second
in-object address would mean the identification was wrong, and the module
refuses rather than guesses.

### What this does *not* fix

The FVSS **entry** SOC still comes from the gauge:
`fg_gen4_enter_soc_scale()` seeds `soc_scale_msoc` from the currently reported
capacity and derives the slope from it. So the tail is now spread **evenly**
from wherever the gauge thought you were at 3600 mV down to cutoff — no early
collapse, no hour at 1% — but that entry point is still profile-dependent.
Correcting it means replacing the battery profile, which is a different job from
moving the floors. `fvss=0` opts out and restores the stock veto.

## How it works (device-independent)

KPMs run in kernel space and can inline-hook kernel functions. This module
installs three hooks:

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

**Hook 3 — `fg_get_msoc(struct fg_dev *fg, int *msoc)`** (pure FVSS): an AFTER
hook that rewrites the result **only** when the destination pointer is
`&chip->msoc_actual`, identified by the arithmetic identity described in
[Pure FVSS](#pure-fvss--report-the-voltage-soc-the-driver-already-computes).
Every other caller passes a stack local and is left alone.

Symbols (`probe_kernel_read`, `fg_sram_write`, `fg_sram_read`,
`fg_get_battery_voltage`, `fg_psy_get_property`, `fg_get_msoc`) are resolved via
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
# load with the defaults (cutoff 3200, empty 3000, shutdown 3100 mV)
kpm load  /data/local/tmp/fg_cutoff.kpm

# load with explicit targets "cutoff[,empty[,shutdown]]" (mV) as module args
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100"

# ...or name them, in any order — clearer, and immune to a miscounted comma
kpm load  /data/local/tmp/fg_cutoff.kpm "cutoff=3200,shutdown=3100"

# an empty positional field keeps that one at its current value
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,,3100"   # cutoff + shutdown only

# change targets at runtime (cutoff only, or more)
kpm control battery-fg-cutoff "3200"
kpm control battery-fg-cutoff "3200,3000,3100"

# force the CAPACITY ordinal if the by-name lookup ever fails on your tree
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,psp=44"

# low-temperature step (mV, 0-200); WHEN it is cold is the driver's call
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,lt=150"  # aggressive
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,lt=0"    # disable the shift

# the 30 s countdown threshold (patches SHUTDOWN_DELAY_VOL in place)
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,sdv=3050" # explicit
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,sdv=0"    # never patch code

# pure FVSS: default on. "fvss=0" restores the stock HW-msoc veto; it is a live
# gate, so `kpm control` can flip it back and forth without unloading
kpm load    /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,fvss=0"
kpm control battery-fg-cutoff "fvss=1"

# unload (restores stock cutoff 3400 / empty 3100; shutdown hook removed)
kpm unload battery-fg-cutoff
```

### Arguments and what happens when they are wrong

Both `kpm load` and `kpm control` take the same string: fields separated by `,`
or `;` (whitespace is padding). A field is `key=value`, a bare decimal
(positional, in the order **cutoff, empty, shutdown**), or empty. Keys are
case-insensitive; for any one parameter the last mention wins.

| key | range | meaning |
|---|---|---|
| `cutoff` | 2800–3400 mV | FVSS 0% scale + hardware `msoc=0%` anchor |
| `empty` | 2500–3300 mV | vbatt-low IRQ threshold |
| `shutdown` | 2800–3300 mV | effective shutdown floor (end of the hang-at-1% window) |
| `lt` | 0–200 mV | cold-tier step; `0` disables the shift |
| `sdv` | `0`, or 2800–3300 mV | countdown threshold; `0` = never patch the instruction stream |
| `fvss` | `0` or `1` | pure-FVSS SOC (default `1`) |
| `psp` | 0–255 | force the `POWER_SUPPLY_PROP_CAPACITY` ordinal |

**A value that cannot be parsed and range-checked is rejected, and the parameter
keeps its current value** — the compiled default at load time, the live value
from `kpm control`. Nothing is coerced. Every rejection is logged with a reason,
`kpm control` appends `[SOME ARGS REJECTED …]` to its reply, and the init line
prints the values actually in force.

This is a **deliberate reversal** of the old clamping behaviour, and the reason
is worth stating: clamping turned `"320"` — a `3200` with a dropped zero — into
`clamp(320, 2800, 3400)` = **2800 mV**, so a single typo silently selected the
most aggressive floor the module allows, with a brown-out reboot as the failure
mode. Out of range is now a rejection, not a clamp.

A malformed **positional** field additionally discards the whole positional
sequence, because position is the only thing that gives a bare number its
meaning: in `"3200,x,3100"` there is no way to know whether `3100` was meant as
`empty` or as `shutdown`, so neither is applied. Well-formed-but-out-of-range is
per-value instead — there the positions are unambiguous, so only the offending
value is dropped. Use `key=` to sidestep the question entirely.

```
"3200,3000,3100"   all three applied
"cutoff=3200"      cutoff only; empty and shutdown keep their current values
"3200,,3100"       empty field = keep empty_volt; 3100 still lands in shutdown
"320"              rejected (out of range) -> cutoff stays 3200
"3200,x,3100"      malformed positional -> ALL THREE keep their current values
"bogus=3300"       unknown key -> rejected
"3200,lt=999"      cutoff applied; lt rejected (out of range) -> lt stays 100
```

To make it persistent, load it from a SukiSU/KernelPatch boot service (the same
place you load other KPMs).

## Verify

Watch the kernel log after loading:

```bash
dmesg | grep fg-cutoff-kpm
```

Expected, within a second or two of the first FG voltage poll:

```
[fg-cutoff-kpm] init (event=load-file, args=) cutoff=3200 empty=3000 shutdown=3100 mV
[fg-cutoff-kpm] syms: pkr=ffffffaa07c35df4 sram_write=ffffffaa08617550 sram_read=ffffffaa08616b30 get_voltage=ffffffaa086186a4 get_property=ffffffaa08612af0 get_msoc=ffffffaa086180bc
[fg-cutoff-kpm] low-temp tier: -100 mV, mirroring is_low_temp_flag=ffffffaa0ac0afd7 (driver decides when cold)
[fg-cutoff-kpm] psp: CAPACITY=44 (by name, stride 32)
[fg-cutoff-kpm] SDV scan: fg_psy_get_property=ffffffaa08612af0 size=1528
[fg-cutoff-kpm] SDV   +0x0518  insn=52818389  MOVZ imm=3100  <== SHUTDOWN_DELAY_VOL_lOW_TEMP
[fg-cutoff-kpm] SDV   +0x0524  insn=52819c88  MOVZ imm=3300  <== SHUTDOWN_DELAY_VOL
[fg-cutoff-kpm] SDV scan done: 2 in range, 3300 x1, 3100 x1
[fg-cutoff-kpm] SDV verified @ffffffaa08613014: 52819c88 -> 52818388 (imm 3300 -> 3100)
[fg-cutoff-kpm] SDV verified @ffffffaa08613008: 52818389 -> 52817709 (imm 3100 -> 3000)
[fg-cutoff-kpm] SDV active: countdown threshold 3100 mV warm, cold follows lt=
[fg-cutoff-kpm] pure FVSS armed; msoc_actual is identified on the first FVSS tick
[fg-cutoff-kpm] hooks installed; applying on next FG voltage read
[fg-cutoff-kpm] dt located @+0x6cc (SRAM cutoff 3399 mV confirms)
[fg-cutoff-kpm] SRAM CUTOFF_VOLT <- 3200 mV (raw 33 33) rc=0
[fg-cutoff-kpm] SRAM VBATT_LOW  <- 3000 mV (raw 40) rc=0
[fg-cutoff-kpm] applied: cutoff 3400->3200, empty 3100->3000, shutdown floor 3100 mV (dt @ ffffffedb728e74c)
```

If `get_property=0000...` (symbol not found), cutoff/empty still apply but the
shutdown floor stays at the stock 3300 mV.

**Pure FVSS confirms itself later** — only once the pack actually discharges
below 3600 mV and the driver enters FVSS:

```
[fg-cutoff-kpm] FVSS: msoc_actual located @chip+0x... ; HW msoc no longer vetoes the voltage SOC
[fg-cutoff-kpm] FVSS: HW msoc 3 ignored, keeping voltage SOC 11 (vbatt avg 3520 mV)
```

`kpm control battery-fg-cutoff ""` reports its live state — `fvss=active (N)`
once the field is identified (`N` = number of overrides so far), `fvss=armed`
while still waiting for the first FVSS tick, `fvss=off` if disabled or the
symbol was missing. Cross-check against the driver's own FVSS log lines
(`Calculated SOC=... SOC reported=...`, `msoc_actual: ...`): with the KPM active,
`msoc_actual` in those lines tracks the reported scale SOC instead of the
gauge's own falling value.

You can cross-check the live SRAM values if `CONFIG_DEBUG_FS` is enabled on your
build:

```bash
# /sys/kernel/debug/fg/sram
#   word 20 = CUTOFF_VOLT (2 bytes LE)   3200 mV -> 33 33
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
- **Pure FVSS fixes the tail's *shape*, not its *start*.** The FVSS entry SOC is
  still seeded from the gauge, so if the graphite profile already reads low at
  3600 mV, the tail is spread evenly from that low number down to cutoff rather
  than collapsing — the hour parked at 1% goes away, but the entry percentage
  itself is still profile-dependent. Also note the deliberate consequence that
  FVSS now exits the moment you plug in (the display carries over via
  `MONOTONIC_SOC`, so this is seamless, but it *is* a behaviour change).
- **The cold tier is the least validated part.** The right step depends on your
  cell's actual DCIR at temperature, which varies by pack. Validate `lt=` at the
  coldest temperature you care about *under load* before trusting it; the
  failure mode is a brown-out reboot, not a clean shutdown.
- **Safety.** Do not set the floor below the cell's real spec or the PMIC UVLO.
  Over-discharge damages cells; too low a floor risks brown-out/reboot under
  load (the cell sags below system-min). Validate the aggressive end on-device
  at high load and cold temperature. The accepted floors stop at cutoff 2800 /
  empty 2500 mV — anything lower is rejected outright, not clamped to the limit.
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
——cutoff `3400→3200`、empty `3100→3000`、以及*有效关机电压* `3300→3100`——
并且让驱动在 3.6 V 以下的尾段直接上报**它自己已经算出来的电压 SOC**，
而不再让电量计基于石墨曲线的 `msoc` 把它否决掉——
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
| `cutoff_volt_mv` | 3400 | **3200** | FVSS 0% 标度 + 硬件 `msoc=0%` 锚点 | 写 DT 字段 + SRAM word 20 |
| `empty_volt_mv` | 3100 | **3000** | vbatt-low 中断阈值 → 触发 rapid-SOC | 写 DT 字段 + SRAM word 35 |
| `SHUTDOWN_DELAY_VOL` | 3300 | **3100** | **真正的**关机电压，以及那 30 秒警告 | `hotpatch()` 改其 MOVZ + hook `fg_psy_get_property` |
| `VBAT_CRITICAL_LOW_THR` | 2800 | *（保持原厂）* | rapid-SOC 立即触发地板 | 被关机 hook **一并覆盖** |
| FVSS 上报 SOC | 被硬件 `msoc` 否决 | **纯电压** | 3600 mV 以下尾段实际显示的数字 | hook `fg_get_msoc`——见[纯 FVSS](#纯-fvss上报驱动自己算好的电压-soc) |

两个 DT 字段参数直接写入（它们是**真值之源**——每次 FG profile 重载时驱动都会从它们重新推导
SRAM 锚点——同时 KPM 也写 SRAM，使改动立即生效）。

`SHUTDOWN_DELAY_VOL` 是一个编译期 `#define`，因此 KPM 不去打补丁改指令，而是 **hook
`fg_psy_get_property`**：当驱动报 `CAPACITY == 0` 而瞬时 vbatt 仍高于目标地板时，hook 把结果改写为
`1%`。这从**功能上**把原厂“卡在 1%”的窗口从 3300 mV 延伸到 3100 mV——稳健，不对指令流做任何假设。

同一个 hook 还**掩盖了 rapid-SOC 的提前触发**（它会强制 `msoc→0`）：报告会一直保持 1%，直到 vbatt
真正跌破地板。所以无论 rapid 何时锁存，有效地板都是 3100 mV，这也是为什么无需降低
`VBAT_CRITICAL_LOW_THR`，该宏保持原厂值。

### 挪走那 30 秒警告，而不是伪造它

改写上报的电量足以阻止 Android 在 0% 关机，但**动不了驱动的倒计时**。它是另一套触发：

```c
if (pval->intval == 0) {
    shutdown_voltage = is_low_temp_flag ? 3100 : 3300;
    if (vbatt_uv/1000 > shutdown_voltage && 未充电) {
        fg->shutdown_delay = true;   // 用户态弹 30 秒，然后关机
        pval->intval = 1;            // 所以你永远看不到 0%
    }
}
```

注意方向：旗子是在电压**高于**阈值时举起的，所以*调低* `SHUTDOWN_DELAY_VOL` 只会让窗口变宽而非推迟；
而且在 3300 以下，我们能控制的任何数值都无法产生倒计时。

原厂行为本身是对的——电量归零、若还有电压就先警告再关机——只是它按石墨标定。因此模块不去
在 hook 里伪造或压制 `fg->shutdown_delay`（那等于用我们的逻辑替换驱动的），而是**把常量挪走**。
`SHUTDOWN_DELAY_VOL` 和 `SHUTDOWN_DELAY_VOL_lOW_TEMP` 都编译成朴素的 `MOVZ w?, #imm16`，
所以模块扫描 `fg_psy_get_property`，再用 `hotpatch()` 改写立即数——那是 KernelPatch 正式导出的
原语，内部走 `stop_machine` 并刷新 I-cache。

阈值瞄准的是**我们自己的地板**，因此常温/低温与 `eff_shutdown_mv()` 保持同步：驱动在两个被改过的
常量之间做选择时，用的正是模块已经在镜像的那个 `is_low_temp_flag`。

因为改错一条指令会让内核跑飞，所以设了这些护栏：

* 扫描用 `kallsyms_lookup_size_offset` 精确定界函数，**除非每个常量恰好出现一次，否则拒绝打补丁**；
* 只改写朴素的 32 位 `MOVZ`——带移位的变体和 `CMP` 形态只报告、绝不触碰；
* 函数前 64 字节内一律不动（那是 hook 的 trampoline）；
* 保存原始指令字，**卸载时恢复**；
* `sdv=0` 完全退出，指令流一个字节都不碰；`sdv=<mv>` 可指定目标值。

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

| | 常温 | 低温（由驱动判定；默认 `lt=100`） |
|---|---|---|
| cutoff | 3200 | **3100** |
| empty | 3000 | **2900** |
| 关机地板 | 3100 | **3000** |

**为什么不直接照抄原厂的 −200 mV。** 低温下真正的约束不是电芯损伤——这种端电压下 OCV 高得多，
真实放电深度其实不大——而是**系统掉电重启**：正是那个「值得往下探」的内阻，同时让负载尖峰塌得更狠。
关键指标是关机地板相对 `sys_min_volt_mv`（lmi 上是 2800 mV）的余量：

| | 相对 sys_min 的余量 |
|---|---|
| 原厂常温 3300 | 500 mV |
| 原厂低温 3100 | 300 mV |
| 本模块常温 3100 | 300 mV |
| 本模块低温 `lt=100` → 3000 | **200 mV** |
| 本模块低温 `lt=150` → 2950 | 150 mV |
| 本模块低温 `lt=200` → 2900 | 100 mV——不建议 |

**限制来自系统和寿命，不是电芯。** 含硅负极的放电曲线下降平滑，正因如此其截止电压可以低到
约 **2.5 V**；而石墨在 3 V 以下电压陡降，通常需要 2.8 V 左右。真正卡住的是：

* **系统**：硅的导电性比石墨差、内阻更高，负载尖峰塌得**更狠**——而这恰好发生在你最接近地板的时候。
  这就是低温步长保持 100 mV 而不照抄原厂 200 mV 的原因。
* **寿命**：硅负极通常建议保持在 **~3.0 V** 以上，以限制导致其衰减的膨胀/收缩应力。默认低温地板
  正好落在这条线上（3100 − 100 = **3000 mV**），这也是硅碳电池不建议把 `lt=` 调过 100 的主要理由。

### 谁来判定「冷」——永远是驱动

KPM **没有自己的温度阈值**。它镜像驱动的 `is_low_temp_flag`（文件级全局变量，用 `kallsyms` 解析）——
那正是驱动判定「冷」的唯一真值来源，它自己的 cutoff 对和 `SHUTDOWN_DELAY_VOL_lOW_TEMP`
都由同一个标志驱动。

这是刻意的，而且它把 lmi 这种情况判对了。该标志只在 `qcom,cutoff-voltage-adjust-enable`
生效的机型上被维护（umi 有、lmi 没有），所以在没有低温策略的内核上，标志永远是 false，
**KPM 也就永远不会进入低温档**。这才是正确答案：本模块是*把驱动的地板整体下移*，
而不是替驱动发明一套它本来没有的策略。

这样也能扛住换内核。驱动的阈值是编译期 `#define`（`LOW_DISCHARGE_TEMP_TRH`），不是设备树属性，
既没有按机型可读的配置，别的树也完全可能编进了不同的值。镜像这个标志意味着我们跟着那棵树
**自己决定的**结果切换，而不需要知道它是多少。

我们这边同样刻意**不加滞回**：任何自己的阈值或阻尼，只会制造出「驱动说冷而我们说不冷」
（或反过来）的窗口。

只有**步长**是我们自己的（`lt=`）——那是上面按掉电余量调过的部分。`lt=0` 可以在仍然跟踪标志的
前提下完全关闭下移。

在驱动确实会管 cutoff 的机型上，KPM 靠「每次读电压都重新写回」赢下这场竞争——频率远高于驱动
10 秒一次的 monitor tick——而且连 SRAM 一起写回，不只是 C 字段。

## 纯 FVSS：上报驱动自己算好的电压 SOC

降低地板买到了那条尾巴，这一节让手机真正把它**数出来**。

在 **3600 mV** 以下（驱动判定为冷时是 3400），驱动会进入 **FVSS**（Fast Voltage Slope
Scaling）：它不再相信电量计的库仑计，改为按滤波后的电压线性推算 SOC——

```
soc = (vbatt_avg - dt.cutoff_volt_mv) / soc_scale_slope
```

——这正是硅碳电池上我们想要的，因为烧进电量计的仍然是原厂那条石墨曲线。但
`soc_scale_work()` 并不会无条件上报这个值，它的**第一个**分支是一票否决：

```c
if ((prev_soc_scale_msoc - msoc_actual) > soc_thr_percent)   /* thr = 1% */
        soc_scale_msoc = prev_soc_scale_msoc - soc_thr_percent;
```

*只要电压算出的 SOC 比硬件 `msoc` **高**出超过 1%，就把电压结果丢掉，改成每个 tick 硬扣 1%。*
用石墨曲线去跑硅碳电芯，这个条件在**整条**尾巴上都成立，于是显示值被一路拖到电量计那个
悲观的 `msoc`，提前掉到 1%，然后在电芯还剩一大截可用电量的情况下长时间停在那里——这就是
「1% 能用一个小时」的由来。光降地板治不了它：**地板决定 0% 何时发生，这里决定在到达之前显示什么**。

**做法。** `chip->msoc_actual` 全驱动只有**一个写入者**
（`fg_gen4_validate_soc_scale_mode`），也只有**两个读取者**：上面那个否决分支，以及充电侧的
FVSS 退出判断 `msoc_actual >= soc_scale_msoc`。驱动里再没有别的地方碰它。所以 KPM hook
`fg_get_msoc()`，并且**只对填充该字段的那一次调用**返回 `prev_soc_scale_msoc`：

| | 效果 |
|---|---|
| 否决分支 | `prev - msoc_actual == 0`，永远不再触发 |
| 实际上报 | 驱动自己的电压 SOC，且保留它自己的平滑：只降不升，电压快速下跌时仍限速 1%/tick |
| 插上充电 | 退出判断立刻成立，于是马上退出 FVSS——这**比原版更好**：`fg_gen4_exit_soc_scale()` 在退出路径上会把标度 SOC 写进硬件 `MONOTONIC_SOC` 寄存器，因此显示值无缝衔接、随充电上升，而不是冻结着等不准的 `msoc` 慢慢追上来 |
| FVSS 之外 | **毫无变化**——`msoc_actual` 没有别的消费者，而 `fg_get_msoc()` 的其它调用者（包括 FVSS 关闭时 `POWER_SUPPLY_PROP_CAPACITY` 背后的那次）传的都是栈上局部变量，会被识别逻辑拒掉 |

### 不靠结构体偏移来识别这个字段

KPM 里不写死任何偏移。它读取驱动刚传进来的那个指针周围的七个 int，要求整片邻域都对得上：

| | 字段 | 判据 |
|---|---|---|
| −3 | `soc_scale_msoc` | 0–100 |
| −2 | `prev_soc_scale_msoc` | 0–100——**写回去的就是它** |
| −1 | `soc_scale_slope` | **> 0**（只有进入过 FVSS 才成立） |
| 0 | `msoc_actual` | 0–100——目标字段 |
| +1 | `vbatt_avg` | 合理电压 mV |
| +2 | `vbatt_now` | 合理电压 mV |
| +3 | `vbatt_res` | **精确等于** `vbatt_avg - cutoff_volt_mv` |

最后一行才是真正的指纹：调用方在此之前刚跑完 `fg_gen4_get_prop_soc_scale()`，而它的最后
一步正是计算这个恒等式——三个我们能读到的字之间的精确算术关系，而且**锚定在本模块自己写进去
的 cutoff 值上**。这与定位 dt 块时用的 SRAM 交叉校验是同一思路，同样不引入任何机型相关常量。
（顺序是这个锚点成立的关键：`fg_gen4_get_prop_soc_scale()` 是先调用 `fg_get_battery_voltage()`
——hook 1 在那里重新写回我们的 cutoff——**之后**才计算 `vbatt_res` 的。）

要求 `slope > 0`，意味着字段会在「否决分支真正可能触发」的第一个 tick 上被确认，而绝不会
在一个全零、从未使用过的结构体上被误认。任何一个 tick 只要有一项判据不过，就干脆不确认、
等下一个 tick 再试，因此输掉一次与其它 CPU 的竞争不会有任何代价。一旦识别成功，偏移就被钉死；
若之后出现第二个落在对象内的地址，说明识别本身有问题，模块选择拒绝而不是猜测。

### 它*治不了*什么

FVSS 的**进入 SOC** 仍然来自电量计：`fg_gen4_enter_soc_scale()` 用当前上报的电量作为
`soc_scale_msoc` 的起点，斜率也由它推出。所以现在这条尾巴是从「电量计在 3600 mV 时认为的那个
数字」**均匀地**铺到 cutoff——不再提前崩塌，也不再有停在 1% 的那一小时——但那个起点值本身
仍然取决于电池曲线。要修它就得换电池 profile，那是与挪地板不同的另一件事。用 `fvss=0`
可以退出本功能、恢复原厂的否决逻辑。

## 工作原理（与机型无关）

KPM 运行在内核态，可以对内核函数做 inline hook。本模块安装三个 hook：

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

**Hook 3 —— `fg_get_msoc(struct fg_dev *fg, int *msoc)`**（纯 FVSS）：一个 AFTER hook，
**仅当**目标指针是 `&chip->msoc_actual` 时改写返回结果，该字段由
[纯 FVSS](#纯-fvss上报驱动自己算好的电压-soc) 一节所述的算术恒等式识别。
其它调用者传的都是栈上局部变量，一概不动。

符号（`probe_kernel_read`、`fg_sram_write`、`fg_sram_read`、`fg_get_battery_voltage`、
`fg_psy_get_property`、`fg_get_msoc`）通过 `kallsyms_lookup_name` 解析。

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
# 用默认值加载（cutoff 3200、empty 3000、shutdown 3100 mV）
kpm load  /data/local/tmp/fg_cutoff.kpm

# 以模块参数 "cutoff[,empty[,shutdown]]"（mV）显式指定目标加载
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100"

# ……也可以按名字给，顺序随意——更清晰，也不怕逗号数错
kpm load  /data/local/tmp/fg_cutoff.kpm "cutoff=3200,shutdown=3100"

# 位置参数留空表示「这一项保持当前值」
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,,3100"   # 只改 cutoff 和 shutdown

# 运行时修改目标（仅 cutoff，或更多）
kpm control battery-fg-cutoff "3200"
kpm control battery-fg-cutoff "3200,3000,3100"

# 万一按名字查找在你的树上失败，可强制指定 CAPACITY 序号
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,psp=44"

# 低温步长（mV，0-200）；何时算低温由驱动决定
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,lt=150"  # 激进
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,lt=0"    # 关闭下移

# 30 秒倒计时的触发阈值（就地改写 SHUTDOWN_DELAY_VOL）
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,sdv=3050" # 指定
kpm load  /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,sdv=0"    # 绝不改指令

# 纯 FVSS：默认开启。"fvss=0" 恢复原厂的硬件 msoc 否决逻辑；它是运行时开关，
# 因此可以用 `kpm control` 随时来回切换，无需卸载
kpm load    /data/local/tmp/fg_cutoff.kpm "3200,3000,3100,fvss=0"
kpm control battery-fg-cutoff "fvss=1"

# 卸载（恢复原厂 cutoff 3400 / empty 3100；移除关机 hook）
kpm unload battery-fg-cutoff
```

### 参数格式，以及写错时会发生什么

`kpm load` 和 `kpm control` 接受同一种字符串：字段之间用 `,` 或 `;` 分隔（空白只是填充）。
每个字段可以是 `key=value`、一个纯十进制数（位置参数，顺序为 **cutoff、empty、shutdown**），
或者留空。key 不区分大小写；同一个参数被提到多次时，以最后一次为准。

| key | 取值范围 | 含义 |
|---|---|---|
| `cutoff` | 2800–3400 mV | FVSS 0% 标度 + 硬件 `msoc=0%` 锚点 |
| `empty` | 2500–3300 mV | vbatt-low 中断阈值 |
| `shutdown` | 2800–3300 mV | 有效关机地板（挂在 1% 那段窗口的终点） |
| `lt` | 0–200 mV | 低温档步长；`0` 关闭下移 |
| `sdv` | `0`，或 2800–3300 mV | 倒计时阈值；`0` 表示绝不改写指令流 |
| `fvss` | `0` 或 `1` | 纯 FVSS SOC（默认 `1`） |
| `psp` | 0–255 | 强制指定 `POWER_SUPPLY_PROP_CAPACITY` 序号 |

**凡是无法解析、或超出范围的值一律被拒绝，对应参数保持当前值**——加载时是编译进去的默认值，
`kpm control` 时是当前生效的值。不做任何静默折中。每一次拒绝都会带原因写进内核日志，
`kpm control` 的回显会追加 `[SOME ARGS REJECTED …]`，init 那行则打印真正生效的值。

这是对旧行为（超范围就**截断**）的一次**刻意反转**，理由值得写清楚：截断会把 `"320"`
——也就是少打一个零的 `3200`——变成 `clamp(320, 2800, 3400)` = **2800 mV**，
于是一个笔误就静默选中了模块允许的最激进地板，而它的失败模式是掉电重启。
所以现在超出范围是**拒绝**，不是截断。

另外，**位置参数**只要有一个格式错误，整组位置参数都会作废——因为位置是纯数字唯一的含义来源：
`"3200,x,3100"` 里没法判断 `3100` 想给的是 `empty` 还是 `shutdown`，于是两者都不应用。
而「格式正确但超范围」是逐个处理的——那时位置是明确的，所以只丢掉出问题的那一个。
用 `key=` 形式可以彻底绕开这个问题。

```
"3200,3000,3100"   三个都生效
"cutoff=3200"      只改 cutoff；empty 和 shutdown 保持当前值
"3200,,3100"       空字段 = 保持 empty_volt；3100 仍然落在 shutdown 上
"320"              被拒（超范围）-> cutoff 保持 3200
"3200,x,3100"      位置参数格式错误 -> 三个全部保持当前值
"bogus=3300"       未知 key -> 被拒
"3200,lt=999"      cutoff 生效；lt 被拒（超范围）-> lt 保持 100
```

若要持久化，请在 SukiSU/KernelPatch 的开机服务里加载它（和你加载其它 KPM 的地方一样）。

## 验证

加载后观察内核日志：

```bash
dmesg | grep fg-cutoff-kpm
```

模块加载后，在首次 FG 电压轮询后一两秒内，可以看看dmesg，如果看到：

```
[fg-cutoff-kpm] init (event=load-file, args=) cutoff=3200 empty=3000 shutdown=3100 mV
[fg-cutoff-kpm] syms: pkr=ffffffaa07c35df4 sram_write=ffffffaa08617550 sram_read=ffffffaa08616b30 get_voltage=ffffffaa086186a4 get_property=ffffffaa08612af0 get_msoc=ffffffaa086180bc
[fg-cutoff-kpm] low-temp tier: -100 mV, mirroring is_low_temp_flag=ffffffaa0ac0afd7 (driver decides when cold)
[fg-cutoff-kpm] psp: CAPACITY=44 (by name, stride 32)
[fg-cutoff-kpm] SDV scan: fg_psy_get_property=ffffffaa08612af0 size=1528
[fg-cutoff-kpm] SDV   +0x0518  insn=52818389  MOVZ imm=3100  <== SHUTDOWN_DELAY_VOL_lOW_TEMP
[fg-cutoff-kpm] SDV   +0x0524  insn=52819c88  MOVZ imm=3300  <== SHUTDOWN_DELAY_VOL
[fg-cutoff-kpm] SDV scan done: 2 in range, 3300 x1, 3100 x1
[fg-cutoff-kpm] SDV verified @ffffffaa08613014: 52819c88 -> 52818388 (imm 3300 -> 3100)
[fg-cutoff-kpm] SDV verified @ffffffaa08613008: 52818389 -> 52817709 (imm 3100 -> 3000)
[fg-cutoff-kpm] SDV active: countdown threshold 3100 mV warm, cold follows lt=
[fg-cutoff-kpm] pure FVSS armed; msoc_actual is identified on the first FVSS tick
[fg-cutoff-kpm] hooks installed; applying on next FG voltage read
[fg-cutoff-kpm] dt located @+0x6cc (SRAM cutoff 3399 mV confirms)
[fg-cutoff-kpm] SRAM CUTOFF_VOLT <- 3200 mV (raw 33 33) rc=0
[fg-cutoff-kpm] SRAM VBATT_LOW  <- 3000 mV (raw 40) rc=0
[fg-cutoff-kpm] applied: cutoff 3400->3200, empty 3100->3000, shutdown floor 3100 mV (dt @ ffffffedb728e74c)
```

若 `get_property=0000...`（符号未找到），cutoff/empty 仍会应用，但关机地板保持原厂 3300 mV。

**纯 FVSS 的确认会晚一些出现**——要等电池真的放到 3600 mV 以下、驱动进入 FVSS 之后：

```
[fg-cutoff-kpm] FVSS: msoc_actual located @chip+0x... ; HW msoc no longer vetoes the voltage SOC
[fg-cutoff-kpm] FVSS: HW msoc 3 ignored, keeping voltage SOC 11 (vbatt avg 3520 mV)
```

`kpm control battery-fg-cutoff ""` 会回报实时状态：字段识别成功后是 `fvss=active (N)`
（`N` 为已改写次数），还在等第一个 FVSS tick 时是 `fvss=armed`，被关闭或符号缺失时是 `fvss=off`。
也可以和驱动自己的 FVSS 日志（`Calculated SOC=... SOC reported=...`、`msoc_actual: ...`）
对照着看：KPM 生效时，那些行里的 `msoc_actual` 会跟随上报的标度 SOC，而不再是电量计自己
一路下滑的值。

如果你的构建启用了 `CONFIG_DEBUG_FS`，可以交叉核对实时 SRAM 值：

```bash
# /sys/kernel/debug/fg/sram
#   word 20 = CUTOFF_VOLT（2 字节 LE）    3200 mV -> 33 33
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
- **纯 FVSS 修的是尾巴的*形状*，不是它的*起点*。** FVSS 的进入 SOC 仍然由电量计给出，
  所以如果石墨曲线在 3600 mV 时读数已经偏低，这条尾巴只是从那个偏低的数字均匀铺到 cutoff，
  而不会崩塌——停在 1% 的那一小时会消失，但进入时的百分比本身仍取决于电池曲线。另外请注意
  一个刻意为之的副作用：现在一插上充电就会立刻退出 FVSS（显示值通过 `MONOTONIC_SOC` 无缝
  衔接，所以观感上没有跳变，但它*确实*是一处行为变化）。
- **低温档是验证最少的部分。** 合适的步长取决于你这颗电芯在对应温度下的真实 DCIR，各家电池不同。
  在你关心的最低温度下**带载**实测验证过再信任 `lt=`；失败模式是掉电重启，而不是干净关机。
- **安全。** 不要把地板设到低于电芯真实规格或 PMIC UVLO。过放会损伤电芯；地板过低有在负载下
  掉压/重启的风险（电芯被拉到系统最低电压以下）。请在设备上以高负载和低温验证激进的下限。
  允许的地板下限是 cutoff 2800 / empty 2500 mV——再低会被直接拒绝，而不是截断到该下限。
- 已针对本树的 FG-Gen4 驱动验证
  （`drivers/power/supply/qcom/qpnp-fg-gen4.c`、`fg-util.c`、`fg-core.h`），
  机型 `lmi` / 内核 4.19.325。

## 许可证

GPL-2.0-or-later（与 Linux 内核及 KernelPatch 一致）。
