// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * fg_cutoff.kpm — SukiSU / KernelPatch KPM module
 *
 * Lowers the QPNP FG-Gen4 discharge floor on the Redmi K30 Pro (lmi) so a
 * silicon/carbon (Si/SiOx) replacement cell is not reported 0% while it still
 * has a usable sub-3.4V tail. EVERYTHING is done from this module — no kernel
 * source edits. Conservative tier defaults:
 *
 *     cutoff_volt_mv        : 3400 -> 3200   (dt field + SRAM CUTOFF_VOLT)
 *     empty_volt_mv         : 3100 -> 3000   (dt field + SRAM VBATT_LOW)
 *     SHUTDOWN_DELAY_VOL eff : 3300 -> 3100   (functional hook, see below)
 *     FVSS reported SOC     : HW-msoc-vetoed -> pure voltage (functional hook)
 *
 * Why a hook for SHUTDOWN_DELAY_VOL instead of patching the #define:
 *   The effective shutdown voltage is SHUTDOWN_DELAY_VOL, not cutoff — when the
 *   computed SOC hits 0 the driver keeps reporting 1% until the voltage falls
 *   below SHUTDOWN_DELAY_VOL (fg_psy_get_property, POWER_SUPPLY_PROP_CAPACITY).
 *   That constant is baked into the instruction stream; patching it blind is
 *   fragile. Instead we hook fg_psy_get_property and, when it reports CAPACITY
 *   == 0 while instantaneous vbatt is still above our floor, rewrite the result
 *   to 1% — i.e. we EXTEND the "hang at 1%" window down to the new floor. This
 *   is robust (no opcode assumptions) and also masks a premature rapid-SOC trip
 *   (which forces msoc->0): the report stays 1% until vbatt truly drops below
 *   the floor. That makes lowering VBAT_CRITICAL_LOW_THR (2800) unnecessary for
 *   the user-visible floor, so that macro is intentionally left at stock.
 *
 * Field/SRAM location carries NO per-device constants:
 *   - Hook fg_get_battery_voltage(struct fg_dev *fg, int *val), an EXPORTED,
 *     frequently-called helper. `struct fg_dev fg` is the FIRST member of
 *     `struct fg_gen4_chip`, so arg0 == the chip base pointer.
 *   - Scan the chip object for the dt block by the SHAPE of the six ints that
 *     open struct fg_dt_props (three plausible mV, one positive mA, two
 *     negative mA), then confirm the candidate against the CUTOFF_VOLT the
 *     device itself has programmed into FG SRAM (read back via fg_sram_read).
 *     Accepted only when unique; ambiguity is logged and nothing is written.
 *   - POWER_SUPPLY_PROP_CAPACITY is resolved at runtime by name from the
 *     kernel's power_supply_attrs[] table, not hard-coded as an ordinal.
 *   - Reads use probe_kernel_read, so a bad address returns -EFAULT, never a
 *     panic. SRAM is written via the exported fg_sram_write.
 *   - Low-temperature tier: all three floors drop by lt_delta_mv while the
 *     driver reports cold. WHEN that is, is mirrored from the driver's own
 *     is_low_temp_flag -- no threshold of our own, so a kernel with no low-temp
 *     policy (lmi) never enters the tier either. Where the driver does manage
 *     cutoff we win the race by re-asserting on every voltage read.
 *   - SHUTDOWN_DELAY_VOL (3300) and its low-temp twin (3100) are compile-time
 *     constants, so the 30 s warning + shutdown is calibrated for graphite. We
 *     keep that behaviour and just move it: both are plain MOVZ imm16, located
 *     by a uniqueness-checked scan and rewritten with hotpatch() (stop_machine
 *     + I-cache flush) to aim at our own floor. Refused unless each constant
 *     occurs exactly once. "sdv=0" opts out.
 *   - Pure FVSS: below 3600 mV (3400 cold) the driver derives SOC from filtered
 *     voltage, but soc_scale_work() vetoes that value whenever the hardware
 *     msoc sits more than 1% below it and walks the display down 1% per tick
 *     instead. With the stock graphite profile driving a Si/C cell that veto is
 *     active for the whole tail, which is what parks the phone at 1% for an
 *     hour. We hook fg_get_msoc() and, ONLY for the single call that fills
 *     chip->msoc_actual, hand back prev_soc_scale_msoc -- so the veto's
 *     difference is exactly zero and the driver's own voltage SOC is reported.
 *     The field is identified by an exact arithmetic identity the driver just
 *     computed (vbatt_res == vbatt_avg - cutoff_volt_mv), anchored on the cutoff
 *     value we ourselves wrote, so again no per-device constant. "fvss=0" opts
 *     out. Nothing outside FVSS changes: msoc_actual has no other consumer, and
 *     every other fg_get_msoc() caller passes a stack local we never touch.
 *   - Stock values are restored on unload, including the patched instructions.
 */

#include <compiler.h>
#include <kpmodule.h>
#include <hook.h>
#include <kallsyms.h>
#include <kputils.h>
#include <linux/printk.h>
#include <linux/string.h>

/*
 * Local formatter. KernelPatch's kpimg resolves a KPM's undefined symbols ONLY
 * against its own KP_EXPORT_SYMBOL table -- the kallsyms fallback in
 * simplify_symbols() is commented out both upstream and in SukiSU's fork -- and
 * snprintf is not in that table. Referencing it makes the loader reject the
 * module with "unknown symbol: snprintf" (-ENOENT) before init ever runs, so
 * format in-module instead. %d, %s and %% only; always NUL-terminates.
 */
static int fmt_msg(char *buf, int size, const char *fmt, ...)
{
    __builtin_va_list ap;
    int n = 0;

    if (!buf || size <= 0)
        return 0;

    __builtin_va_start(ap, fmt);
    for (; *fmt && n < size - 1; fmt++) {
        if (*fmt != '%') {
            buf[n++] = *fmt;
            continue;
        }
        fmt++;
        if (!*fmt)
            break;
        if (*fmt == 'd') {
            int v = __builtin_va_arg(ap, int);
            unsigned int u = v < 0 ? -(unsigned int)v : (unsigned int)v;
            char tmp[12];
            int i = 0;

            if (v < 0)
                buf[n++] = '-';
            do {
                tmp[i++] = (char)('0' + u % 10u);
                u /= 10u;
            } while (u);
            while (i > 0 && n < size - 1)
                buf[n++] = tmp[--i];
        } else if (*fmt == 's') {
            const char *s = __builtin_va_arg(ap, const char *);

            while (*s && n < size - 1)
                buf[n++] = *s++;
        } else {
            buf[n++] = *fmt;
        }
    }
    __builtin_va_end(ap);
    buf[n] = '\0';
    return n;
}

KPM_NAME("battery-fg-cutoff");
KPM_VERSION("1.7.0");
KPM_LICENSE("GPL v2");
KPM_AUTHOR("battery-fg-fix-kpm");
KPM_DESCRIPTION("FG-Gen4: lower cutoff/empty/shutdown floor + pure-FVSS SOC (Si-C tier)");

/* ------------------------------------------------------------------ */
/* Tunables (conservative tier)                                        */
/* ------------------------------------------------------------------ */

#define DEFAULT_CUTOFF_MV   3200
#define CUTOFF_MV_MIN       2800
#define CUTOFF_MV_MAX       3400

#define DEFAULT_EMPTY_MV    3000
#define EMPTY_MV_MIN        2500
#define EMPTY_MV_MAX        3300

#define DEFAULT_SHUTDOWN_MV 3100   /* effective shutdown floor (hang-at-1% end) */
#define SHUTDOWN_MV_MIN     2800
#define SHUTDOWN_MV_MAX     3300

/*
 * Low-temperature tier.
 *
 * Cold raises cell impedance, so the LOADED terminal voltage sags far below
 * OCV and holding the warm floor throws away real charge. Stock handles this by
 * shifting its whole set down 200 mV when cold:
 *   cutoff 3400->3200, SHUTDOWN_DELAY_VOL 3300->3100, FVSS entry 3600->3400.
 *
 * WHEN it is cold is decided ENTIRELY by the driver: we mirror its
 * is_low_temp_flag. That flag is only maintained where
 * qcom,cutoff-voltage-adjust-enable is set (umi does, lmi does not), so on a
 * device whose kernel has no low-temp policy we correctly never enter the tier
 * either -- the module shifts the driver's floors down, it does not invent a
 * policy the driver does not have. Deliberately no own threshold: the driver's
 * is a compile-time #define that another tree may well have changed, and the
 * target kernel need not be the one this was developed against.
 *
 * HOW FAR to drop is ours. The binding constraint in the cold is not cell
 * damage -- at these terminal voltages OCV is much higher, so true
 * depth-of-discharge stays modest -- it is system brown-out, because the same
 * impedance that justifies going lower also makes load spikes sag harder.
 * Headroom over sys_min_volt_mv (2800 on lmi) at the shutdown floor:
 *     stock warm 500 mV | stock cold 300 mV | ours warm 300 mV
 *     ours cold at -100 => 200 mV   (default)
 *     ours cold at -150 => 150 mV
 *     ours cold at -200 => 100 mV   (avoid)
 * Si/C tolerates the cell voltage (silicon anodes run to 2.5-3.0 V), so the
 * limit here is the SYSTEM, not the cell -- and silicon's higher impedance
 * makes load spikes sag harder than graphite, which is why the cold step is
 * left at 100 rather than matching stock's 200.
 *
 * Applied uniformly to all three floors so the cutoff->shutdown "hang at 1%"
 * window keeps its width. Tunable at load time with "lt=<mv>".
 */
#define DEFAULT_LT_DELTA_MV 100    /* how far the whole set drops when cold   */
#define LT_DELTA_MV_MAX     200

/*
 * POWER_SUPPLY_PROP_CAPACITY ordinal. Derived at runtime by name from the
 * kernel's power_supply_attrs[] table (which is indexed by the property enum),
 * so it is NOT tied to one tree's include/linux/power_supply.h. This value is
 * only the fallback if that derivation fails; it can also be forced with the
 * "psp=<n>" module argument.
 */
#define PSP_CAPACITY_FALLBACK 44
#define PSP_MAX               255  /* "psp=<n>" sanity bound on the ordinal */

/*
 * dt-block detection — deliberately NOT keyed on any device's specific DT
 * values. `struct fg_dt_props` opens with six ints in a fixed order:
 *
 *   idx0 cutoff_volt_mv       <- we write     idx3 cutoff_curr_ma    (> 0)
 *   idx1 empty_volt_mv        <- we write     idx4 sys_term_curr_ma  (< 0)
 *   idx2 sys_min_volt_mv      (untouched)     idx5 ffc_sys_term_curr_ma (< 0,
 *                                                   or -EINVAL when absent)
 *
 * We match on that *shape* (three plausible mV, one positive mA, two negative
 * mA) and then confirm the candidate against the value the driver actually
 * programmed into SRAM CUTOFF_VOLT. That anchor comes from the device itself,
 * so the scan needs no per-device constants. A candidate is only accepted if
 * it is UNIQUE in the whole scan window -- ambiguity is reported and nothing
 * is written.
 */
#define FP_MV_LO            2000   /* plausible cell voltage, mV */
#define FP_MV_HI            4500
#define FP_CURR_HI          5000   /* plausible termination current, mA */
#define FP_SRAM_TOL_MV      4      /* CUTOFF_VOLT round-trip quantisation */

#define SCAN_MAX_BYTES      0x4000 /* 16 KiB, probe_kernel_read-guarded */

/*
 * Upper bound on how far into the chip object the FVSS scale block can sit.
 * struct fg_gen4_chip opens with fg + dt (found by the scan above) and the
 * soc_scale_* / vbatt_* ints come well after a PROFILE_LEN byte array, so this
 * is deliberately loose -- the identity test below, not this bound, is what
 * identifies the field. Its only job is to reject stack pointers cheaply.
 */
#define MSOC_CHIP_SPAN      0x10000 /* 64 KiB, probe_kernel_read-guarded */

/* FG SRAM parameters (identical for pm8150b v1 and v2 tables) */
#define CUTOFF_VOLT_WORD    20
#define CUTOFF_VOLT_BYTE    0
#define CUTOFF_VOLT_LEN     2
#define CUTOFF_VOLT_NUMRTR  1000000
#define CUTOFF_VOLT_DENMTR  244141
#define CUTOFF_VOLT_VOFF    0

#define VBATT_LOW_WORD      35
#define VBATT_LOW_BYTE      1
#define VBATT_LOW_LEN       1
#define VBATT_LOW_NUMRTR    1000
#define VBATT_LOW_DENMTR    15625
#define VBATT_LOW_VOFF      (-2000)

#define FG_IMA_DEFAULT      0

#define LOG_PREFIX "[fg-cutoff-kpm] "

/* ------------------------------------------------------------------ */
/* Resolved kernel symbols                                            */
/* ------------------------------------------------------------------ */

typedef long (*probe_kernel_read_t)(void *dst, const void *src, unsigned long size);
typedef int  (*fg_sram_write_t)(void *fg, unsigned short address, unsigned char offset,
                                unsigned char *val, int len, int flags);
typedef int  (*fg_sram_read_t)(void *fg, unsigned short address, unsigned char offset,
                               unsigned char *val, int len, int flags);
typedef int  (*fg_get_batt_volt_t)(void *fg, int *val_uv);
typedef int  (*ksym_size_off_t)(unsigned long addr, unsigned long *size, unsigned long *off);
typedef int  (*insn_patch_text_t)(void *addrs[], unsigned int insns[], int cnt);

static probe_kernel_read_t p_probe_kernel_read;
static fg_sram_write_t     p_fg_sram_write;
static fg_sram_read_t      p_fg_sram_read;          /* optional: anchor source */
static void               *p_fg_get_battery_voltage; /* hook target + callable */
static void               *p_fg_psy_get_property;    /* hook target            */
static void               *p_fg_get_msoc;             /* hook target            */
static const void         *p_is_low_temp_flag;      /* driver's own cold verdict */

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

static int   target_cutoff_mv   = DEFAULT_CUTOFF_MV;
static int   target_empty_mv    = DEFAULT_EMPTY_MV;
static int   target_shutdown_mv = DEFAULT_SHUTDOWN_MV;
static int   lt_delta_mv    = DEFAULT_LT_DELTA_MV;  /* cold-tier step, mV      */
static int   in_low_temp;                           /* mirrors driver's flag   */
static int   orig_cutoff_mv     = -1;  /* stock values, captured on first apply */
static int   orig_empty_mv      = -1;
static int  *cutoff_field;             /* kernel VA of dt.cutoff_volt_mv         */
static int  *empty_field;              /* kernel VA of dt.empty_volt_mv (= +1)   */
static void *chip_ptr;                 /* cached chip == fg base; "located" gate */
static int   psp_capacity  = -1;       /* resolved at init (name lookup / arg)   */
static int   psp_from_arg  = -1;       /* "psp=<n>" override, -1 = unset         */

/* SHUTDOWN_DELAY_VOL instruction patch: sites, originals, and the arg. */
static void        *sdv_addr[2];        /* [0] = normal site, [1] = low-temp   */
static unsigned int sdv_orig[2];        /* original words, for restore         */
static int          sdv_patched;
static int          sdv_from_arg = -1;  /* -1 unset, 0 disable, >0 explicit mV */
static insn_patch_text_t p_insn_patch_text;

/* Pure-FVSS state: see after_get_msoc(). */
static int   fvss_pure = 1;             /* "fvss=0" opts out                    */
static long  msoc_actual_off = -1;      /* confirmed chip offset of msoc_actual */
static int   fvss_overrides;            /* how many times we have fed it        */
static int   fvss_logs_left = 3;        /* first-few-overrides notices          */
static int   fvss_id_logs_left = 3;     /* identity-test failure notices        */

static int   reassert_logs_left = 3;    /* driver-overwrote-us notices          */
static int   diag_dumps_left = 3;       /* one-shot memory dumps on scan failure */

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static inline int clamp_i(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * Effective floors = configured targets, shifted down by lt_delta_mv while the
 * cold tier is active. Clamped, so a large "lt=" can never punch through the
 * absolute minimums.
 */
static int eff_cutoff_mv(void)
{
    return clamp_i(target_cutoff_mv - (in_low_temp ? lt_delta_mv : 0),
                   CUTOFF_MV_MIN, CUTOFF_MV_MAX);
}

static int eff_empty_mv(void)
{
    return clamp_i(target_empty_mv - (in_low_temp ? lt_delta_mv : 0),
                   EMPTY_MV_MIN, EMPTY_MV_MAX);
}

static int eff_shutdown_mv(void)
{
    return clamp_i(target_shutdown_mv - (in_low_temp ? lt_delta_mv : 0),
                   SHUTDOWN_MV_MIN, SHUTDOWN_MV_MAX);
}

static int safe_read_ints(const void *src, int *dst, int n)
{
    if (!p_probe_kernel_read)
        return -1;
    return (int)p_probe_kernel_read(dst, src, (unsigned long)n * sizeof(int));
}

/* Inverse of encode_volt(): raw SRAM value -> mV. */
static int decode_volt(int raw, int numrtr, int denmtr, int voff)
{
    return (int)(((long long)raw * denmtr) / numrtr) - voff;
}

/*
 * Read back the CUTOFF_VOLT the driver actually programmed into SRAM. This is
 * the device-supplied anchor that replaces the old hard-coded DT fingerprint.
 * Returns mV, or -1 if unavailable.
 */
static int read_cutoff_sram(void *fg)
{
    unsigned char buf[CUTOFF_VOLT_LEN] = { 0 };
    int raw = 0, i, rc;

    if (!p_fg_sram_read)
        return -1;

    rc = p_fg_sram_read(fg, CUTOFF_VOLT_WORD, CUTOFF_VOLT_BYTE, buf,
                        CUTOFF_VOLT_LEN, FG_IMA_DEFAULT);
    if (rc)
        return -1;

    for (i = CUTOFF_VOLT_LEN - 1; i >= 0; i--)
        raw = (raw << 8) | buf[i];

    return decode_volt(raw, CUTOFF_VOLT_NUMRTR, CUTOFF_VOLT_DENMTR,
                       CUTOFF_VOLT_VOFF);
}

/* Does this 6-int window have the shape of the head of struct fg_dt_props? */
static int dt_shape_ok(const int *w)
{
    return w[0] >= FP_MV_LO   && w[0] <= FP_MV_HI &&      /* cutoff_volt_mv  */
           w[1] >= FP_MV_LO   && w[1] <= FP_MV_HI &&      /* empty_volt_mv   */
           w[2] >= FP_MV_LO   && w[2] <= FP_MV_HI &&      /* sys_min_volt_mv */
           w[3] > 0           && w[3] <= FP_CURR_HI &&    /* cutoff_curr_ma  */
           w[4] < 0           && w[4] >= -FP_CURR_HI &&   /* sys_term_curr   */
           w[5] < 0           && w[5] >= -FP_CURR_HI &&   /* ffc_sys_term    */
           w[1] <= w[0];      /* empty is never above cutoff (3100 <= 3400)  */
}

/*
 * Resolve POWER_SUPPLY_PROP_CAPACITY by NAME instead of hard-coding the enum
 * ordinal. power_supply_attrs[] is indexed by the property enum and each entry
 * begins with a `const char *name`. struct device_attribute is 32 bytes when
 * CONFIG_DEBUG_LOCK_ALLOC is off and larger when it is on, so probe a few
 * strides and accept the one where index 0 really is "status".
 */
static int name_at(unsigned long base, unsigned long stride, int idx, char *out, int outlen)
{
    const char *np = 0;
    int i;

    if (safe_read_ints((const void *)(base + (unsigned long)idx * stride),
                       (int *)&np, (int)(sizeof(np) / sizeof(int))) != 0 || !np)
        return -1;

    /* Byte at a time: a fixed-size read could run off the end of a mapping. */
    for (i = 0; i < outlen - 1; i++) {
        if (p_probe_kernel_read(&out[i], np + i, 1) != 0)
            return -1;
        if (!out[i])
            return 0;
    }
    out[outlen - 1] = '\0';
    return 0;
}

static int str_eq(const char *a, const char *b)
{
    while (*a && *a == *b) { a++; b++; }
    return *a == '\0' && *b == '\0';
}

static int resolve_psp_capacity(void)
{
    unsigned long base = kallsyms_lookup_name("power_supply_attrs");
    unsigned long strides[] = { 32, 40, 48, 56, 64 };
    char name[24];
    unsigned int si;
    int i;

    if (!base)
        return -1;

    for (si = 0; si < sizeof(strides) / sizeof(strides[0]); si++) {
        unsigned long stride = strides[si];

        if (name_at(base, stride, 0, name, sizeof(name)) != 0 || !str_eq(name, "status"))
            continue;
        if (name_at(base, stride, 1, name, sizeof(name)) != 0 || !str_eq(name, "charge_type"))
            continue;

        for (i = 0; i < 200; i++) {
            if (name_at(base, stride, i, name, sizeof(name)) != 0)
                break;
            if (str_eq(name, "capacity")) {
                pr_info(LOG_PREFIX "psp: CAPACITY=%d (by name, stride %d)\n",
                        i, (int)stride);
                return i;
            }
        }
    }
    return -1;
}

/*
 * READ-ONLY probe: how is SHUTDOWN_DELAY_VOL materialised in this build?
 *
 * The driver decides the shutdown/countdown threshold from two compile-time
 * constants that live in the instruction stream, not in memory:
 *     SHUTDOWN_DELAY_VOL          3300
 *     SHUTDOWN_DELAY_VOL_lOW_TEMP 3100
 * Both are < 4096, so the compiler can encode either as a MOVZ imm16 or fold
 * it straight into a CMP imm12. This walks fg_psy_get_property() and reports
 * every voltage-looking immediate it finds, so we can see whether the constant
 * appears EXACTLY ONCE (patchable with hotpatch()) or is ambiguous (leave it
 * alone). Nothing is written -- this only reads and prints.
 */
#define INSN_IS_MOVZ32(i)  (((i) & 0xFFE00000u) == 0x52800000u)  /* movz w?, #imm16 */
#define INSN_IS_MOVZ64(i)  (((i) & 0xFFE00000u) == 0xD2800000u)  /* movz x?, #imm16 */
#define INSN_MOVZ_IMM(i)   (((i) >> 5) & 0xFFFFu)
#define INSN_MOVZ_HW(i)    (((i) >> 21) & 3u)
#define INSN_IS_CMP32(i)   (((i) & 0xFF000000u) == 0x71000000u)  /* subs wzr, w?, #imm12 */
#define INSN_CMP_IMM(i)    (((i) >> 10) & 0xFFFu)
#define INSN_CMP_SH(i)     (((i) >> 22) & 1u)

#define SDV_SCAN_FALLBACK  0x2000   /* if the symbol size is unavailable */
#define SDV_MV_LO          2500     /* only report plausible cell voltages */
#define SDV_MV_HI          3700
#define SDV_MAX_PRINT      16
#define SDV_ENTRY_GUARD    64       /* never touch the hook trampoline area */

/* Replace the imm16 field of a MOVZ, leaving sf/hw/Rd untouched. */
static unsigned int movz_set_imm(unsigned int insn, unsigned int imm)
{
    return (insn & ~(0xFFFFu << 5)) | ((imm & 0xFFFFu) << 5);
}

/*
 * Locate the two constants. Returns 1 only when BOTH the 3300 and the 3100
 * site are unique -- anything else and we refuse to touch the instruction
 * stream, because patching the wrong word takes the kernel down.
 */
static int find_sdv_sites(int report)
{
    unsigned long base = (unsigned long)p_fg_psy_get_property;
    unsigned long size = 0, off = 0, i, n;
    ksym_size_off_t size_off;
    int shown = 0, n3300 = 0, n3100 = 0, sized = 1;

    sdv_addr[0] = sdv_addr[1] = 0;

    if (!base || !p_probe_kernel_read)
        return 0;

    size_off = (ksym_size_off_t)kallsyms_lookup_name("kallsyms_lookup_size_offset");
    if (!size_off || !size_off(base, &size, &off) || !size) {
        size = SDV_SCAN_FALLBACK;
        sized = 0;
    }

    if (report)
        pr_info(LOG_PREFIX "SDV scan: fg_psy_get_property=%px size=%d%s\n",
                (void *)base, (int)size, sized ? "" : " (fallback, size unknown)");

    n = size / 4;
    for (i = 0; i < n; i++) {
        unsigned long at = base + i * 4;
        unsigned int insn = 0, imm = 0;
        const char *kind, *tag = "";

        if (safe_read_ints((const void *)at, (int *)&insn, 1) != 0)
            break;

        if (INSN_IS_MOVZ32(insn) && INSN_MOVZ_HW(insn) == 0) {
            imm = INSN_MOVZ_IMM(insn);
            kind = "MOVZ";
        } else if (INSN_IS_MOVZ64(insn) && INSN_MOVZ_HW(insn) == 0) {
            imm = INSN_MOVZ_IMM(insn);
            kind = "MOVZx";
        } else if (INSN_IS_CMP32(insn) && INSN_CMP_SH(insn) == 0) {
            imm = INSN_CMP_IMM(insn);
            kind = "CMP ";
        } else {
            continue;
        }

        if (imm < SDV_MV_LO || imm > SDV_MV_HI)
            continue;

        /* Only a plain 32-bit MOVZ is safely rewritable in place; a CMP would
         * need the same treatment but we have never seen one here, so treat it
         * as "found but not patchable" and let the uniqueness test fail. */
        if (imm == 3300) {
            n3300++;
            tag = "  <== SHUTDOWN_DELAY_VOL";
            if (INSN_IS_MOVZ32(insn) && i * 4 >= SDV_ENTRY_GUARD) {
                sdv_addr[0] = (void *)at;
                sdv_orig[0] = insn;
            }
        }
        if (imm == 3100) {
            n3100++;
            tag = "  <== SHUTDOWN_DELAY_VOL_lOW_TEMP";
            if (INSN_IS_MOVZ32(insn) && i * 4 >= SDV_ENTRY_GUARD) {
                sdv_addr[1] = (void *)at;
                sdv_orig[1] = insn;
            }
        }

        if (report && shown++ < SDV_MAX_PRINT)
            pr_info(LOG_PREFIX "SDV   +0x%04x  insn=%08x  %s imm=%d%s\n",
                    (int)(i * 4), insn, kind, (int)imm, tag);
    }

    if (report)
        pr_info(LOG_PREFIX "SDV scan done: %d in range, 3300 x%d, 3100 x%d\n",
                shown, n3300, n3100);

    if (n3300 != 1 || !sdv_addr[0]) {
        sdv_addr[0] = sdv_addr[1] = 0;
        return 0;
    }
    if (n3100 != 1)
        sdv_addr[1] = 0;       /* patch the normal one only */
    return 1;
}

/*
 * Lower the driver's own countdown threshold instead of faking its output.
 *
 * The stock logic is exactly what we want -- capacity hits 0, and if there is
 * still voltage the user gets a 30 s warning before shutdown -- it is only
 * calibrated for graphite. Rewriting the two constants keeps that behaviour
 * bit-for-bit and just moves it down for a Si/C cell, which is far cleaner
 * than asserting or suppressing fg->shutdown_delay from our hook.
 *
 * We aim the threshold at our own floor, so warm/cold stay in step with
 * eff_shutdown_mv(): the driver picks between the two patched constants using
 * the same is_low_temp_flag we already mirror.
 *
 * Patching goes through the KERNEL's own aarch64_insn_patch_text(), resolved by
 * kallsyms -- not KernelPatch's hotpatch(). Both do the same job (stop_machine,
 * fixmap-writable alias, cache maintenance), but hotpatch() was only added to
 * the KP_EXPORT_SYMBOL table in kpimg 0.13.0, and referencing a symbol an older
 * kpimg does not export makes the module fail to LOAD AT ALL -- the same class
 * of failure as snprintf. The kernel symbol has no such version coupling.
 * Originals are kept and put back on unload.
 */
static void patch_shutdown_delay_vol(void)
{
    void *addrs[2];
    unsigned int vals[2];
    int warm, cold, cnt = 0, rc, i;

    if (sdv_from_arg == 0) {
        pr_info(LOG_PREFIX "SDV patch disabled (sdv=0)\n");
        return;
    }

    warm = sdv_from_arg > 0 ? sdv_from_arg : target_shutdown_mv;
    warm = clamp_i(warm, SHUTDOWN_MV_MIN, SHUTDOWN_MV_MAX);
    cold = clamp_i(warm - lt_delta_mv, SHUTDOWN_MV_MIN, SHUTDOWN_MV_MAX);

    if (!p_insn_patch_text) {
        pr_info(LOG_PREFIX "SDV: aarch64_insn_patch_text not found; leaving 3300 alone\n");
        return;
    }

    if (!find_sdv_sites(1)) {
        pr_info(LOG_PREFIX "SDV NOT unique or not a plain MOVZ -- refusing to patch\n");
        return;
    }

    addrs[cnt] = sdv_addr[0];
    vals[cnt++] = movz_set_imm(sdv_orig[0], (unsigned int)warm);
    if (sdv_addr[1]) {
        addrs[cnt] = sdv_addr[1];
        vals[cnt++] = movz_set_imm(sdv_orig[1], (unsigned int)cold);
    }

    rc = p_insn_patch_text(addrs, vals, cnt);
    if (rc) {
        pr_err(LOG_PREFIX "SDV patch failed rc=%d; stock 3300 retained\n", rc);
        return;
    }
    sdv_patched = cnt;

    /*
     * rc == 0 only says the API accepted the write. Read the words back and
     * confirm the immediate really changed -- an instruction patch that
     * silently did not land would be indistinguishable from success, and this
     * module has been bitten by exactly that failure mode twice already.
     */
    for (i = 0; i < cnt; i++) {
        unsigned int now = 0;

        if (safe_read_ints(addrs[i], (int *)&now, 1) != 0) {
            pr_err(LOG_PREFIX "SDV verify: cannot read back %px\n", addrs[i]);
            continue;
        }
        if (now != vals[i]) {
            pr_err(LOG_PREFIX "SDV verify FAILED @%px: want %08x got %08x\n",
                   addrs[i], vals[i], now);
            continue;
        }
        pr_info(LOG_PREFIX "SDV verified @%px: %08x -> %08x (imm %d -> %d)\n",
                addrs[i], sdv_orig[i], now,
                (int)INSN_MOVZ_IMM(sdv_orig[i]), (int)INSN_MOVZ_IMM(now));
    }

    pr_info(LOG_PREFIX "SDV active: countdown threshold %d mV warm%s\n",
            warm, sdv_addr[1] ? ", cold follows lt=" : " (low-temp site left stock)");
}

static void restore_shutdown_delay_vol(void)
{
    void *addrs[2];
    unsigned int vals[2];
    int i, cnt = 0;

    for (i = 0; i < sdv_patched; i++) {
        if (!sdv_addr[i])
            continue;
        addrs[cnt] = sdv_addr[i];
        vals[cnt++] = sdv_orig[i];
    }
    if (!cnt)
        return;
    if (!p_insn_patch_text || p_insn_patch_text(addrs, vals, cnt)) {
        pr_err(LOG_PREFIX "SDV restore FAILED -- constants left patched\n");
    } else {
        int ok = 1;

        for (i = 0; i < cnt; i++) {
            unsigned int now = 0;

            if (safe_read_ints(addrs[i], (int *)&now, 1) != 0 || now != vals[i])
                ok = 0;
        }
        pr_info(LOG_PREFIX "SDV restored to stock (%s)\n",
                ok ? "verified" : "READ-BACK MISMATCH");
    }
    sdv_patched = 0;
}

/*
 * Diagnostic: when the fingerprint scan fails, dump what the scanner actually
 * sees so a layout/pointer problem is visible in dmesg. Prints at most a few
 * times (diag_dumps_left) to avoid flooding. Uses printk formats directly
 * (printk is a resolved KP export; only snprintf was the missing symbol).
 */
static void diag_dump_chip(void *chip)
{
    unsigned long o;
    int probe1 = -999, first = 0;
    int hits = 0;
    int w[6];

    if (diag_dumps_left <= 0)
        return;
    diag_dumps_left--;

    probe1 = safe_read_ints(chip, &first, 1);
    pr_info(LOG_PREFIX "DIAG chip=%px pkr=%px probe_rc=%d first_int=%d sram_read=%px sram_cutoff=%d\n",
            chip, p_probe_kernel_read, probe1, first,
            p_fg_sram_read, read_cutoff_sram(chip));

    if (probe1 != 0) {
        pr_info(LOG_PREFIX "DIAG chip base unreadable via probe_kernel_read; arg0 is not a valid kernel ptr\n");
        return;
    }

    /* Scan a WIDE window (64 KiB), ignoring the SRAM cross-check, and print
     * every shape candidate so a layout change is visible. */
    for (o = 0; o + sizeof(w) <= 0x10000; o += sizeof(int)) {
        if (safe_read_ints((const char *)chip + o, w, 6) != 0) {
            pr_info(LOG_PREFIX "DIAG read stopped at +0x%lx (walked off mapping)\n", o);
            break;
        }
        if (dt_shape_ok(w)) {
            hits++;
            pr_info(LOG_PREFIX "DIAG cand@+0x%lx : cutoff=%d empty=%d sysmin=%d curr=%d systerm=%d ffc=%d\n",
                    o, w[0], w[1], w[2], w[3], w[4], w[5]);
        }
    }
    if (!hits) {
        pr_info(LOG_PREFIX "DIAG no dt-shaped window in 64 KiB from chip; first 32 ints:\n");
        for (o = 0; o < 32; o += 8) {
            int r[8];
            if (safe_read_ints((const char *)chip + o * sizeof(int), r, 8) != 0)
                break;
            pr_info(LOG_PREFIX "DIAG +0x%03lx: %d %d %d %d %d %d %d %d\n",
                    o * sizeof(int), r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
        }
    }
}

static int *find_cutoff_field(void *chip)
{
    unsigned long o, hit_off = 0;
    int sram_mv = read_cutoff_sram(chip);
    int hits = 0;
    int w[6];

    /*
     * One pass over the whole window. Shape-match every 6-int candidate and,
     * when SRAM gave us an anchor, require cutoff_volt_mv to agree with what
     * the driver actually programmed. Accept only a UNIQUE survivor.
     */
    for (o = 0; o + sizeof(w) <= SCAN_MAX_BYTES; o += sizeof(int)) {
        if (safe_read_ints((const char *)chip + o, w, 6) != 0)
            break; /* walked off the allocation */

        if (!dt_shape_ok(w))
            continue;

        if (sram_mv > 0) {
            int d = w[0] - sram_mv;

            if (d < 0)
                d = -d;
            if (d > FP_SRAM_TOL_MV)
                continue;
        }

        hits++;
        if (hits == 1)
            hit_off = o;
        else
            pr_info(LOG_PREFIX "extra dt candidate @+0x%lx: %d/%d/%d %d/%d/%d\n",
                    o, w[0], w[1], w[2], w[3], w[4], w[5]);
    }

    if (hits == 1) {
        if (sram_mv > 0)
            pr_info(LOG_PREFIX "dt located @+0x%lx (SRAM cutoff %d mV confirms)\n",
                    hit_off, sram_mv);
        else
            pr_info(LOG_PREFIX "dt located @+0x%lx (shape only; SRAM unavailable)\n",
                    hit_off);
        return (int *)((char *)chip + hit_off);
    }

    if (hits > 1)
        pr_info(LOG_PREFIX "%d dt candidates — ambiguous, refusing to write\n", hits);

    return NULL;
}

static void encode_volt(int mv, unsigned char *buf, int len,
                        int numrtr, int denmtr, int voff)
{
    long long temp = ((long long)(mv + voff) * numrtr) / denmtr;
    int i;

    for (i = 0; i < len; i++) {
        buf[i] = (unsigned char)(temp & 0xff);
        temp >>= 8;
    }
}

static int sram_write(void *fg, int word, int byte, unsigned char *buf, int len)
{
    if (!p_fg_sram_write)
        return -1;
    return p_fg_sram_write(fg, (unsigned short)word, (unsigned char)byte,
                           buf, len, FG_IMA_DEFAULT);
}

static void write_cutoff_sram(void *fg, int mv, int log)
{
    unsigned char buf[CUTOFF_VOLT_LEN];
    int rc;

    encode_volt(mv, buf, CUTOFF_VOLT_LEN, CUTOFF_VOLT_NUMRTR,
                CUTOFF_VOLT_DENMTR, CUTOFF_VOLT_VOFF);
    rc = sram_write(fg, CUTOFF_VOLT_WORD, CUTOFF_VOLT_BYTE, buf, CUTOFF_VOLT_LEN);
    if (log)
        pr_info(LOG_PREFIX "SRAM CUTOFF_VOLT <- %d mV (raw %02x %02x) rc=%d\n",
                mv, buf[0], buf[1], rc);
}

static void write_empty_sram(void *fg, int mv, int log)
{
    unsigned char buf[VBATT_LOW_LEN];
    int rc;

    encode_volt(mv, buf, VBATT_LOW_LEN, VBATT_LOW_NUMRTR,
                VBATT_LOW_DENMTR, VBATT_LOW_VOFF);
    rc = sram_write(fg, VBATT_LOW_WORD, VBATT_LOW_BYTE, buf, VBATT_LOW_LEN);
    if (log)
        pr_info(LOG_PREFIX "SRAM VBATT_LOW  <- %d mV (raw %02x) rc=%d\n",
                mv, buf[0], rc);
}

static void apply_from_chip(void *chip)
{
    int *field = find_cutoff_field(chip);

    if (!field) {
        if (diag_dumps_left > 0) {
            pr_info(LOG_PREFIX "fingerprint not found; dumping chip memory\n");
            diag_dump_chip(chip);
        }
        return;
    }

    chip_ptr     = chip;
    cutoff_field = field;
    empty_field  = field + 1;          /* empty_volt_mv immediately follows */
    if (orig_cutoff_mv < 0)
        orig_cutoff_mv = *cutoff_field;
    if (orig_empty_mv < 0)
        orig_empty_mv = *empty_field;

    *cutoff_field = eff_cutoff_mv();
    *empty_field  = eff_empty_mv();
    write_cutoff_sram(chip, eff_cutoff_mv(), 1);
    write_empty_sram(chip, eff_empty_mv(), 1);

    pr_info(LOG_PREFIX "applied: cutoff %d->%d, empty %d->%d, shutdown floor %d mV (dt @ %px)\n",
            orig_cutoff_mv, eff_cutoff_mv(), orig_empty_mv, eff_empty_mv(),
            eff_shutdown_mv(), field);
}

/* Read the driver's own bool. 1 byte; -1 if unavailable. */
static int read_driver_cold_flag(int *out)
{
    unsigned char b = 0;

    if (!p_is_low_temp_flag || !p_probe_kernel_read)
        return -1;
    if (p_probe_kernel_read(&b, p_is_low_temp_flag, 1) != 0)
        return -1;
    *out = b ? 1 : 0;
    return 0;
}

/*
 * Mirror the driver's cold verdict. Returns 1 if the tier changed.
 *
 * is_low_temp_flag is the driver's single source of truth for "cold": its own
 * cutoff pair and its SHUTDOWN_DELAY_VOL_lOW_TEMP both key off it. Following it
 * verbatim -- no threshold of our own, no hysteresis -- is what keeps us in
 * step: a threshold or hysteresis on our side would only create windows where
 * the driver says cold and we say warm, or the reverse.
 *
 * If the symbol is absent, or the driver never sets it (its low-temp path is
 * gated on qcom,cutoff-voltage-adjust-enable, which lmi does not set), we stay
 * in the warm tier forever -- which is exactly right: that kernel has no
 * low-temp policy for us to shift.
 */
static int update_low_temp_tier(void)
{
    int flag = 0, was = in_low_temp;

    if (read_driver_cold_flag(&flag) != 0)
        return 0;

    in_low_temp = flag;
    if (in_low_temp == was)
        return 0;

    pr_info(LOG_PREFIX "driver is_low_temp_flag=%d -> %s tier: cutoff=%d empty=%d shutdown=%d mV\n",
            flag, in_low_temp ? "COLD" : "normal",
            eff_cutoff_mv(), eff_empty_mv(), eff_shutdown_mv());
    return 1;
}

/* ------------------------------------------------------------------ */
/* Hook 1: fg_get_battery_voltage(struct fg_dev *fg, int *val)        */
/* arg0 == fg == chip base. Locates + applies cutoff/empty.           */
/* ------------------------------------------------------------------ */

static void before_get_voltage(hook_fargs2_t *args, void *udata)
{
    void *chip = (void *)args->arg0;
    int clobbered = 0, tier_changed = 0, driver_wrote = 0;

    if (!chip)
        return;

    if (chip != chip_ptr || !cutoff_field) {
        update_low_temp_tier();
        apply_from_chip(chip);
        return;
    }

    /* A tier change makes the stored values stale -- treat it as a clobber so
     * both the C fields and SRAM are rewritten below. */
    tier_changed = update_low_temp_tier();
    if (tier_changed)
        clobbered = 1;

    if (*cutoff_field != eff_cutoff_mv()) {
        driver_wrote = *cutoff_field;
        *cutoff_field = eff_cutoff_mv();
        clobbered = 1;
    }
    if (empty_field && *empty_field != eff_empty_mv()) {
        *empty_field = eff_empty_mv();
        clobbered = 1;
    }

    /*
     * The driver overwrote the dt block. On trees that set
     * qcom,cutoff-voltage-adjust-enable (umi does; lmi does not),
     * soc_monitor_work() rewrites dt.cutoff_volt_mv every 10 s AND reprograms
     * SRAM CUTOFF_VOLT via fg_dynamic_set_cutoff_voltage(). Restoring the C
     * field alone would leave the hardware msoc=0% anchor at the stock value,
     * so re-assert SRAM as well. Quiet: this can repeat every monitor tick.
     */
    if (clobbered) {
        write_cutoff_sram(chip, eff_cutoff_mv(), 0);
        write_empty_sram(chip, eff_empty_mv(), 0);
        if (!tier_changed && driver_wrote && reassert_logs_left > 0) {
            reassert_logs_left--;
            pr_info(LOG_PREFIX "driver wrote cutoff=%d mV; re-asserted %d mV (C field + SRAM)%s\n",
                    driver_wrote, eff_cutoff_mv(),
                    reassert_logs_left ? "" : " (further notices suppressed)");
        }
    }
}

/* ------------------------------------------------------------------ */
/* Hook 2: fg_psy_get_property(psy, psp, val) — AFTER hook            */
/* Extends the "hang at 1%" window down to target_shutdown_mv.        */
/* ------------------------------------------------------------------ */

static void after_get_property(hook_fargs3_t *args, void *udata)
{
    int psp = (int)args->arg1;
    int *pintval;
    int vbatt_uv = 0;
    fg_get_batt_volt_t get_v;

    if (psp != psp_capacity || !args->arg2)
        return;

    pintval = (int *)args->arg2;   /* intval is the first member of the union */
    if (*pintval != 0)
        return;                    /* only act once the driver reports 0%     */

    if (!chip_ptr || !p_fg_get_battery_voltage)
        return;

    /* Report 1% while instantaneous vbatt is still above our floor, i.e. carry
     * the shutdown point down from the stock 3300 mV to target_shutdown_mv.
     * This also masks a premature rapid-SOC-forced 0% for the same window. */
    get_v = (fg_get_batt_volt_t)p_fg_get_battery_voltage;
    if (get_v(chip_ptr, &vbatt_uv) == 0 && vbatt_uv > eff_shutdown_mv() * 1000)
        *pintval = 1;
}

/* ------------------------------------------------------------------ */
/* Hook 3: fg_get_msoc(struct fg_dev *fg, int *msoc) — AFTER hook     */
/* Stops the hardware msoc from vetoing the FVSS voltage SOC.          */
/* ------------------------------------------------------------------ */

/*
 * WHY.
 *
 * Below 3600 mV (3400 when the driver says cold) the driver enters FVSS and
 * stops trusting the gauge's coulomb counter, deriving SOC linearly from
 * filtered voltage instead:
 *
 *     soc = (vbatt_avg - dt.cutoff_volt_mv) / soc_scale_slope
 *
 * That is exactly what we want on a Si/C cell, because the profile programmed
 * into the gauge is still the stock graphite one and its msoc collapses early.
 * But soc_scale_work() does not report that soc unconditionally. Its first
 * branch reads:
 *
 *     if ((prev_soc_scale_msoc - msoc_actual) > soc_thr_percent)
 *             soc_scale_msoc = prev_soc_scale_msoc - soc_thr_percent;
 *
 * i.e. whenever the voltage-derived SOC sits more than one percent ABOVE the
 * hardware msoc, throw the voltage away and step down 1% per tick. Feeding a
 * graphite profile from a Si/C cell keeps that condition true for the entire
 * tail, so the display is walked down to the gauge's pessimistic msoc and parks
 * at 1% while the cell still holds a large usable reserve.
 *
 * WHAT WE DO.
 *
 * chip->msoc_actual has exactly ONE writer -- fg_gen4_validate_soc_scale_mode()
 * -- and exactly TWO readers: the veto above, and the charging-side FVSS exit
 * test "msoc_actual >= soc_scale_msoc". Nothing else in the driver touches it.
 * So handing that one field prev_soc_scale_msoc instead of the hardware value:
 *
 *   - makes (prev - msoc_actual) == 0, permanently disabling the veto and
 *     leaving soc_scale_work()'s remaining branches in charge: report the
 *     voltage SOC, never let it rise, and rate-limit a fast drop to 1% per
 *     tick. That IS pure FVSS, with the driver's own smoothing intact.
 *   - makes the charging exit test true at once, so plugging in leaves FVSS
 *     immediately. Better than stock, not worse: fg_gen4_exit_soc_scale() calls
 *     fg_gen4_write_scale_msoc() on the way out, which programs the scaled SOC
 *     into the hardware MONOTONIC_SOC register, so the displayed value carries
 *     over seamlessly and then climbs with the charge -- instead of freezing
 *     until an inaccurate HW msoc catches up to it.
 *   - changes NOTHING outside FVSS. msoc_actual has no other consumer, and
 *     every other caller of fg_get_msoc() -- including the one behind
 *     POWER_SUPPLY_PROP_CAPACITY when FVSS is off -- passes a stack local,
 *     which the identity test below rejects.
 *
 * WHAT IT DOES NOT FIX: the FVSS ENTRY soc still comes from the gauge, because
 * fg_gen4_enter_soc_scale() seeds soc_scale_msoc from the currently reported
 * capacity and derives the slope from it. So the tail is now spread evenly from
 * wherever the gauge thought we were at 3600 mV down to cutoff -- no more early
 * collapse and no more hour at 1% -- but that entry point is still profile
 * dependent. Correcting it would mean replacing the battery profile, which is a
 * different job from moving the floors.
 */

/*
 * Is `w` (a window of seven ints centred on the candidate) really the
 * neighbourhood of chip->msoc_actual?
 *
 * struct fg_gen4_chip lays these out consecutively:
 *
 *   -3 soc_scale_msoc  -2 prev_soc_scale_msoc  -1 soc_scale_slope
 *    0 msoc_actual     +1 vbatt_avg  +2 vbatt_now  +3 vbatt_res
 *
 * and the one caller that passes us a pointer into the chip object runs
 * fg_gen4_get_prop_soc_scale() immediately beforehand, which fills those three
 * voltage words and finishes with
 *
 *     vbatt_res = vbatt_avg - dt.cutoff_volt_mv
 *
 * That is an exact arithmetic identity between three words we can read,
 * anchored on the cutoff value THIS MODULE placed in the dt block -- the same
 * spirit as the SRAM cross-check that locates the dt block itself, and again
 * with no per-device constant of our own. Note the ordering that makes the
 * anchor sound: fg_gen4_get_prop_soc_scale() calls fg_get_battery_voltage()
 * (where hook 1 re-asserts our cutoff) BEFORE it computes vbatt_res, so by the
 * time the identity is formed our value is the one in the field.
 *
 * slope > 0 is only true once fg_gen4_enter_soc_scale() has run, so the field
 * is confirmed on the first tick where the veto could actually fire -- exactly
 * when we need it, and never on the basis of a zeroed, never-used struct. A
 * tick that fails any test simply does not confirm and we try the next one, so
 * losing a race against another CPU reprogramming the cutoff costs nothing.
 */
static int msoc_shape_ok(const int *w, int cutoff_mv)
{
    return w[0] >= 0 && w[0] <= 100 &&                 /* soc_scale_msoc      */
           w[1] >= 0 && w[1] <= 100 &&                 /* prev_soc_scale_msoc */
           w[2] >  0 && w[2] <= FP_MV_HI &&            /* soc_scale_slope     */
           w[3] >= 0 && w[3] <= 100 &&                 /* msoc_actual         */
           w[4] >= FP_MV_LO && w[4] <= FP_MV_HI &&     /* vbatt_avg           */
           w[5] >= FP_MV_LO && w[5] <= FP_MV_HI &&     /* vbatt_now           */
           w[6] == w[4] - cutoff_mv;                   /* vbatt_res identity  */
}

static void after_get_msoc(hook_fargs2_t *args, void *udata)
{
    void *chip = (void *)args->arg0;
    int *dst = (int *)args->arg1;
    unsigned long off;
    int w[7];

    if (!fvss_pure || (int)args->ret != 0 || !dst)
        return;

    /* No located dt block means no cutoff anchor, so no identity test and no
     * writes. chip_ptr is set by hook 1, which runs constantly. */
    if (!chip || chip != chip_ptr || !cutoff_field)
        return;

    off = (unsigned long)((char *)dst - (char *)chip);
    if (off & (sizeof(int) - 1))
        return;
    /* Below the window we need to read, or past the object: a stack local. A
     * dst < chip wraps the unsigned subtraction and is caught by the bound. */
    if (off < 3 * sizeof(int) || off + 4 * sizeof(int) > MSOC_CHIP_SPAN)
        return;
    /* Once identified, the field never moves; a second in-object address would
     * mean the identification was wrong, so refuse rather than guess. */
    if (msoc_actual_off >= 0 && off != (unsigned long)msoc_actual_off)
        return;

    if (safe_read_ints(dst - 3, w, 7) != 0)
        return;

    if (!msoc_shape_ok(w, *cutoff_field)) {
        /* Only interesting once FVSS is running (slope set): before that the
         * struct is legitimately zeroed and failing is the correct answer. */
        if (msoc_actual_off < 0 && w[2] > 0 && fvss_id_logs_left > 0) {
            fvss_id_logs_left--;
            pr_info(LOG_PREFIX "FVSS: chip+0x%lx failed identity: soc=%d prev=%d slope=%d msoc=%d vavg=%d vnow=%d vres=%d cutoff=%d%s\n",
                    off, w[0], w[1], w[2], w[3], w[4], w[5], w[6], *cutoff_field,
                    fvss_id_logs_left ? "" : " (further notices suppressed)");
        }
        return;
    }

    if (msoc_actual_off < 0) {
        msoc_actual_off = (long)off;
        pr_info(LOG_PREFIX "FVSS: msoc_actual located @chip+0x%lx; HW msoc no longer vetoes the voltage SOC\n",
                off);
    }

    /*
     * prev_soc_scale_msoc -- the very value soc_scale_work() is about to
     * subtract this field from, so the veto sees a difference of exactly zero.
     */
    *dst = w[1];
    fvss_overrides++;

    if (fvss_logs_left > 0 && w[3] != w[1]) {
        fvss_logs_left--;
        pr_info(LOG_PREFIX "FVSS: HW msoc %d ignored, keeping voltage SOC %d (vbatt avg %d mV)%s\n",
                w[3], w[1], w[4],
                fvss_logs_left ? "" : " (further notices suppressed)");
    }
}

/* ------------------------------------------------------------------ */
/* KPM lifecycle                                                      */
/* ------------------------------------------------------------------ */

#define ARG_TOK_MAX   32       /* longest token we echo back in a message */
#define ARG_VAL_MAX   1000000  /* refuse before an int could overflow       */

/*
 * Whitespace only PADS -- runs of it collapse. ',' and ';' DELIMIT, so each one
 * starts a new field and two in a row make an empty one, which is what lets
 * "3200,,3100" mean "leave empty_volt alone" instead of silently sliding 3100
 * into it.
 */
static int is_ws(char c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

static int is_delim(char c)
{
    return c == ',' || c == ';';
}

/* The WHOLE of [s, end) must be plain decimal digits. 0 on success. */
static int parse_uint(const char *s, const char *end, int *out)
{
    long long v = 0;

    if (s >= end)
        return -1;                     /* empty token */
    for (; s < end; s++) {
        if (*s < '0' || *s > '9')
            return -1;                 /* not a plain decimal */
        v = v * 10 + (*s - '0');
        if (v > ARG_VAL_MAX)
            return -1;                 /* absurd, and about to overflow */
    }
    *out = (int)v;
    return 0;
}

/* Case-insensitive compare of [s, end) against a NUL-terminated key. */
static int key_eq(const char *s, const char *end, const char *key)
{
    for (; s < end && *key; s++, key++) {
        char c = (*s >= 'A' && *s <= 'Z') ? (char)(*s + 32) : *s;

        if (c != *key)
            return 0;
    }
    return s == end && !*key;
}

/* NUL-terminated copy of [s, end), truncated to fit, for log messages. */
static void tok_copy(char *buf, int len, const char *s, const char *end)
{
    int n = 0;

    while (s < end && n < len - 1)
        buf[n++] = *s++;
    buf[n] = '\0';
}

/*
 * Apply one value, or refuse it and leave the parameter alone.
 *
 * "Leave it alone" is the whole point: at load time that is the compiled
 * default, from KPM_CTL0 it is whatever is currently live. Either way a
 * parameter we could not validate never changes, and never gets coerced into
 * something the caller did not ask for.
 */
static int arg_set(const char *what, int v, int lo, int hi, int *dst, int *rejected)
{
    if (v < lo || v > hi) {
        pr_err(LOG_PREFIX "arg: %s=%d out of range [%d,%d] -- rejected, keeping %d\n",
               what, v, lo, hi, *dst);
        (*rejected)++;
        return -1;
    }
    *dst = v;
    return 0;
}

/*
 * Strict parser shared by the load-time args and KPM_CTL0.
 *
 * Grammar: fields separated by ',' or ';', with whitespace as padding (runs of
 * whitespace also separate). A field is either "key=value", a bare decimal, or
 * empty; bare decimals are positional in the order cutoff, empty, shutdown, and
 * an EMPTY field consumes its slot without setting anything:
 *
 *     "3200"   "3200,3000,3100"   "cutoff=3200,shutdown=3100"
 *     "3200,,3100"   -> cutoff and shutdown, empty_volt left alone
 *     "3200,3000,3100,lt=150,sdv=0,fvss=0,psp=44"
 *
 * Keys: cutoff, empty, shutdown (mV, same clamps as the positional form),
 *       lt (0..200), sdv (0 = never patch code, else mV), fvss (0|1),
 *       psp (CAPACITY ordinal override).
 *
 * THE CONTRACT: anything we cannot parse AND range-check leaves the parameter
 * it would have set at its current value, and says so in the log. Nothing is
 * silently coerced. That is a deliberate reversal of the old behaviour, which
 * clamped: "320" -- a 3200 with a dropped zero -- became clamp(320, 2800, 3400)
 * == 2800, i.e. one typo silently selecting the most aggressive floor the
 * module allows, with a brown-out reboot as the failure mode. Out of range is
 * now a rejection, not a clamp.
 *
 * A malformed POSITIONAL token additionally discards the whole positional
 * sequence, because position is the only thing that gives a bare number its
 * meaning: in "3200,x,3100" there is no way to know whether 3100 was meant as
 * empty or as shutdown, so neither is applied. Well-formed-but-out-of-range is
 * per-value instead -- there the positions are unambiguous, so only the
 * offending value is dropped. Use the key= form to avoid the question.
 *
 * Two passes: the first decides only whether the positional sequence can be
 * trusted, the second applies. So everything takes effect in the order written
 * and, for any one parameter, the last mention wins.
 *
 * Returns the number of rejected tokens (0 = everything understood).
 */
static int set_targets_from_args(const char *args)
{
    int pos_n = 0, pos_bad = 0, rejected = 0, pass;

    if (!args)
        return 0;

    for (pass = 0; pass < 2; pass++) {
        const char *s;

        if (pass == 1 && pos_bad) {
            pr_err(LOG_PREFIX "arg: malformed positional value -- cutoff/empty/shutdown ALL left unchanged (use cutoff=/empty=/shutdown= to be explicit)\n");
            rejected++;
        }

        pos_n = 0;
        s = args;
        for (;;) {
            const char *tok, *end, *eq;
            char tb[ARG_TOK_MAX];
            int v, more;

            while (is_ws(*s))
                s++;
            tok = s;
            while (*s && !is_ws(*s) && !is_delim(*s))
                s++;
            end = s;
            while (is_ws(*s))
                s++;

            /*
             * The field is now fully captured in [tok, end), so consume its
             * trailing delimiter HERE, once. Every path below can then
             * `continue` freely: `s` has already advanced, so no branch can
             * ever fail to make progress. In a kernel module that is worth a
             * little bookkeeping -- the failure mode of a non-advancing loop
             * is a hung CPU, not a wrong value.
             */
            more = is_delim(*s);
            if (more)
                s++;

            if (end == tok) {
                /*
                 * An empty field. It consumes its positional slot and sets
                 * nothing, so "3200,,3100" leaves empty_volt at its current
                 * value and still puts 3100 in shutdown -- nothing shifts into
                 * the wrong parameter, which is the entire hazard an empty
                 * field would otherwise create. A trailing ',' is therefore
                 * harmless; only running out of string ends the scan.
                 */
                if (!more)
                    break;
                if (pos_n < 3)
                    pos_n++;
                continue;
            }

            for (eq = tok; eq < end && *eq != '='; eq++)
                ;

            /* ---- bare decimal: positional ---- */
            if (eq == end) {
                if (parse_uint(tok, end, &v) != 0) {
                    pos_bad = 1;       /* pass 0 detects, pass 1 already knows */
                    continue;
                }
                if (pos_bad)
                    continue;
                if (pos_n >= 3) {
                    if (pass == 1) {
                        tok_copy(tb, sizeof(tb), tok, end);
                        pr_err(LOG_PREFIX "arg: '%s' is a 4th positional value -- rejected\n", tb);
                        rejected++;
                    }
                    continue;
                }
                if (pass == 1) {
                    if (pos_n == 0)
                        arg_set("cutoff", v, CUTOFF_MV_MIN, CUTOFF_MV_MAX,
                                &target_cutoff_mv, &rejected);
                    else if (pos_n == 1)
                        arg_set("empty", v, EMPTY_MV_MIN, EMPTY_MV_MAX,
                                &target_empty_mv, &rejected);
                    else
                        arg_set("shutdown", v, SHUTDOWN_MV_MIN, SHUTDOWN_MV_MAX,
                                &target_shutdown_mv, &rejected);
                }
                pos_n++;
                continue;
            }

            /* ---- key=value ---- */
            if (pass != 1)
                continue;

            tok_copy(tb, sizeof(tb), tok, end);

            if (parse_uint(eq + 1, end, &v) != 0) {
                pr_err(LOG_PREFIX "arg: '%s' has no plain decimal value -- rejected\n", tb);
                rejected++;
                continue;
            }

            if (key_eq(tok, eq, "cutoff"))
                arg_set("cutoff", v, CUTOFF_MV_MIN, CUTOFF_MV_MAX,
                        &target_cutoff_mv, &rejected);
            else if (key_eq(tok, eq, "empty"))
                arg_set("empty", v, EMPTY_MV_MIN, EMPTY_MV_MAX,
                        &target_empty_mv, &rejected);
            else if (key_eq(tok, eq, "shutdown"))
                arg_set("shutdown", v, SHUTDOWN_MV_MIN, SHUTDOWN_MV_MAX,
                        &target_shutdown_mv, &rejected);
            else if (key_eq(tok, eq, "lt"))
                arg_set("lt", v, 0, LT_DELTA_MV_MAX, &lt_delta_mv, &rejected);
            else if (key_eq(tok, eq, "fvss"))
                arg_set("fvss", v, 0, 1, &fvss_pure, &rejected);
            else if (key_eq(tok, eq, "psp"))
                arg_set("psp", v, 0, PSP_MAX, &psp_from_arg, &rejected);
            else if (key_eq(tok, eq, "sdv")) {
                /* 0 is not a voltage here, it means "never touch the
                 * instruction stream"; any other value is a target mV. */
                if (v == 0)
                    sdv_from_arg = 0;
                else
                    arg_set("sdv", v, SHUTDOWN_MV_MIN, SHUTDOWN_MV_MAX,
                            &sdv_from_arg, &rejected);
            } else {
                pr_err(LOG_PREFIX "arg: unknown key in '%s' -- rejected (known: cutoff empty shutdown lt sdv fvss psp)\n",
                       tb);
                rejected++;
            }
        }
    }

    /*
     * Every value is individually inside a safe range by now, so an odd
     * ordering is not dangerous -- but it does mean the cutoff -> shutdown
     * "hang at 1%" window is inverted or empty, which is almost certainly not
     * what was intended. Say so rather than silently obeying it.
     */
    if (target_shutdown_mv > target_cutoff_mv || target_empty_mv > target_shutdown_mv)
        pr_info(LOG_PREFIX "arg: note -- expected cutoff >= shutdown >= empty, have %d >= %d >= %d\n",
                target_cutoff_mv, target_shutdown_mv, target_empty_mv);

    if (rejected)
        pr_err(LOG_PREFIX "arg: %d token(s) rejected; those parameters keep their previous values\n",
               rejected);

    return rejected;
}

static long fg_cutoff_init(const char *args, const char *event, void *__user reserved)
{
    hook_err_t err;
    int rejected;

    rejected = set_targets_from_args(args);

    /* Effective values, AFTER parsing: whatever a rejected token would have
     * set is still at its compiled default here, so this line is the truth
     * about what the module is about to apply. */
    pr_info(LOG_PREFIX "init (event=%s, args=%s) cutoff=%d empty=%d shutdown=%d mV lt=%d%s\n",
            event ? event : "?", args ? args : "", target_cutoff_mv,
            target_empty_mv, target_shutdown_mv, lt_delta_mv,
            rejected ? " [SOME ARGS REJECTED -- see above; defaults kept]" : "");

    p_probe_kernel_read = (probe_kernel_read_t)kallsyms_lookup_name("probe_kernel_read");
    if (!p_probe_kernel_read)
        p_probe_kernel_read = (probe_kernel_read_t)kallsyms_lookup_name("__probe_kernel_read");
    p_fg_sram_write          = (fg_sram_write_t)kallsyms_lookup_name("fg_sram_write");
    p_fg_sram_read           = (fg_sram_read_t)kallsyms_lookup_name("fg_sram_read");
    p_fg_get_battery_voltage = (void *)kallsyms_lookup_name("fg_get_battery_voltage");
    p_fg_psy_get_property    = (void *)kallsyms_lookup_name("fg_psy_get_property");
    p_fg_get_msoc            = (void *)kallsyms_lookup_name("fg_get_msoc");
    p_is_low_temp_flag       = (const void *)kallsyms_lookup_name("is_low_temp_flag");
    p_insn_patch_text        = (insn_patch_text_t)kallsyms_lookup_name("aarch64_insn_patch_text");

    pr_info(LOG_PREFIX "syms: pkr=%px sram_write=%px sram_read=%px get_voltage=%px get_property=%px get_msoc=%px\n",
            p_probe_kernel_read, p_fg_sram_write, p_fg_sram_read,
            p_fg_get_battery_voltage, p_fg_psy_get_property, p_fg_get_msoc);

    if (!p_probe_kernel_read || !p_fg_get_battery_voltage) {
        pr_err(LOG_PREFIX "missing required symbols; aborting\n");
        return -1;
    }
    if (!p_fg_sram_write)
        pr_info(LOG_PREFIX "fg_sram_write not found; C-field only\n");
    if (!p_fg_sram_read)
        pr_info(LOG_PREFIX "fg_sram_read not found; dt scan falls back to shape+uniqueness\n");
    pr_info(LOG_PREFIX "low-temp tier: -%d mV, mirroring is_low_temp_flag=%px (%s)\n",
            lt_delta_mv, p_is_low_temp_flag,
            p_is_low_temp_flag ? "driver decides when cold"
                               : "absent -- driver has no low-temp policy, tier stays off");

    /* CAPACITY ordinal: explicit arg > by-name lookup > compiled fallback. */
    if (psp_from_arg >= 0) {
        psp_capacity = psp_from_arg;
        pr_info(LOG_PREFIX "psp: CAPACITY=%d (from args)\n", psp_capacity);
    } else {
        psp_capacity = resolve_psp_capacity();
        if (psp_capacity < 0) {
            psp_capacity = PSP_CAPACITY_FALLBACK;
            pr_info(LOG_PREFIX "psp: name lookup failed, using fallback %d — verify, or pass psp=<n>\n",
                    psp_capacity);
        }
    }

    err = hook_wrap2(p_fg_get_battery_voltage, before_get_voltage, 0, 0);
    if (err) {
        pr_err(LOG_PREFIX "hook_wrap2(get_voltage) failed err=%d\n", err);
        return -1;
    }

    /* Move the driver's own countdown threshold down to our floor, before the
     * hook rewrites the function entry. */
    patch_shutdown_delay_vol();

    /* Shutdown-floor extension: optional but recommended. If the symbol is
     * missing we still deliver cutoff/empty. */
    if (p_fg_psy_get_property) {
        err = hook_wrap3(p_fg_psy_get_property, 0, after_get_property, 0);
        if (err) {
            pr_err(LOG_PREFIX "hook_wrap3(get_property) failed err=%d; shutdown floor NOT active\n",
                   err);
            p_fg_psy_get_property = 0;
        }
    } else {
        pr_info(LOG_PREFIX "fg_psy_get_property not found; shutdown floor stays stock 3300 mV\n");
    }

    /*
     * Pure FVSS: optional, and hooked unconditionally so "fvss=" stays live
     * through KPM_CTL0 -- the gate inside the hook is one compare against the
     * two SPMI SRAM reads fg_get_msoc() itself does. Without it the floors
     * still apply, the tail is just still walked down by the gauge's own
     * pessimistic msoc.
     */
    if (p_fg_get_msoc) {
        err = hook_wrap2(p_fg_get_msoc, 0, after_get_msoc, 0);
        if (err) {
            pr_err(LOG_PREFIX "hook_wrap2(get_msoc) failed err=%d; pure FVSS NOT available\n",
                   err);
            p_fg_get_msoc = 0;
        } else {
            pr_info(LOG_PREFIX "pure FVSS %s; msoc_actual is identified on the first FVSS tick\n",
                    fvss_pure ? "armed" : "hooked but disabled (fvss=0)");
        }
    } else {
        pr_info(LOG_PREFIX "fg_get_msoc not found; FVSS SOC stays HW-msoc-vetoed\n");
    }

    pr_info(LOG_PREFIX "hooks installed; applying on next FG voltage read\n");
    return 0;
}

/*
 * Reported back to userspace so a rejected argument is visible without going
 * to dmesg -- the values printed alongside it are the ones actually in force.
 */
static const char *arg_reject_note(int rejected)
{
    return rejected ? " [SOME ARGS REJECTED -- unchanged; see dmesg]" : "";
}

/* "off" | "armed" (hooked, field not yet identified) | "active". */
static const char *fvss_state_str(void)
{
    if (!fvss_pure || !p_fg_get_msoc)
        return "off";
    return msoc_actual_off >= 0 ? "active" : "armed";
}

/* Runtime reconfigure: kpm control battery-fg-cutoff "<cutoff>[,<empty>[,<shutdown>]]" */
static long fg_cutoff_control0(const char *args, char *__user out_msg, int outlen)
{
    char msg[256];
    int len, rejected;

    rejected = set_targets_from_args(args);

    if (cutoff_field && chip_ptr) {
        *cutoff_field = eff_cutoff_mv();
        if (empty_field)
            *empty_field = eff_empty_mv();
        write_cutoff_sram(chip_ptr, eff_cutoff_mv(), 1);
        write_empty_sram(chip_ptr, eff_empty_mv(), 1);
        len = fmt_msg(msg, (int)sizeof(msg),
                      "cutoff=%d empty=%d shutdown=%d mV live (stock %d/%d) fvss=%s (%d)%s\n",
                      eff_cutoff_mv(), eff_empty_mv(), eff_shutdown_mv(),
                      orig_cutoff_mv, orig_empty_mv, fvss_state_str(),
                      fvss_overrides, arg_reject_note(rejected));
    } else {
        len = fmt_msg(msg, (int)sizeof(msg),
                      "cutoff=%d empty=%d shutdown=%d mV queued (not yet located) fvss=%s%s\n",
                      eff_cutoff_mv(), eff_empty_mv(), eff_shutdown_mv(),
                      fvss_state_str(), arg_reject_note(rejected));
    }

    pr_info(LOG_PREFIX "%s", msg);
    /* len + 1 for the NUL — never hand userspace the uninitialised tail. */
    if (out_msg && outlen > 0) {
        int n = outlen < len + 1 ? outlen : len + 1;
        compat_copy_to_user(out_msg, msg, n);
    }
    return 0;
}

static long fg_cutoff_exit(void *__user reserved)
{
    if (cutoff_field && orig_cutoff_mv > 0) {
        *cutoff_field = orig_cutoff_mv;
        if (empty_field && orig_empty_mv > 0)
            *empty_field = orig_empty_mv;
        if (chip_ptr) {
            write_cutoff_sram(chip_ptr, orig_cutoff_mv, 1);
            if (orig_empty_mv > 0)
                write_empty_sram(chip_ptr, orig_empty_mv, 1);
        }
        pr_info(LOG_PREFIX "restored cutoff=%d empty=%d mV\n",
                orig_cutoff_mv, orig_empty_mv);
    }

    restore_shutdown_delay_vol();

    if (p_fg_get_msoc)
        unhook(p_fg_get_msoc);
    if (p_fg_psy_get_property)
        unhook(p_fg_psy_get_property);
    if (p_fg_get_battery_voltage)
        unhook(p_fg_get_battery_voltage);

    pr_info(LOG_PREFIX "exit\n");
    return 0;
}

KPM_INIT(fg_cutoff_init);
KPM_CTL0(fg_cutoff_control0);
KPM_EXIT(fg_cutoff_exit);
