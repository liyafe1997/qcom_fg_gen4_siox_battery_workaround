// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * fg_cutoff.kpm — SukiSU / KernelPatch KPM module
 *
 * Lowers the QPNP FG-Gen4 discharge floor on the Redmi K30 Pro (lmi) so a
 * silicon/carbon (Si/SiOx) replacement cell is not reported 0% while it still
 * has a usable sub-3.4V tail. EVERYTHING is done from this module — no kernel
 * source edits. Conservative tier defaults:
 *
 *     cutoff_volt_mv        : 3400 -> 3250   (dt field + SRAM CUTOFF_VOLT)
 *     empty_volt_mv         : 3100 -> 3000   (dt field + SRAM VBATT_LOW)
 *     SHUTDOWN_DELAY_VOL eff : 3300 -> 3150   (functional hook, see below)
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
 *   - Low-temperature tier: cold raises cell impedance, so the loaded terminal
 *     voltage sags well below OCV and holding the warm floor throws away real
 *     charge. Below lt_enter_dc all three floors drop by lt_delta_mv (with
 *     hysteresis, which stock lacks). Works whether or not the driver runs its
 *     own qcom,cutoff-voltage-adjust-enable path; where it does, we simply win
 *     the race because we re-assert on every voltage read.
 *   - Stock values are restored on unload.
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
KPM_VERSION("1.5.0");
KPM_LICENSE("GPL v2");
KPM_AUTHOR("battery-fg-fix-kpm");
KPM_DESCRIPTION("Lower FG-Gen4 cutoff + empty + shutdown floor (lmi / Si-C tier)");

/* ------------------------------------------------------------------ */
/* Tunables (conservative tier)                                        */
/* ------------------------------------------------------------------ */

#define DEFAULT_CUTOFF_MV   3250
#define CUTOFF_MV_MIN       2800
#define CUTOFF_MV_MAX       3400

#define DEFAULT_EMPTY_MV    3000
#define EMPTY_MV_MIN        2500
#define EMPTY_MV_MAX        3300

#define DEFAULT_SHUTDOWN_MV 3150   /* effective shutdown floor (hang-at-1% end) */
#define SHUTDOWN_MV_MIN     2800
#define SHUTDOWN_MV_MAX     3300

/*
 * Low-temperature tier.
 *
 * Cold raises cell impedance, so the LOADED terminal voltage sags far below
 * OCV. Holding the same floor in cold therefore throws away real charge. Stock
 * handles this by shifting its whole set down 200 mV below 15.0 C:
 *   cutoff 3400->3200, SHUTDOWN_DELAY_VOL 3300->3100, FVSS entry 3600->3400.
 *
 * We do the same, but by a SMALLER default step. The binding constraint in the
 * cold is not cell damage -- at these terminal voltages OCV is much higher, so
 * true depth-of-discharge stays modest -- it is system brown-out, because the
 * same impedance that justifies going lower also makes load spikes sag harder.
 * Headroom over sys_min_volt_mv (2800 on lmi) at the shutdown floor:
 *     stock normal 500 mV | stock cold 300 mV | ours normal 350 mV
 *     ours cold at -100 => 250 mV   (default)
 *     ours cold at -150 => 200 mV   (aggressive)
 *     ours cold at -200 => 150 mV   (below what the OEM considered safe; avoid)
 *
 * Applied uniformly to all three floors so the cutoff->shutdown "hang at 1%"
 * window keeps its width. Tunable at load time with "lt=<mv>" / "lttemp=<dC>".
 */
#define DEFAULT_LT_DELTA_MV 100    /* how far the whole set drops when cold   */
#define LT_DELTA_MV_MAX     200
#define DEFAULT_LT_ENTER_DC 150    /* deci-degrees C: cold below 15.0 C       */
#define LT_HYST_DC          20     /* leave cold above enter + 2.0 C          */

/*
 * POWER_SUPPLY_PROP_CAPACITY ordinal. Derived at runtime by name from the
 * kernel's power_supply_attrs[] table (which is indexed by the property enum),
 * so it is NOT tied to one tree's include/linux/power_supply.h. This value is
 * only the fallback if that derivation fails; it can also be forced with the
 * "psp=<n>" module argument.
 */
#define PSP_CAPACITY_FALLBACK 44

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
typedef int  (*fg_get_batt_temp_t)(void *fg, int *val_dc);

static probe_kernel_read_t p_probe_kernel_read;
static fg_sram_write_t     p_fg_sram_write;
static fg_sram_read_t      p_fg_sram_read;          /* optional: anchor source */
static void               *p_fg_get_battery_voltage; /* hook target + callable */
static void               *p_fg_psy_get_property;    /* hook target            */
static fg_get_batt_temp_t  p_fg_get_battery_temp;   /* optional: low-temp tier */

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

static int   target_cutoff_mv   = DEFAULT_CUTOFF_MV;
static int   target_empty_mv    = DEFAULT_EMPTY_MV;
static int   target_shutdown_mv = DEFAULT_SHUTDOWN_MV;
static int   lt_delta_mv    = DEFAULT_LT_DELTA_MV;  /* cold-tier step, mV      */
static int   lt_enter_dc    = DEFAULT_LT_ENTER_DC;  /* cold below this, deci-C */
static int   in_low_temp;                           /* current tier            */
static int   lt_poll_countdown;                     /* throttle temp reads     */
static int   orig_cutoff_mv     = -1;  /* stock values, captured on first apply */
static int   orig_empty_mv      = -1;
static int  *cutoff_field;             /* kernel VA of dt.cutoff_volt_mv         */
static int  *empty_field;              /* kernel VA of dt.empty_volt_mv (= +1)   */
static void *chip_ptr;                 /* cached chip == fg base; "located" gate */
static int   psp_capacity  = -1;       /* resolved at init (name lookup / arg)   */
static int   psp_from_arg  = -1;       /* "psp=<n>" override, -1 = unset         */

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

/*
 * Re-evaluate the cold tier. Returns 1 if the tier changed.
 *
 * Throttled: fg_gen4_get_battery_temp() does an SRAM read, and this runs off a
 * hot hook. Battery temperature has large thermal mass, so sampling every
 * LT_POLL_CALLS voltage reads is far more resolution than the effect needs.
 *
 * Hysteresis (LT_HYST_DC) is ours, not the driver's: stock flips at exactly
 * 15.0 C on a 10 s timer, so a battery sitting at the threshold makes it
 * oscillate. We would follow that oscillation with an SRAM rewrite each time.
 */
#define LT_POLL_CALLS 16

static int update_low_temp_tier(void *chip, int force)
{
    int temp_dc = 0, was = in_low_temp;

    if (!p_fg_get_battery_temp)
        return 0;

    if (!force && --lt_poll_countdown > 0)
        return 0;
    lt_poll_countdown = LT_POLL_CALLS;

    if (p_fg_get_battery_temp(chip, &temp_dc) != 0)
        return 0;

    if (in_low_temp)
        in_low_temp = (temp_dc < lt_enter_dc + LT_HYST_DC);
    else
        in_low_temp = (temp_dc < lt_enter_dc);

    if (in_low_temp != was) {
        pr_info(LOG_PREFIX "temp %d.%d C -> %s tier: cutoff=%d empty=%d shutdown=%d mV\n",
                temp_dc / 10, temp_dc < 0 ? -(temp_dc % 10) : temp_dc % 10,
                in_low_temp ? "COLD" : "normal",
                eff_cutoff_mv(), eff_empty_mv(), eff_shutdown_mv());
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Hook 1: fg_get_battery_voltage(struct fg_dev *fg, int *val)        */
/* arg0 == fg == chip base. Locates + applies cutoff/empty.           */
/* ------------------------------------------------------------------ */

static void before_get_voltage(hook_fargs2_t *args, void *udata)
{
    void *chip = (void *)args->arg0;
    int clobbered = 0, tier_changed = 0;

    if (!chip)
        return;

    if (chip != chip_ptr || !cutoff_field) {
        update_low_temp_tier(chip, 1);
        apply_from_chip(chip);
        return;
    }

    /* A tier change makes the stored values stale -- treat it as a clobber so
     * both the C fields and SRAM are rewritten below. */
    tier_changed = update_low_temp_tier(chip, 0);
    if (tier_changed)
        clobbered = 1;

    if (*cutoff_field != eff_cutoff_mv()) {
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
        if (!tier_changed && reassert_logs_left > 0) {
            reassert_logs_left--;
            pr_info(LOG_PREFIX "driver overwrote dt block; re-asserted C fields + SRAM%s\n",
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
/* KPM lifecycle                                                      */
/* ------------------------------------------------------------------ */

/*
 * Args: up to three bare decimal ints, positionally cutoff[,empty[,shutdown]]
 * ("3250", "3250,3000", "3250,3000,3150"), plus an optional "psp=<n>" token
 * that forces the POWER_SUPPLY_PROP_CAPACITY ordinal when the by-name lookup
 * cannot work on an unusual tree. Order and separators are free-form:
 *   "3250,3000,3150"      "3250,3000,3150,psp=44"      "psp=44"
 *
 * Low-temperature tier:
 *   "lt=<mv>"      how far all three floors drop when cold (0..200, default 100)
 *   "lttemp=<dC>"  cold threshold in deci-degrees C (default 150 = 15.0 C)
 *   e.g. "3250,3000,3150,lt=150"   "lt=0"  disables the cold tier entirely
 */
static int kw_is(const char *s, const char *kw)
{
    for (; *kw; s++, kw++) {
        char c = (*s >= 'A' && *s <= 'Z') ? (char)(*s + 32) : *s;

        if (c != *kw)
            return 0;
    }
    return 1;
}

static void set_targets_from_args(const char *args)
{
    const char *s = args;
    int vals[3];
    int idx = 0, v = 0, in = 0;

    for (; s && *s; s++) {
        /* "lttemp=<deci-C>" must be tested before the "lt=" prefix. */
        if (kw_is(s, "lttemp=")) {
            int v = 0, got = 0, neg = 0;

            s += 7;
            if (*s == '-') { neg = 1; s++; }
            for (; *s >= '0' && *s <= '9'; s++) { v = v * 10 + (*s - '0'); got = 1; }
            if (got)
                lt_enter_dc = clamp_i(neg ? -v : v, -200, 300);
            if (!*s)
                break;
            continue;
        }
        if (kw_is(s, "lt=")) {
            int v = 0, got = 0;

            s += 3;
            for (; *s >= '0' && *s <= '9'; s++) { v = v * 10 + (*s - '0'); got = 1; }
            if (got)
                lt_delta_mv = clamp_i(v, 0, LT_DELTA_MV_MAX);
            if (!*s)
                break;
            continue;
        }
        if ((s[0] == 'p' || s[0] == 'P') && (s[1] == 's' || s[1] == 'S') &&
            (s[2] == 'p' || s[2] == 'P') && s[3] == '=') {
            int pv = 0, got = 0;

            for (s += 4; *s >= '0' && *s <= '9'; s++) {
                pv = pv * 10 + (*s - '0');
                got = 1;
            }
            if (got)
                psp_from_arg = pv;
            if (!*s)
                break;
            continue;               /* *s is a separator; loop's s++ skips it */
        }

        if (*s >= '0' && *s <= '9') {
            v = v * 10 + (*s - '0');
            in = 1;
        } else if (in) {
            if (idx < 3)
                vals[idx++] = v;
            v = 0;
            in = 0;
        }
    }
    if (in && idx < 3)
        vals[idx++] = v;

    if (idx >= 1) target_cutoff_mv   = clamp_i(vals[0], CUTOFF_MV_MIN,   CUTOFF_MV_MAX);
    if (idx >= 2) target_empty_mv    = clamp_i(vals[1], EMPTY_MV_MIN,    EMPTY_MV_MAX);
    if (idx >= 3) target_shutdown_mv = clamp_i(vals[2], SHUTDOWN_MV_MIN, SHUTDOWN_MV_MAX);
}

static long fg_cutoff_init(const char *args, const char *event, void *__user reserved)
{
    hook_err_t err;

    set_targets_from_args(args);

    pr_info(LOG_PREFIX "init (event=%s, args=%s) cutoff=%d empty=%d shutdown=%d mV\n",
            event ? event : "?", args ? args : "", target_cutoff_mv,
            target_empty_mv, target_shutdown_mv);

    p_probe_kernel_read = (probe_kernel_read_t)kallsyms_lookup_name("probe_kernel_read");
    if (!p_probe_kernel_read)
        p_probe_kernel_read = (probe_kernel_read_t)kallsyms_lookup_name("__probe_kernel_read");
    p_fg_sram_write          = (fg_sram_write_t)kallsyms_lookup_name("fg_sram_write");
    p_fg_sram_read           = (fg_sram_read_t)kallsyms_lookup_name("fg_sram_read");
    p_fg_get_battery_voltage = (void *)kallsyms_lookup_name("fg_get_battery_voltage");
    p_fg_psy_get_property    = (void *)kallsyms_lookup_name("fg_psy_get_property");
    p_fg_get_battery_temp    = (fg_get_batt_temp_t)kallsyms_lookup_name("fg_gen4_get_battery_temp");

    pr_info(LOG_PREFIX "syms: pkr=%px sram_write=%px sram_read=%px get_voltage=%px get_property=%px\n",
            p_probe_kernel_read, p_fg_sram_write, p_fg_sram_read,
            p_fg_get_battery_voltage, p_fg_psy_get_property);

    if (!p_probe_kernel_read || !p_fg_get_battery_voltage) {
        pr_err(LOG_PREFIX "missing required symbols; aborting\n");
        return -1;
    }
    if (!p_fg_sram_write)
        pr_info(LOG_PREFIX "fg_sram_write not found; C-field only\n");
    if (!p_fg_sram_read)
        pr_info(LOG_PREFIX "fg_sram_read not found; dt scan falls back to shape+uniqueness\n");
    if (!p_fg_get_battery_temp)
        pr_info(LOG_PREFIX "fg_gen4_get_battery_temp not found; low-temp tier disabled\n");
    else
        pr_info(LOG_PREFIX "low-temp tier: -%d mV below %d.%d C (hyst %d.%d C)\n",
                lt_delta_mv, lt_enter_dc / 10, lt_enter_dc % 10,
                LT_HYST_DC / 10, LT_HYST_DC % 10);

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

    pr_info(LOG_PREFIX "hooks installed; applying on next FG voltage read\n");
    return 0;
}

/* Runtime reconfigure: kpm control battery-fg-cutoff "<cutoff>[,<empty>[,<shutdown>]]" */
static long fg_cutoff_control0(const char *args, char *__user out_msg, int outlen)
{
    char msg[160];
    int len;

    set_targets_from_args(args);

    if (cutoff_field && chip_ptr) {
        *cutoff_field = eff_cutoff_mv();
        if (empty_field)
            *empty_field = eff_empty_mv();
        write_cutoff_sram(chip_ptr, eff_cutoff_mv(), 1);
        write_empty_sram(chip_ptr, eff_empty_mv(), 1);
        len = fmt_msg(msg, (int)sizeof(msg),
                      "cutoff=%d empty=%d shutdown=%d mV live (stock %d/%d)\n",
                      eff_cutoff_mv(), eff_empty_mv(), eff_shutdown_mv(),
                      orig_cutoff_mv, orig_empty_mv);
    } else {
        len = fmt_msg(msg, (int)sizeof(msg),
                      "cutoff=%d empty=%d shutdown=%d mV queued (not yet located)\n",
                      eff_cutoff_mv(), eff_empty_mv(), eff_shutdown_mv());
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
