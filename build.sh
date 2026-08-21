#!/usr/bin/env bash
# Convenience build wrapper for fg_cutoff.kpm.
#
# Clang (auto-detected if CLANG points at an LLVM bin dir):
#   CLANG=$HOME/clang-android/clang-r487747c/bin ./build.sh
#
# GCC:
#   TARGET_COMPILE=aarch64-linux-gnu- ./build.sh
#
# KernelPatch headers are cloned next to this script if KP_DIR is unset. The
# default remote is SukiSU-Ultra's fork, because that is the kpimg that will
# actually load this module: it trails upstream (0.13.2 vs 0.13.5) and lacks
# some upstream-only KPM APIs, e.g. KPM_EVENT / .kpm.event. Building against
# upstream headers can therefore produce a module SukiSU rejects.
#
#   KP_REPO=https://github.com/bmax121/KernelPatch ./build.sh   # upstream

set -euo pipefail
cd "$(dirname "$0")"

KP_DIR="${KP_DIR:-KernelPatch}"
KP_REPO="${KP_REPO:-https://github.com/SukiSU-Ultra/SukiSU_KernelPatch_patch}"
KP_REF="${KP_REF:-main}"
TARGET="${TARGET:-aarch64-linux-gnu}"

if [ ! -d "$KP_DIR/kernel/include" ]; then
    echo ">> fetching KernelPatch headers into $KP_DIR ($KP_REPO $KP_REF)"
    if [ ! -d "$KP_DIR/.git" ]; then
        git clone --depth 1 -b "$KP_REF" "$KP_REPO" "$KP_DIR"
    fi
elif [ -d "$KP_DIR/.git" ]; then
    have="$(git -C "$KP_DIR" remote get-url origin 2>/dev/null || echo '?')"
    case "$have" in
        "$KP_REPO"|"$KP_REPO".git) ;;
        *) echo "!! $KP_DIR tracks $have, not $KP_REPO — delete it to re-clone" >&2 ;;
    esac
fi

if [ -n "${CLANG:-}" ]; then
    echo ">> building with CLANG=$CLANG TARGET=$TARGET KP_DIR=$KP_DIR"
    make CLANG="$CLANG" TARGET="$TARGET" KP_DIR="$KP_DIR" "$@"
elif [ -n "${TARGET_COMPILE:-}" ]; then
    echo ">> building with TARGET_COMPILE=$TARGET_COMPILE KP_DIR=$KP_DIR"
    make TARGET_COMPILE="$TARGET_COMPILE" KP_DIR="$KP_DIR" "$@"
else
    echo "!! set CLANG=<clang bin dir> or TARGET_COMPILE=aarch64-linux-gnu-" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Undefined-symbol check.
#
# kpimg resolves a KPM's undefined symbols ONLY against its own
# KP_EXPORT_SYMBOL table — the kallsyms fallback in simplify_symbols() is
# commented out ("kernel symbol cause overflow in relocation") both upstream and
# in SukiSU's fork. Anything else (snprintf, memcpy, ...) makes the loader bail
# out with "unknown symbol: <name>" and the manager just reports a failed load.
# ---------------------------------------------------------------------------
if [ -f fg_cutoff.kpm ]; then
    readelf=
    for cand in "${CLANG:-}/llvm-readelf" llvm-readelf "${TARGET_COMPILE:-}readelf" \
                "$TARGET-readelf" readelf; do
        [ -n "$cand" ] && command -v "$cand" >/dev/null 2>&1 && { readelf=$cand; break; }
    done

    if [ -z "$readelf" ]; then
        echo ">> no readelf found, skipping undefined-symbol check"
    else
        exported="$(grep -rho 'KP_EXPORT_SYMBOL([A-Za-z0-9_]*)' "$KP_DIR/kernel" \
                    | sed 's/^KP_EXPORT_SYMBOL(//; s/)$//' | sort -u)"
        missing=
        while read -r sym; do
            [ -n "$sym" ] || continue
            grep -qx "$sym" <<<"$exported" || missing="$missing $sym"
        done < <("$readelf" -sW fg_cutoff.kpm | awk '$7 == "UND" && $8 != "" { print $8 }' | sort -u)

        if [ -n "$missing" ]; then
            echo
            echo "!! these symbols are not exported by $KP_DIR — the loader will reject the module:" >&2
            for sym in $missing; do echo "!!   $sym" >&2; done
            echo "!! implement them in-module or use an exported equivalent" >&2
            exit 1
        fi
        echo ">> undefined symbols all resolvable by kpimg"

        # -------------------------------------------------------------------
        # kpimg export-surface baseline.
        #
        # Checking against $KP_DIR is NOT sufficient: it validates against the
        # headers we happen to build with, while the phone runs whatever kpimg
        # its boot image was patched with -- often older. kpimg's export table
        # has GROWN over releases (hotpatch, for one, only became a
        # KP_EXPORT_SYMBOL in 0.13.0), and referencing a symbol the installed
        # kpimg lacks makes the module fail to LOAD AT ALL, before init runs --
        # the same silent failure as an unexported libc function.
        #
        # So pin the surface: these five are known to resolve as far back as
        # kpimg 0.12.0. Anything new must be justified against the OLDEST kpimg
        # you intend to support, then added here.
        # -------------------------------------------------------------------
        baseline="${KP_UND_BASELINE:-compat_copy_to_user hook_wrap kallsyms_lookup_name printk unhook}"
        newsyms=
        while read -r sym; do
            [ -n "$sym" ] || continue
            case " $baseline " in
                *" $sym "*) ;;
                *) newsyms="$newsyms $sym" ;;
            esac
        done < <("$readelf" -sW fg_cutoff.kpm | awk '$7 == "UND" && $8 != "" { print $8 }' | sort -u)

        if [ -n "$newsyms" ]; then
            echo >&2
            echo "!! new kpimg symbol dependency beyond the verified baseline:" >&2
            for sym in $newsyms; do echo "!!   $sym" >&2; done
            echo "!! \$KP_DIR exports it, but an older installed kpimg may not --" >&2
            echo "!! that makes the module fail to load with no init output at all." >&2
            echo "!! Prefer resolving kernel functions via kallsyms_lookup_name;" >&2
            echo "!! if the dependency is deliberate, add it to KP_UND_BASELINE." >&2
            exit 1
        fi
        echo ">> kpimg symbol surface matches the verified baseline"

        # -------------------------------------------------------------------
        # Alignment / relocation check.
        #
        # kpimg allocates a module with kp_malloc_exec(), which only guarantees
        # 8-byte alignment (that is why .text lands at <base>+8). Section offsets
        # are aligned relative to that base, so any section needing >8 bytes of
        # alignment ends up misaligned in absolute terms. When such a section is
        # reached via R_AARCH64_LDST128_ABS_LO12_NC -- whose imm12 is scaled by
        # 16 -- the low bits are silently DROPPED and the load reads the wrong
        # address. It does not fail to load; it reads garbage at runtime.
        #
        # Both are avoided by -mgeneral-regs-only (see Makefile), which a KPM
        # needs anyway: kernel context has no saved FP/SIMD state.
        # -------------------------------------------------------------------
        badalign="$("$readelf" -SW fg_cutoff.kpm | sed 's/\[ *[0-9]*\]//' \
                    | awk '/PROGBITS|NOBITS/ && $NF+0 > 8 { print $1 " (align " $NF ")" }')"
        badrelo="$("$readelf" -rW fg_cutoff.kpm | grep -c LDST128_ABS_LO12_NC || true)"

        if [ -n "$badalign" ] || [ "$badrelo" != "0" ]; then
            echo >&2
            echo "!! module needs alignment kpimg does not provide (8-byte heap):" >&2
            [ -n "$badalign" ] && echo "$badalign" | sed 's/^/!!   over-aligned section: /' >&2
            [ "$badrelo" != "0" ] && echo "!!   $badrelo x R_AARCH64_LDST128_ABS_LO12_NC (imm12 scaled by 16)" >&2
            echo "!! these relocate to a WRONG address silently -- build with -mgeneral-regs-only" >&2
            exit 1
        fi
        echo ">> no over-aligned sections or LDST128 relocations"
    fi
fi

echo
echo ">> done: $(ls -l fg_cutoff.kpm 2>/dev/null || echo 'build failed')"
