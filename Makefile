# Makefile for battery-fg-cutoff.kpm
#
# Two toolchain modes:
#
#   Clang (Android / NDK LLVM) — verified with clang-r487747c (clang 17):
#     make CLANG=$HOME/clang-android/clang-r487747c/bin \
#          KP_DIR=/path/to/KernelPatch
#     (TARGET defaults to aarch64-linux-gnu)
#
#   GCC (bare-metal aarch64 cross toolchain):
#     make TARGET_COMPILE=aarch64-linux-gnu- KP_DIR=/path/to/KernelPatch
#
# KP_DIR must point at a KernelPatch source checkout (for its headers);
# build.sh auto-clones it if absent.
#
# Output: fg_cutoff.kpm  (an aarch64 relocatable ELF the KernelPatch loader maps)

TARGET ?= aarch64-linux-gnu

ifdef CLANG
    CC          := $(CLANG)/clang --target=$(TARGET)
    LINK_FLAGS  := -fuse-ld=lld -nostdlib
    CC_EXTRA    := -ffreestanding
else ifdef TARGET_COMPILE
    CC          := $(TARGET_COMPILE)gcc
    LINK_FLAGS  :=
    CC_EXTRA    :=
else ifneq ($(filter-out clean,$(if $(MAKECMDGOALS),$(MAKECMDGOALS),all)),)
    $(error set CLANG=<clang bin dir> [TARGET=triple], or TARGET_COMPILE=aarch64-linux-gnu-)
endif

ifndef KP_DIR
    KP_DIR := KernelPatch
endif

INCLUDE_DIRS := . include patch/include linux/include \
                linux/arch/arm64/include linux/tools/arch/arm64/include
INCLUDE_FLAGS := $(foreach dir,$(INCLUDE_DIRS),-I$(KP_DIR)/kernel/$(dir))

# -mgeneral-regs-only is REQUIRED, not an optimisation choice. A KPM runs in
# kernel context (inside hooks on kernel functions), and arm64 Linux does not
# save/restore FP/SIMD state on kernel entry -- that is why the kernel itself
# builds with this flag. Without it clang vectorises ordinary integer code into
# NEON, which (a) can corrupt another task's FPSIMD state and (b) emits 16-byte
# aligned .rodata.cst16 constants reached via R_AARCH64_LDST128_ABS_LO12_NC.
# kpimg allocates modules from an 8-byte-aligned heap, so that relocation's
# imm12 (scaled by 16) silently drops the low bits and the load reads the wrong
# address -- a silent wrong-data bug, not a load failure.
CFLAGS ?= -Wall -O2 -fno-stack-protector -fno-pic -mgeneral-regs-only

objs := fg_cutoff.o

all: fg_cutoff.kpm

fg_cutoff.kpm: $(objs)
	$(CC) -r $(LINK_FLAGS) -o $@ $^
	@echo "built: $@"

%.o: %.c
	$(CC) $(CFLAGS) $(CC_EXTRA) $(INCLUDE_FLAGS) -c -o $@ $<

.PHONY: clean
clean:
	rm -f *.kpm *.o
