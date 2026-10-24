// 32-bit exception reporter: an IDT for the 32-bit flat code that prints
// the vector, error code, registers and a stack sample on the debug console
// and halts, instead of the dummy IDT's silent machine shutdown.
//
// Copyright (C) 2026 Marco Janssen Groesbeek
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // VARFSEG
#include "config.h" // CONFIG_*
#include "output.h" // dprintf
#include "string.h" // memset
#include "util.h" // exc_setup
#include "x86.h" // struct descloc_s

extern struct descloc_s pmode_IDT_info;
static u64 ExcIDT[32] VARFSEG __aligned(8);
static struct descloc_s ExcDummy VARFSEG;

void VISIBLE32FLAT
exc_handle(u32 *f)
{
    // f: edi esi ebp esp ebx edx ecx eax | vec err eip cs eflags
    dprintf(1, "\nEXCEPTION %d err %x at %x:%x eflags %x\n"
            "  eax %x ebx %x ecx %x edx %x esi %x edi %x ebp %x esp %x\n"
            , f[8], f[9], f[11], f[10], f[12]
            , f[7], f[4], f[6], f[5], f[1], f[0], f[2], f[3]);
    u32 *sp = (u32*)(f + 13);
    dprintf(1, "  stack %x %x %x %x %x %x %x %x\n"
            , sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], sp[6], sp[7]);
    for (;;)
        asm volatile("hlt");
}

void __attribute__((naked, used)) exc_common(void)
{
    asm volatile("pushal\n movl %esp, %eax\n pushl %eax\n call exc_handle\n 1: hlt\n jmp 1b");
}
#define STUB_NOERR(n) void __attribute__((naked, used)) exc_##n(void) \
    { asm volatile("pushl $0\n pushl $" #n "\n jmp exc_common"); }
#define STUB_ERR(n) void __attribute__((naked, used)) exc_##n(void) \
    { asm volatile("pushl $" #n "\n jmp exc_common"); }
STUB_NOERR(0) STUB_NOERR(1) STUB_NOERR(2) STUB_NOERR(3) STUB_NOERR(4)
STUB_NOERR(5) STUB_NOERR(6) STUB_NOERR(7) STUB_ERR(8) STUB_NOERR(9)
STUB_ERR(10) STUB_ERR(11) STUB_ERR(12) STUB_ERR(13) STUB_ERR(14)
STUB_NOERR(15) STUB_NOERR(16) STUB_ERR(17) STUB_NOERR(18) STUB_NOERR(19)
STUB_NOERR(20) STUB_NOERR(21) STUB_NOERR(22) STUB_NOERR(23) STUB_NOERR(24)
STUB_NOERR(25) STUB_NOERR(26) STUB_NOERR(27) STUB_NOERR(28) STUB_NOERR(29)
STUB_NOERR(30) STUB_NOERR(31)

static void (*const ExcStubs[32])(void) = {
    exc_0, exc_1, exc_2, exc_3, exc_4, exc_5, exc_6, exc_7, exc_8, exc_9,
    exc_10, exc_11, exc_12, exc_13, exc_14, exc_15, exc_16, exc_17, exc_18,
    exc_19, exc_20, exc_21, exc_22, exc_23, exc_24, exc_25, exc_26, exc_27,
    exc_28, exc_29, exc_30, exc_31,
};

// Install the IDT (after init relocation; pmode_IDT_info keeps it across
// the 16-bit transitions).
void
exc_setup(void)
{
    if (!CONFIG_DEBUG_IDT)
        return;
    int i;
    for (i = 0; i < 32; i++) {
        u32 a = (u32)ExcStubs[i];
        ExcIDT[i] = ((u64)(a & 0xffff0000) << 32) | ((u64)0x8e00 << 32)
                    | ((u32)SEG32_MODE32_CS << 16) | (a & 0xffff);
    }
    ExcDummy = pmode_IDT_info;
    pmode_IDT_info.length = sizeof(ExcIDT) - 1;
    pmode_IDT_info.addr = (u32)ExcIDT;
    asm volatile("lidtl %0" : : "m"(pmode_IDT_info) : "memory");
    dprintf(1, "exc: 32-bit exception IDT installed at %x\n", (u32)ExcIDT);
}

// Back to the dummy IDT before the boot loader runs.
void
exc_prepboot(void)
{
    if (!CONFIG_DEBUG_IDT)
        return;
    pmode_IDT_info = ExcDummy;
    asm volatile("lidtl %0" : : "m"(pmode_IDT_info) : "memory");
}
