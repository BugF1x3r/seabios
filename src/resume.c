// Code for handling calls to "post" that are resume related.
//
// Copyright (C) 2008,2009  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "bregs.h" // struct bregs
#include "config.h" // CONFIG_*
#include "farptr.h" // FLATPTR_TO_SEGOFF
#include "hw/pci.h" // pci_reboot
#include "hw/pic.h" // pic_eoi2
#include "hw/ps2port.h" // i8042_reboot
#include "hw/rtc.h" // rtc_read
#include "output.h" // dprintf
#include "stacks.h" // farcall16big
#include "std/bda.h" // struct bios_data_area_s
#include "string.h" // memset
#include "util.h" // dma_setup
#include "tcgbios.h" // tpm_s3_resume
#include "fw/romfile_loader.h" // romfile_fw_cfg_resume

// Handler for post calls that look like a resume.
void VISIBLE16
handle_resume(void)
{
    ASSERT16();
    int status = rtc_read(CMOS_RESET_CODE);
    rtc_write(CMOS_RESET_CODE, 0);
    dprintf(1, "In resume (status=%d)\n", status);

    dma_setup();

    switch (status) {
    case 0x01 ... 0x04:
    case 0x06 ... 0x09:
        panic("Unimplemented shutdown status: %02x\n", status);

    case 0x05:
        // flush keyboard (issue EOI) and jump via 40h:0067h
        pic_eoi2();
        // NO BREAK
    case 0x0a:
#define BDA_JUMP (((struct bios_data_area_s *)0)->jump)
        // resume execution by jump via 40h:0067h
        asm volatile(
            "movw %w1, %%ds\n"
            "ljmpw *%0\n"
            : : "m"(BDA_JUMP), "r"(SEG_BDA)
            );
        break;

    case 0x0b:
        // resume execution via IRET via 40h:0067h
        asm volatile(
            "movw %w1, %%ds\n"
            "lssw %0, %%sp\n"
            "iretw\n"
            : : "m"(BDA_JUMP), "r"(SEG_BDA)
            );
        break;

    case 0x0c:
        // resume execution via RETF via 40h:0067h
        asm volatile(
            "movw %w1, %%ds\n"
            "lssw %0, %%sp\n"
            "lretw\n"
            : : "m"(BDA_JUMP), "r"(SEG_BDA)
            );
        break;

    default:
        break;
    }

    // Not a 16bit resume - do remaining checks in 32bit mode
    asm volatile(
        "movw %w1, %%ss\n"
        "movl %0, %%esp\n"
        "movl $_cfunc32flat_handle_resume32, %%edx\n"
        "jmp transition32\n"
        : : "i"(BUILD_S3RESUME_STACK_ADDR), "r"(0), "a"(status)
        );
}

// Handle an S3 resume event
static void
s3_resume(void)
{
    if (!CONFIG_S3_RESUME)
        return;

    u32 s3_resume_vector = find_resume_vector();
    if (!s3_resume_vector) {
        dprintf(1, "No resume vector set!\n");
        return;
    }

    pic_setup();
    smm_setup();
    smp_resume();

    pci_resume();

    /* resume TPM before we may measure option roms */
    tpm_s3_resume();
    s3_resume_vga();

    /* Replay any fw_cfg entries that go back to the host */
    romfile_fw_cfg_resume();

    make_bios_readonly();

    // Invoke the resume vector.
    struct bregs br;
    memset(&br, 0, sizeof(br));
    dprintf(1, "Jump to resume vector (%x)\n", s3_resume_vector);
    br.code = FLATPTR_TO_SEGOFF((void*)s3_resume_vector);
    farcall16big(&br);
}

// Xbox: a boot failure ends in the chip OS menu, not in another go at the
// same bank. The SMC's power cycle (register 0x02 <- 0x40) drops the main
// rails, which resets the modchip's bank register, so the console comes
// back the way a power-on does: the chip OS shows its menu and the next
// test run can start with nobody at the console. A CF9 reset or the SMC's
// plain reset (0x01) would restart XBIOS on the same bank.
//
// The byte write must complete on the wire before anything resets the
// MCPX: the first version of this hook started the transaction and fell
// through to CF9 at once, and the 1.4 went dark. So: wait for the
// controller (MCPX SMBus at I/O 0xC000, as the init stage programs it),
// start the write, wait for its DONE or error status, then give the SMC
// a few seconds to act before the generic methods get their turn.
#define XBOX_SMBUS_IO           0xc000
#define XBOX_SMBUS_BUSY         0x0800
#define XBOX_SMBUS_DONE         0x10
#define XBOX_SMBUS_ERROR        0x24
#define XBOX_SMBUS_ANY          0x36
#define XBOX_SMC_ADDR           0x10
#define XBOX_SMC_REG_RESET      0x02
#define XBOX_SMC_POWER_CYCLE    0x40    // 0x01 reset, 0x80 power off

static void
xbox_smc_reboot(void)
{
    if (!CONFIG_XBOX)
        return;
    dprintf(1, "xbox: asking the SMC for a power cycle\n");
    u32 end = timer_calc(100);
    while ((inw(XBOX_SMBUS_IO + 0) & XBOX_SMBUS_BUSY) && !timer_check(end))
        ;
    outw(inw(XBOX_SMBUS_IO + 0), XBOX_SMBUS_IO + 0);    // clear stale status
    outb((XBOX_SMC_ADDR << 1) | 0, XBOX_SMBUS_IO + 4);  // SMC, write
    outb(XBOX_SMC_REG_RESET, XBOX_SMBUS_IO + 8);
    outw(XBOX_SMC_POWER_CYCLE, XBOX_SMBUS_IO + 6);
    outb(0x1a, XBOX_SMBUS_IO + 2);                      // start a byte write
    u8 status = 0;
    end = timer_calc(100);
    while (!((status = inb(XBOX_SMBUS_IO + 0)) & XBOX_SMBUS_ANY)
           && !timer_check(end))
        ;
    int ok = (status & XBOX_SMBUS_DONE) && !(status & XBOX_SMBUS_ERROR);
    dprintf(1, "xbox: SMC power cycle %s (status %02x)\n"
            , ok ? "requested" : "write failed", status);
    // The SMC acts well within this; still running afterwards means it
    // did not, and the generic methods below get their turn.
    end = timer_calc(5000);
    while (!timer_check(end))
        ;
    dprintf(1, "xbox: still running, the SMC did not act\n");
}

// Attempt to invoke a hard-reboot.
static void
tryReboot(void)
{
    dprintf(1, "Attempting a hard reboot\n");

    // Xbox: the SMC does it
    xbox_smc_reboot();

    // Use a QEMU specific reboot on QEMU
    qemu_reboot();

    // Reboot using ACPI RESET_REG
    acpi_reboot();

    // Try keyboard controller reboot.
    i8042_reboot();

    // Try PCI 0xcf9 reboot
    pci_reboot();

    // Try triple fault
    asm volatile("int3");

    panic("Could not reboot");
}

void VISIBLE32FLAT
handle_resume32(int status)
{
    ASSERT32FLAT();
    dprintf(1, "In 32bit resume\n");

    if (status == 0xfe)
        s3_resume();

    // Must be a soft reboot - invoke a hard reboot.
    tryReboot();
}
