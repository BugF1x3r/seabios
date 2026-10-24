// Low-level serial (and serial-like) device access.
//
// Copyright (C) 2008-1013  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "config.h" // CONFIG_DEBUG_SERIAL
#include "fw/paravirt.h" // RunningOnQEMU
#include "output.h" // dprintf
#include "serialio.h" // serial_debug_preinit
#include "x86.h" // outb


/****************************************************************
 * Serial port debug output
 ****************************************************************/

#define DEBUG_TIMEOUT 100000

// Zero when CONFIG_DEBUG_SERIAL_PROBE found no UART (serial_debug_detect).
u8 SerialDebugPresent VARFSEG = 1;

// Write to a serial port register
static void
serial_debug_write(u8 offset, u8 val)
{
    if (CONFIG_DEBUG_SERIAL) {
        outb(val, CONFIG_DEBUG_SERIAL_PORT + offset);
    } else if (CONFIG_DEBUG_SERIAL_MMIO) {
        ASSERT32FLAT();
        writeb((void*)CONFIG_DEBUG_SERIAL_MEM_ADDRESS + 4*offset, val);
    }
}

// Read from a serial port register
static u8
serial_debug_read(u8 offset)
{
    if (CONFIG_DEBUG_SERIAL)
        return inb(CONFIG_DEBUG_SERIAL_PORT + offset);
    if (CONFIG_DEBUG_SERIAL_MMIO) {
        ASSERT32FLAT();
        return readb((void*)CONFIG_DEBUG_SERIAL_MEM_ADDRESS + 4*offset);
    }
}

// Setup the debug serial port for output.
void
serial_debug_preinit(void)
{
    if (!CONFIG_DEBUG_SERIAL && (!CONFIG_DEBUG_SERIAL_MMIO || MODESEGMENT))
        return;
    // setup for serial logging: 8N1
    u8 oldparam, newparam = 0x03;
    oldparam = serial_debug_read(SEROFF_LCR);
    serial_debug_write(SEROFF_LCR, newparam);
    // Disable irqs
    u8 oldier, newier = 0;
    oldier = serial_debug_read(SEROFF_IER);
    serial_debug_write(SEROFF_IER, newier);

    if (oldparam != newparam || oldier != newier)
        dprintf(1, "Changing serial settings was %x/%x now %x/%x\n"
                , oldparam, oldier, newparam, newier);
}

// Probe the UART's scratch register; silence serial output if absent.
void
serial_debug_detect(void)
{
    if (!CONFIG_DEBUG_SERIAL || !CONFIG_DEBUG_SERIAL_PROBE)
        return;
    serial_debug_write(SEROFF_SCR, 0x5a);
    u8 a = serial_debug_read(SEROFF_SCR);
    serial_debug_write(SEROFF_SCR, 0xa5);
    u8 b = serial_debug_read(SEROFF_SCR);
    if (a == 0x5a && b == 0xa5)
        return;
    SerialDebugPresent = 0;
    dprintf(1, "No UART at %x (scratch read %x %x): serial debug off\n"
            , CONFIG_DEBUG_SERIAL_PORT, a, b);
}

// Write a character to the serial port.
static void
serial_debug(char c)
{
    if (!CONFIG_DEBUG_SERIAL && (!CONFIG_DEBUG_SERIAL_MMIO || MODESEGMENT))
        return;
    if (CONFIG_DEBUG_SERIAL_PROBE && !GET_GLOBAL(SerialDebugPresent))
        return;
    int timeout = DEBUG_TIMEOUT;
    while ((serial_debug_read(SEROFF_LSR) & 0x20) != 0x20)
        if (!timeout--)
            // Ran out of time.
            return;
    serial_debug_write(SEROFF_DATA, c);
}

void
serial_debug_putc(char c)
{
    if (c == '\n')
        serial_debug('\r');
    serial_debug(c);
}

// Make sure all serial port writes have been completely sent.
void
serial_debug_flush(void)
{
    if (!CONFIG_DEBUG_SERIAL && (!CONFIG_DEBUG_SERIAL_MMIO || MODESEGMENT))
        return;
    if (CONFIG_DEBUG_SERIAL_PROBE && !GET_GLOBAL(SerialDebugPresent))
        return;
    int timeout = DEBUG_TIMEOUT;
    while ((serial_debug_read(SEROFF_LSR) & 0x60) != 0x60)
        if (!timeout--)
            // Ran out of time.
            return;
}


/****************************************************************
 * QEMU debug port
 ****************************************************************/

u16 DebugOutputPort VARFSEG = 0x402;

void
qemu_debug_preinit(void)
{
    /* Xen doesn't support checking if debug output is active. */
    if (runningOnXen())
        return;

    /* Check if the QEMU debug output port is active */
    if (CONFIG_DEBUG_IO &&
        inb(GET_GLOBAL(DebugOutputPort)) != QEMU_DEBUGCON_READBACK)
        DebugOutputPort = 0;
}

// Write a character to the special debugging port.
void
qemu_debug_putc(char c)
{
    if (!CONFIG_DEBUG_IO || !runningOnQEMU())
        return;
    u16 port = GET_GLOBAL(DebugOutputPort);
    if (port)
        // Send character to debug port.
        outb(c, port);
}
