#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <asm/page.h>

#include "evrmemmap.h"

static u32 IntEvent[256] = {
    /* 0x00 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x10 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x20 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x30 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x40 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x50 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x60 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x70 */  1<<C_EVR_MAP_SECONDS_0, 1<<C_EVR_MAP_SECONDS_1, 
    /* 0x72 */  0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x7a */  1<<C_EVR_MAP_HEARTBEAT_EVENT, 1<<C_EVR_MAP_RESETPRESC_EVENT,
                1<<C_EVR_MAP_TIMESTAMP_CLK, 1<<C_EVR_MAP_TIMESTAMP_RESET,
    /* 0x7e */  0, 0,
    /* 0x80 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0x90 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0xa0 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0xb0 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0xc0 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0xd0 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0xe0 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* 0xf0 */  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

void ErInitializeRams(volatile struct MrfErRegs *pEr)
{
    u32 ctrl = be32_to_cpu(pEr->Control);
    int code;
    int i;

    /* Disable the ram, and select ram 0 */
    ctrl &= ~((1 << C_EVR_CTRL_MAP_RAM_ENABLE) | (1 << C_EVR_CTRL_MAP_RAM_SELECT));
    pEr->Control = be32_to_cpu(ctrl);

    for (i = 1; i > 0; i--) {
        for(code=0; code < EVR_MAX_EVENT_CODE; code++) {
            pEr->MapRam[i][code].IntEvent = be32_to_cpu(IntEvent[code]);
            pEr->MapRam[i][code].PulseSet = 0;
            pEr->MapRam[i][code].PulseClear = 0;
            pEr->MapRam[i][code].PulseTrigger = 0;
        }

        if (i) {
            /* Select ram 1, albeit still disabled.  Do we really need this? */
            ctrl |= (1 << C_EVR_CTRL_MAP_RAM_SELECT);
            pEr->Control = be32_to_cpu(ctrl);
        }
    }
}

void ErUpdateRam(volatile struct MrfErRegs *pEr, u16 *RamBuf)
{
    u32 ctrl = be32_to_cpu(pEr->Control);
    int newram;
    int code;
    u32 mask;

    if ((ctrl>>C_EVR_CTRL_MAP_RAM_SELECT) & 1)
        newram = 0;
    else
        newram = 1;

    for(code=0; code < EVR_MAX_EVENT_CODE; code++) {
        mask = IntEvent[code];
        if (RamBuf[code] & (1<<15))
            mask |= 1<<C_EVR_MAP_SAVE_EVENT;
        if (RamBuf[code] & (1<<14))
            mask |= 1<<C_EVR_MAP_LATCH_TIMESTAMP;
        pEr->MapRam[newram][code].IntEvent = be32_to_cpu(mask);
        pEr->MapRam[newram][code].PulseTrigger = be32_to_cpu(RamBuf[code] & 0x3fff);
    }

    ctrl &= ~((1 << C_EVR_CTRL_MAP_RAM_ENABLE) | (1 << C_EVR_CTRL_MAP_RAM_SELECT));
    ctrl |= (1 << C_EVR_CTRL_MAP_RAM_ENABLE);
    if (newram == 1)
        ctrl |= (1 << C_EVR_CTRL_MAP_RAM_SELECT);
    pEr->Control = be32_to_cpu(ctrl);
}
