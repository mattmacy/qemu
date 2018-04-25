/*
 *  ASPEED LPC Controller
 *
 *  Copyright (C) 2017-2018 IBM Corp.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/misc/aspeed_lpc.h"
#include "qapi/error.h"

static uint64_t aspeed_lpc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    AspeedLPCState *s = ASPEED_LPC(opaque);
    uint64_t val = 0;

    addr >>= 2;

    if (addr >= ASPEED_LPC_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr << 2);
    } else {
        val = s->regs[addr];
    }

    return val;
}

static void aspeed_lpc_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned int size)
{
    AspeedLPCState *s = ASPEED_LPC(opaque);

    addr >>= 2;

    if (addr >= ASPEED_LPC_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr << 2);
        return;
    }

    s->regs[addr] = data;
}

static const MemoryRegionOps aspeed_lpc_ops = {
    .read = aspeed_lpc_read,
    .write = aspeed_lpc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aspeed_lpc_reset(DeviceState *dev)
{
    struct AspeedLPCState *s = ASPEED_LPC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void aspeed_lpc_realize(DeviceState *dev, Error **errp)
{
    AspeedLPCState *s = ASPEED_LPC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_lpc_ops, s,
            TYPE_ASPEED_LPC, 0x1000);

    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_aspeed_lpc = {
    .name = TYPE_ASPEED_LPC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AspeedLPCState, ASPEED_LPC_NR_REGS),
        VMSTATE_END_OF_LIST(),
    }
};

static void aspeed_lpc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_lpc_realize;
    dc->reset = aspeed_lpc_reset;
    dc->desc = "Aspeed LPC Controller",
    dc->vmsd = &vmstate_aspeed_lpc;
}

static const TypeInfo aspeed_lpc_info = {
    .name = TYPE_ASPEED_LPC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedLPCState),
    .class_init = aspeed_lpc_class_init,
};

static void aspeed_lpc_register_types(void)
{
    type_register_static(&aspeed_lpc_info);
}

type_init(aspeed_lpc_register_types);
