/*
 * QEMU PowerPC PowerNV PHB3 model
 *
 * Copyright (c) 2014-2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "hw/ppc/fdt.h"
#include "hw/pci-host/pnv_phb3_regs.h"
#include "hw/pci-host/pnv_phb3.h"
#include "hw/ppc/pnv.h"
#include "hw/ppc/pnv_xscom.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"

#include <libfdt.h>

static uint64_t pnv_pbcq_nest_xscom_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    PnvPBCQState *pbcq = PNV_PBCQ(opaque);
    uint32_t offset = addr >> 3;

    return pbcq->nest_regs[offset];
}

static uint64_t pnv_pbcq_pci_xscom_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    PnvPBCQState *pbcq = PNV_PBCQ(opaque);
    uint32_t offset = addr >> 3;

    return pbcq->pci_regs[offset];
}

static uint64_t pnv_pbcq_spci_xscom_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    PnvPBCQState *pbcq = PNV_PBCQ(opaque);
    uint32_t offset = addr >> 3;

    if (offset == PBCQ_SPCI_ASB_DATA) {
        return pnv_phb3_reg_read(pbcq->phb,
                                 pbcq->spci_regs[PBCQ_SPCI_ASB_ADDR], 8);
    }
    return pbcq->spci_regs[offset];
}

static void pnv_pbcq_update_map(PnvPBCQState *pbcq)
{
    uint64_t bar_en = pbcq->nest_regs[PBCQ_NEST_BAR_EN];
    uint64_t bar, mask, size;

    /*
     * NOTE: This will really not work well if those are remapped
     * after the PHB has created its sub regions. We could do better
     * if we had a way to resize regions but we don't really care
     * that much in practice as the stuff below really only happens
     * once early during boot
     */

    /* Handle unmaps */
    if (pbcq->mmio0_mapped && !(bar_en & PBCQ_NEST_BAR_EN_MMIO0)) {
        memory_region_del_subregion(get_system_memory(), &pbcq->mmbar0);
        pbcq->mmio0_mapped = false;
    }
    if (pbcq->mmio1_mapped && !(bar_en & PBCQ_NEST_BAR_EN_MMIO1)) {
        memory_region_del_subregion(get_system_memory(), &pbcq->mmbar1);
        pbcq->mmio1_mapped = false;
    }
    if (pbcq->phb_mapped && !(bar_en & PBCQ_NEST_BAR_EN_PHB)) {
        memory_region_del_subregion(get_system_memory(), &pbcq->phbbar);
        pbcq->phb_mapped = false;
    }

    /* Update PHB */
    pnv_phb3_update_regions(pbcq->phb);

    /* Handle maps */
    if (!pbcq->mmio0_mapped && (bar_en & PBCQ_NEST_BAR_EN_MMIO0)) {
        bar = pbcq->nest_regs[PBCQ_NEST_MMIO_BAR0] >> 14;
        mask = pbcq->nest_regs[PBCQ_NEST_MMIO_MASK0];
        size = ((~mask) >> 14) + 1;
        memory_region_init(&pbcq->mmbar0, OBJECT(pbcq), "pbcq-mmio0", size);
        memory_region_add_subregion(get_system_memory(), bar, &pbcq->mmbar0);
        pbcq->mmio0_mapped = true;
        pbcq->mmio0_base = bar;
        pbcq->mmio0_size = size;
    }
    if (!pbcq->mmio1_mapped && (bar_en & PBCQ_NEST_BAR_EN_MMIO1)) {
        bar = pbcq->nest_regs[PBCQ_NEST_MMIO_BAR1] >> 14;
        mask = pbcq->nest_regs[PBCQ_NEST_MMIO_MASK1];
        size = ((~mask) >> 14) + 1;
        memory_region_init(&pbcq->mmbar1, OBJECT(pbcq), "pbcq-mmio1", size);
        memory_region_add_subregion(get_system_memory(), bar, &pbcq->mmbar1);
        pbcq->mmio1_mapped = true;
        pbcq->mmio1_base = bar;
        pbcq->mmio1_size = size;
    }
    if (!pbcq->phb_mapped && (bar_en & PBCQ_NEST_BAR_EN_PHB)) {
        bar = pbcq->nest_regs[PBCQ_NEST_PHB_BAR] >> 14;
        size = 0x1000;
        memory_region_init(&pbcq->phbbar, OBJECT(pbcq), "pbcq-phb", size);
        memory_region_add_subregion(get_system_memory(), bar, &pbcq->phbbar);
        pbcq->phb_mapped = true;
    }

    /* Update PHB */
    pnv_phb3_update_regions(pbcq->phb);
}

static void pnv_pbcq_nest_xscom_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    PnvPBCQState *pbcq = PNV_PBCQ(opaque);
    uint32_t reg = addr >> 3;

    switch (reg) {
    case PBCQ_NEST_MMIO_BAR0:
    case PBCQ_NEST_MMIO_BAR1:
    case PBCQ_NEST_MMIO_MASK0:
    case PBCQ_NEST_MMIO_MASK1:
        if (pbcq->nest_regs[PBCQ_NEST_BAR_EN] &
            (PBCQ_NEST_BAR_EN_MMIO0 |
             PBCQ_NEST_BAR_EN_MMIO1)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                           "PHB3: Changing enabled BAR unsupported\n");
        }
        pbcq->nest_regs[reg] = val & 0xffffffffc0000000ull;
        break;
    case PBCQ_NEST_PHB_BAR:
        if (pbcq->nest_regs[PBCQ_NEST_BAR_EN] & PBCQ_NEST_BAR_EN_PHB) {
            qemu_log_mask(LOG_GUEST_ERROR,
                           "PHB3: Changing enabled BAR unsupported\n");
        }
        pbcq->nest_regs[reg] = val & 0xfffffffffc000000ull;
        break;
    case PBCQ_NEST_BAR_EN:
        pbcq->nest_regs[reg] = val & 0xf800000000000000ull;
        pnv_pbcq_update_map(pbcq);
        pnv_phb3_remap_irqs(pbcq->phb);
        break;
    case PBCQ_NEST_IRSN_COMPARE:
    case PBCQ_NEST_IRSN_MASK:
        pbcq->nest_regs[reg] = val & PBCQ_NEST_IRSN_COMP_MASK;
        pnv_phb3_remap_irqs(pbcq->phb);
        break;
    case PBCQ_NEST_LSI_SRC_ID:
        pbcq->nest_regs[reg] = val & PBCQ_NEST_LSI_SRC_MASK;
        pnv_phb3_remap_irqs(pbcq->phb);
        break;
    }

    /* XXX Don't error out on other regs for now ... */
}

static void pnv_pbcq_pci_xscom_write(void *opaque, hwaddr addr,
                                     uint64_t val, unsigned size)
{
    PnvPBCQState *pbcq = PNV_PBCQ(opaque);
    uint32_t reg = addr >> 3;

    switch (reg) {
    case PBCQ_PCI_BAR2:
        pbcq->pci_regs[reg] = val & 0xfffffffffc000000ull;
        pnv_pbcq_update_map(pbcq);
        break;
    }

    /* XXX Don't error out on other regs for now ... */
}

static void pnv_pbcq_spci_xscom_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    PnvPBCQState *pbcq = PNV_PBCQ(opaque);
    uint32_t reg = addr >> 3;

    switch (reg) {
    case PBCQ_SPCI_ASB_ADDR:
        pbcq->spci_regs[reg] = val & 0xfff;
        break;
    case PBCQ_SPCI_ASB_STATUS:
        pbcq->spci_regs[reg] &= ~val;
        break;
    case PBCQ_SPCI_ASB_DATA:
        pnv_phb3_reg_write(pbcq->phb, pbcq->spci_regs[PBCQ_SPCI_ASB_ADDR],
                           val, 8);
        break;
    case PBCQ_SPCI_AIB_CAPP_EN:
    case PBCQ_SPCI_CAPP_SEC_TMR:
        break;
    }

    /* XXX Don't error out on other regs for now ... */
}

static const MemoryRegionOps pnv_pbcq_nest_xscom_ops = {
    .read = pnv_pbcq_nest_xscom_read,
    .write = pnv_pbcq_nest_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const MemoryRegionOps pnv_pbcq_pci_xscom_ops = {
    .read = pnv_pbcq_pci_xscom_read,
    .write = pnv_pbcq_pci_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const MemoryRegionOps pnv_pbcq_spci_xscom_ops = {
    .read = pnv_pbcq_spci_xscom_read,
    .write = pnv_pbcq_spci_xscom_write,
    .valid.min_access_size = 8,
    .valid.max_access_size = 8,
    .impl.min_access_size = 8,
    .impl.max_access_size = 8,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void pnv_pbcq_default_bars(PnvPBCQState *pbcq)
{
    uint64_t mm0, mm1, reg;

    mm0 = 0x3d00000000000ull +
            0x4000000000ull * pbcq->chip_id +
            0x1000000000ull * pbcq->phb_id;
    mm1 = 0x3ff8000000000ull +
            0x0200000000ull * pbcq->chip_id +
            0x0080000000ull * pbcq->phb_id;
    reg = 0x3fffe40000000ull +
            0x0000400000ull * pbcq->chip_id +
            0x0000100000ull * pbcq->phb_id;

    pbcq->nest_regs[PBCQ_NEST_MMIO_BAR0] = mm0 << 14;
    pbcq->nest_regs[PBCQ_NEST_MMIO_BAR1] = mm1 << 14;
    pbcq->nest_regs[PBCQ_NEST_PHB_BAR] = reg << 14;
    pbcq->nest_regs[PBCQ_NEST_MMIO_MASK0] = 0x3fff000000000ull << 14;
    pbcq->nest_regs[PBCQ_NEST_MMIO_MASK1] = 0x3ffff80000000ull << 14;
    pbcq->pci_regs[PBCQ_PCI_BAR2] = reg << 14;
}

static void pnv_pbcq_realize(DeviceState *dev, Error **errp)
{
    PnvPBCQState *pbcq = PNV_PBCQ(dev);
    Object *obj;
    Error *err = NULL;

    assert(pbcq->phb_id < 4);

    obj = object_property_get_link(OBJECT(dev), "phb", &err);
    if (!obj) {
        error_setg(errp, "%s: required link 'phb' not found: %s",
                   __func__, error_get_pretty(err));
        return;
    }
    pbcq->phb = PNV_PHB3(obj);

    /* XXX Fix OPAL to do that: establish default BAR values */
    pnv_pbcq_default_bars(pbcq);

    /* XScom region for PBCQ registers */
    pnv_xscom_region_init(&pbcq->xscom_nest_regs, OBJECT(dev),
                          &pnv_pbcq_nest_xscom_ops,
                          pbcq, "xscom-pbcq-nest",
                          PNV_XSCOM_PBCQ_NEST_SIZE);
    pnv_xscom_region_init(&pbcq->xscom_pci_regs, OBJECT(dev),
                          &pnv_pbcq_pci_xscom_ops,
                          pbcq, "xscom-pbcq-pci",
                          PNV_XSCOM_PBCQ_PCI_SIZE);
    pnv_xscom_region_init(&pbcq->xscom_spci_regs, OBJECT(dev),
                          &pnv_pbcq_spci_xscom_ops,
                          pbcq, "xscom-pbcq-spci",
                          PNV_XSCOM_PBCQ_SPCI_SIZE);
}

static int pnv_pbcq_dt_xscom(PnvXScomInterface *dev, void *fdt,
                             int xscom_offset)
{
    const char compat[] = "ibm,power8-pbcq";
    PnvPBCQState *pbcq = PNV_PBCQ(dev);
    char *name;
    int offset;
    uint32_t lpc_pcba = PNV_XSCOM_PBCQ_NEST_BASE;
    uint32_t reg[] = {
        cpu_to_be32(lpc_pcba),
        cpu_to_be32(PNV_XSCOM_PBCQ_NEST_SIZE),
        cpu_to_be32(PNV_XSCOM_PBCQ_PCI_BASE),
        cpu_to_be32(PNV_XSCOM_PBCQ_PCI_SIZE),
        cpu_to_be32(PNV_XSCOM_PBCQ_SPCI_BASE),
        cpu_to_be32(PNV_XSCOM_PBCQ_SPCI_SIZE)
    };

    name = g_strdup_printf("pbcq@%x", lpc_pcba);
    offset = fdt_add_subnode(fdt, xscom_offset, name);
    _FDT(offset);
    g_free(name);

    _FDT((fdt_setprop(fdt, offset, "reg", reg, sizeof(reg))));

    _FDT((fdt_setprop_cell(fdt, offset, "ibm,phb-index", pbcq->phb_id)));
    _FDT((fdt_setprop(fdt, offset, "compatible", compat,
                      sizeof(compat))));
    return 0;
}

static Property pnv_pbcq_properties[] = {
        DEFINE_PROP_UINT32("phb-id", PnvPBCQState, phb_id, 0),
        DEFINE_PROP_UINT32("chip-id", PnvPBCQState, chip_id, 0),
        DEFINE_PROP_END_OF_LIST(),

};

static void pnv_pbcq_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PnvXScomInterfaceClass *xdc = PNV_XSCOM_INTERFACE_CLASS(klass);

    xdc->dt_xscom = pnv_pbcq_dt_xscom;

    dc->realize = pnv_pbcq_realize;
    dc->props = pnv_pbcq_properties;
}

static const TypeInfo pnv_pbcq_type_info = {
    .name          = TYPE_PNV_PBCQ,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(PnvPBCQState),
    .class_init    = pnv_pbcq_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { TYPE_PNV_XSCOM_INTERFACE },
        { }
    }
};

static void pnv_pbcq_register_types(void)
{
    type_register_static(&pnv_pbcq_type_info);
}

type_init(pnv_pbcq_register_types)
