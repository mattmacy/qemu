/*
 * QEMU PowerPC XIVE interrupt controller model
 *
 * Copyright (c) 2017-2018, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later. See the
 * COPYING file in the top-level directory.
 */

#ifndef PPC_XIVE_H
#define PPC_XIVE_H

#include "hw/sysbus.h"
#include "hw/ppc/xive_regs.h"

/*
 * XIVE Fabric (Interface between Source and Router)
 */

typedef struct XiveFabric {
    Object parent;
} XiveFabric;

#define TYPE_XIVE_FABRIC "xive-fabric"
#define XIVE_FABRIC(obj)                                     \
    OBJECT_CHECK(XiveFabric, (obj), TYPE_XIVE_FABRIC)
#define XIVE_FABRIC_CLASS(klass)                                     \
    OBJECT_CLASS_CHECK(XiveFabricClass, (klass), TYPE_XIVE_FABRIC)
#define XIVE_FABRIC_GET_CLASS(obj)                                   \
    OBJECT_GET_CLASS(XiveFabricClass, (obj), TYPE_XIVE_FABRIC)

typedef struct XiveFabricClass {
    InterfaceClass parent;
    void (*notify)(XiveFabric *xf, uint32_t lisn);
} XiveFabricClass;

/*
 * XIVE Interrupt Source
 */

#define TYPE_XIVE_SOURCE "xive-source"
#define XIVE_SOURCE(obj) OBJECT_CHECK(XiveSource, (obj), TYPE_XIVE_SOURCE)

/*
 * XIVE Interrupt Source characteristics, which define how the ESB are
 * controlled.
 */
#define XIVE_SRC_H_INT_ESB     0x1 /* ESB managed with hcall H_INT_ESB */
#define XIVE_SRC_STORE_EOI     0x2 /* Store EOI supported */

typedef struct XiveSource {
    SysBusDevice parent;

    /* IRQs */
    uint32_t        nr_irqs;
    qemu_irq        *qirqs;
    unsigned long   *lsi_map;
    int32_t         lsi_map_size; /* for VMSTATE_BITMAP */

    /* PQ bits and LSI assertion bit */
    uint8_t         *status;

    /* ESB memory region */
    uint64_t        esb_flags;
    uint32_t        esb_shift;
    MemoryRegion    esb_mmio;

    XiveFabric      *xive;
} XiveSource;

/*
 * ESB MMIO setting. Can be one page, for both source triggering and
 * source management, or two different pages. See below for magic
 * values.
 */
#define XIVE_ESB_4K          12 /* PSI HB only */
#define XIVE_ESB_4K_2PAGE    13
#define XIVE_ESB_64K         16
#define XIVE_ESB_64K_2PAGE   17

static inline bool xive_source_esb_has_2page(XiveSource *xsrc)
{
    return xsrc->esb_shift == XIVE_ESB_64K_2PAGE ||
        xsrc->esb_shift == XIVE_ESB_4K_2PAGE;
}

/* The trigger page is always the first/even page */
static inline hwaddr xive_source_esb_page(XiveSource *xsrc, uint32_t srcno)
{
    assert(srcno < xsrc->nr_irqs);
    return (1ull << xsrc->esb_shift) * srcno;
}

/* In a two pages ESB MMIO setting, the odd page is for management */
static inline hwaddr xive_source_esb_mgmt(XiveSource *xsrc, int srcno)
{
    hwaddr addr = xive_source_esb_page(xsrc, srcno);

    if (xive_source_esb_has_2page(xsrc)) {
        addr += (1 << (xsrc->esb_shift - 1));
    }

    return addr;
}

/*
 * Each interrupt source has a 2-bit state machine which can be
 * controlled by MMIO. P indicates that an interrupt is pending (has
 * been sent to a queue and is waiting for an EOI). Q indicates that
 * the interrupt has been triggered while pending.
 *
 * This acts as a coalescing mechanism in order to guarantee that a
 * given interrupt only occurs at most once in a queue.
 *
 * When doing an EOI, the Q bit will indicate if the interrupt
 * needs to be re-triggered.
 */
#define XIVE_STATUS_ASSERTED  0x4  /* Extra bit for LSI */
#define XIVE_ESB_VAL_P        0x2
#define XIVE_ESB_VAL_Q        0x1

#define XIVE_ESB_RESET        0x0
#define XIVE_ESB_PENDING      XIVE_ESB_VAL_P
#define XIVE_ESB_QUEUED       (XIVE_ESB_VAL_P | XIVE_ESB_VAL_Q)
#define XIVE_ESB_OFF          XIVE_ESB_VAL_Q

/*
 * "magic" Event State Buffer (ESB) MMIO offsets.
 *
 * The following offsets into the ESB MMIO allow to read or manipulate
 * the PQ bits. They must be used with an 8-byte load instruction.
 * They all return the previous state of the interrupt (atomically).
 *
 * Additionally, some ESB pages support doing an EOI via a store and
 * some ESBs support doing a trigger via a separate trigger page.
 */
#define XIVE_ESB_STORE_EOI      0x400 /* Store */
#define XIVE_ESB_LOAD_EOI       0x000 /* Load */
#define XIVE_ESB_GET            0x800 /* Load */
#define XIVE_ESB_SET_PQ_00      0xc00 /* Load */
#define XIVE_ESB_SET_PQ_01      0xd00 /* Load */
#define XIVE_ESB_SET_PQ_10      0xe00 /* Load */
#define XIVE_ESB_SET_PQ_11      0xf00 /* Load */

uint8_t xive_source_esb_get(XiveSource *xsrc, uint32_t srcno);
uint8_t xive_source_esb_set(XiveSource *xsrc, uint32_t srcno, uint8_t pq);

void xive_source_pic_print_info(XiveSource *xsrc, uint32_t offset,
                                Monitor *mon);

static inline qemu_irq xive_source_qirq(XiveSource *xsrc, uint32_t srcno)
{
    assert(srcno < xsrc->nr_irqs);
    return xsrc->qirqs[srcno];
}

static inline bool xive_source_irq_is_lsi(XiveSource *xsrc, uint32_t srcno)
{
    assert(srcno < xsrc->nr_irqs);
    return test_bit(srcno, xsrc->lsi_map);
}

static inline void xive_source_irq_set(XiveSource *xsrc, uint32_t srcno,
                                       bool lsi)
{
    assert(srcno < xsrc->nr_irqs);
    if (lsi) {
        bitmap_set(xsrc->lsi_map, srcno, 1);
    }
}

/*
 * XIVE Router
 */

typedef struct XiveRouter {
    SysBusDevice    parent;

    uint32_t        chip_id;
} XiveRouter;

#define TYPE_XIVE_ROUTER "xive-router"
#define XIVE_ROUTER(obj)                                \
    OBJECT_CHECK(XiveRouter, (obj), TYPE_XIVE_ROUTER)
#define XIVE_ROUTER_CLASS(klass)                                        \
    OBJECT_CLASS_CHECK(XiveRouterClass, (klass), TYPE_XIVE_ROUTER)
#define XIVE_ROUTER_GET_CLASS(obj)                              \
    OBJECT_GET_CLASS(XiveRouterClass, (obj), TYPE_XIVE_ROUTER)

typedef struct XiveRouterClass {
    SysBusDeviceClass parent;

    /* XIVE table accessors */
    int (*get_ive)(XiveRouter *xrtr, uint32_t lisn, XiveIVE *ive);
    int (*set_ive)(XiveRouter *xrtr, uint32_t lisn, XiveIVE *ive);
    int (*get_eq)(XiveRouter *xrtr, uint8_t eq_blk, uint32_t eq_idx,
                   XiveEQ *eq);
    int (*set_eq)(XiveRouter *xrtr, uint8_t eq_blk, uint32_t eq_idx,
                   XiveEQ *eq);
    int (*get_vp)(XiveRouter *xrtr, uint8_t vp_blk, uint32_t vp_idx,
                   XiveVP *vp);
    int (*set_vp)(XiveRouter *xrtr, uint8_t vp_blk, uint32_t vp_idx,
                  XiveVP *vp);
} XiveRouterClass;

void xive_router_print_ive(XiveRouter *xrtr, uint32_t lisn, XiveIVE *ive,
                           Monitor *mon);
int xive_router_get_ive(XiveRouter *xrtr, uint32_t lisn, XiveIVE *ive);
int xive_router_set_ive(XiveRouter *xrtr, uint32_t lisn, XiveIVE *ive);
int xive_router_get_eq(XiveRouter *xrtr, uint8_t eq_blk, uint32_t eq_idx,
                       XiveEQ *eq);
int xive_router_set_eq(XiveRouter *xrtr, uint8_t eq_blk, uint32_t eq_idx,
                       XiveEQ *eq);
int xive_router_get_vp(XiveRouter *xrtr, uint8_t vp_blk, uint32_t vp_idx,
                       XiveVP *vp);
int xive_router_set_vp(XiveRouter *xrtr, uint8_t vp_blk, uint32_t vp_idx,
                       XiveVP *vp);

/*
 * XIVE EQ ESBs
 */

#define TYPE_XIVE_EQ_SOURCE "xive-eq-source"
#define XIVE_EQ_SOURCE(obj) \
    OBJECT_CHECK(XiveEQSource, (obj), TYPE_XIVE_EQ_SOURCE)

typedef struct XiveEQSource {
    SysBusDevice parent;

    uint32_t        nr_eqs;

    /* ESB memory region */
    uint32_t        esb_shift;
    MemoryRegion    esb_mmio;

    XiveRouter      *xrtr;
} XiveEQSource;

/*
 * For legacy compatibility, the exceptions define up to 256 different
 * priorities. P9 implements only 9 levels : 8 active levels [0 - 7]
 * and the least favored level 0xFF.
 */
#define XIVE_PRIORITY_MAX  7

void xive_eq_reset(XiveEQ *eq);


/*
 * XIVE Thread interrupt Management (TM) context
 */

#define TYPE_XIVE_TCTX "xive-tctx"
#define XIVE_TCTX(obj) OBJECT_CHECK(XiveTCTX, (obj), TYPE_XIVE_TCTX)

/*
 * XIVE Thread interrupt Management register rings :
 *
 *   QW-0  User       event-based exception state
 *   QW-1  O/S        OS context for priority management, interrupt acks
 *   QW-2  Pool       hypervisor context for virtual processor being dispatched
 *   QW-3  Physical   for the security monitor to manage the entire context
 */
#define TM_RING_COUNT           4
#define TM_RING_SIZE            0x10

typedef struct XiveTCTX {
    DeviceState parent_obj;

    CPUState    *cs;
    qemu_irq    output;

    uint8_t     regs[TM_RING_COUNT * TM_RING_SIZE];

    XiveRouter  *xrtr;
} XiveTCTX;

/*
 * XIVE Thread Interrupt Management Aera (TIMA)
 */
extern const MemoryRegionOps xive_tm_ops;

void xive_tctx_pic_print_info(XiveTCTX *tctx, Monitor *mon);

#endif /* PPC_XIVE_H */