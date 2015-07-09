/*-
 * Copyright (c) 2012, Gavin Atkinson <gavin@FreeBSD.org>
 * Copyright (c) 2015 The FreeBSD Foundation
 * All rights reserved.
 *
 * Portions of this software were developed by John-Mark Gurney
 * under sponsorship from the FreeBSD Foundation.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_bus.h"
#include "opt_kdtrace.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/queue.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/endian.h>
#include <sys/sdt.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>
#include <machine/stdarg.h>

#if defined(__i386__) || defined(__amd64__) || defined(__powerpc__)
#include <machine/intr_machdep.h>
#endif

#include <sys/pciio.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pci_private.h>

#include "pcib_if.h"
#include "pci_if.h"

SDT_PROVIDER_DEFINE(pci);

SDT_PROBE_DEFINE2(pci, pciehp, intr, entry, "uint32_t", "uint32_t");
SDT_PROBE_DEFINE4(pci, pciehp, task, entry, "uint32_t", "uint32_t",
	"uint32_t", "uint32_t");
SDT_PROBE_DEFINE(pci, pciehp, task, add_children);
SDT_PROBE_DEFINE(pci, pciehp, task, remove_children);
SDT_PROBE_DEFINE(pci, pciehp, task, ignored);
SDT_PROBE_DEFINE1(pci, pciehp, task, other, "uint32_t");

static struct resource_spec hotplug_res_spec_msi[] = {
	{ SYS_RES_IRQ,		1,		RF_ACTIVE },
	{ -1,			0,		0 }
};


static void
pci_link_status_print(device_t pcib)
{
	struct pci_devinfo *dinfo;
	int link_sta, pos;

	dinfo = device_get_ivars(pcib);
	pos = dinfo->cfg.pcie.pcie_location;
	link_sta = pci_read_config(pcib, pos + PCIER_LINK_STA, 2);
	device_printf(pcib, "... LINK_STA=0x%b, width %dx\n",
	    link_sta,
	    "\020"
#if 0
	    "\001<b0>"
	    "\002<b1>"
	    "\003<b2>"
	    "\004<b3>"
	    "\005<b4>"
	    "\006<b5>"
	    "\007<b6>"
	    "\010<b7>"
	    "\011<b8>"
	    "\012<b9>"
	    "\013Undef"
#endif
	    "\014LinkTrain"
	    "\015SlotClkConfig"
	    "\016DLLLinkActive"
	    "\017LinkBWManStat"
	    "\020LinkAutonBwStat",
	    (link_sta & 0x03f0) >> 4);
}

static int power_values[] = { 250, 275, 300 };

static void
pci_slot_cap_power(uint32_t reg, char buf[6])
{
	int val, mag;
	int whole, fract, fractcnt, fractmul;
	int avail;

	val = (reg >> 7) & 0xff;
	mag = (reg >> 15) & 0x3;
	fract = 0;
	fractcnt = 0;
	if (val < 0xf0) {
		fractmul = 1;
		whole = val;
		while (mag--) {
			fract += (whole % 10) * fractmul;
			whole /= 10;
			fractcnt++;
			fractmul *= 10;
		}
	} else {
		if (val < 0xf3)
			whole = power_values[val - 0xf0];
		else
			whole = 301;
	}
	avail = 6;
	snprintf(buf, avail, "%d", whole);
	if (fractcnt) {
		avail -= strlen(buf);
		buf += strlen(buf);
		snprintf(buf, avail, ".%0*d", fractcnt, fract);
	}
}

static void
pci_slot_cap_print(device_t pcib)
{
	char buf[6];
	struct pci_devinfo *dinfo;
	int pos;
	uint32_t reg;

	dinfo = device_get_ivars(pcib);
	pos = dinfo->cfg.pcie.pcie_location;
	reg = pci_read_config(pcib, pos+PCIER_SLOT_CAP, 4);
	pci_slot_cap_power(reg, buf);
	device_printf(pcib, "... SLOT_CAP=0x%b, #%u, %sW\n", reg,
	    "\020"
	    "\001AttnButt"
	    "\002PowerCtrl"
	    "\003MRLSens"
	    "\004AttnInd"
	    "\005PwrInd"
	    "\006HotPlugSurp"
	    "\007HotPlugCap"
#if 0
	    "\010<b7>"
	    "\011<b8>"
	    "\012<b9>"
	    "\013<b10>"
	    "\014<b11>"
	    "\015<b12>"
	    "\016<b13>"
	    "\017<b14>"
	    "\020<b15>"
	    "\021<b16>"
#endif
	    "\022ElecMechInt"
	    "\023NoCCS"
#if 0
	    "\024<b19>"
	    "\025<b20>"
	    "\026<b21>"
	    "\027<b22>"
#endif
	    , PCIEM_SLOT_CAP_GETPSN(reg), buf);
}

static void
pci_slot_control_print(device_t pcib)
{
	struct pci_devinfo *dinfo;
	int pos;

	dinfo = device_get_ivars(pcib);
	pos = dinfo->cfg.pcie.pcie_location;
	device_printf(pcib, "... SLOT_CTL=0x%b\n",
	    pci_read_config(pcib, pos + PCIER_SLOT_CTL, 2),
	    "\020"
	    "\001AttnButtPressEn"
	    "\002PowerFaultDetEn"
	    "\003MRLSensChgEn"
	    "\004PresDetChgEn"
	    "\005CmdCompIntEn"
	    "\006HotPlugIntEn"
	    "\007AttnIndCtl1"
	    "\010AttnIndCtl2"
	    "\012PwrIndCtl2"
	    "\013PwrCtrlrCtl"
	    "\014ElecMechIntCtl"
	    "\015DLLStatChEn"
#if 0
	    "\016<b13>"
	    "\017<b14>"
	    "\020<b15>"
#endif
	    );
}

static void
pci_slot_status_print(device_t pcib)
{
	struct pci_devinfo *dinfo;
	int pos;

	dinfo = device_get_ivars(pcib);
	pos = dinfo->cfg.pcie.pcie_location;
	device_printf(pcib, "... SLOT_STA=0x%b\n",
	    pci_read_config(pcib, pos + PCIER_SLOT_STA, 2),
	    "\020"
	    "\001AttnButtPress"
	    "\002PowerFaultDet"
	    "\003MRLSensChg"
	    "\004PresDetChg"
	    "\005CmdComplete"
	    "\006MRLSensState"
	    "\007PresDetState"
	    "\010ElecMechIntState"
	    "\011DLLStateChg"
#if 0
	    "\012<b9>"
	    "\013<b10>"
	    "\014<b11>"
	    "\015<b12>"
	    "\016<b13>"
	    "\017<b14>"
	    "\020<b15>"
#endif
	    );
}

void pci_slot_reg_print(device_t pcib);
void
pci_slot_reg_print(device_t pcib)
{

	pci_link_status_print(pcib);
	pci_slot_cap_print(pcib);
	pci_slot_status_print(pcib);
	pci_slot_control_print(pcib);
}

static void
rescan_bus(void *arg)
{
	device_t dev;

	dev = arg;

	SDT_PROBE(pci, pciehp, task, add_children, 0, 0, 0, 0, 0);
	pci_add_children(dev);
	(void)bus_generic_attach(dev);

}

static void
pci_hotplug_intr_task(void *arg, int npending)
{
	device_t dev;
	device_t pcib;
	device_t *devlistp;
	struct pci_devinfo *dinfo;
	int devcnt, i, pos;
	uint32_t linksta, slotsta, staclr;
	uint16_t ctrl;

	dev = arg;
	pcib = device_get_parent(dev);

	dinfo = device_get_ivars(pcib);
	pos = dinfo->cfg.pcie.pcie_location;

	mtx_lock(&Giant);

	linksta = pci_read_config(pcib, pos + PCIER_LINK_STA, 2);
	slotsta = pci_read_config(pcib, pos + PCIER_SLOT_STA, 2);
	staclr = 0;

	SDT_PROBE(pci, pciehp, task, entry,
	    slotsta,
	    linksta,
	    pci_read_config(pcib, pos + PCIER_SLOT_CAP, 2),
	    pci_read_config(pcib, pos + PCIER_SLOT_CTL, 2),
	    0);
#if 0
	pci_slot_reg_print(pcib);
#endif
/* XXXGA: HACK AHEAD */
	if (slotsta & PCIEM_SLOT_STA_CC) {
		/* XXX - handle command completed events to advance state machine */
	}

	if (slotsta & PCIEM_SLOT_STA_PDC &&	/* presence change */
	    dinfo->cfg.hp.hp_slotcap & PCIEM_SLOT_CAP_PCP && /* power ctrlr */
	    slotsta & PCIEM_SLOT_STA_PDS) {	/* present */
		ctrl = pci_read_config(pcib, pos + PCIER_SLOT_CTL, 2);
		ctrl &= ~PCIEM_SLOT_CTL_PCC;
		pci_write_config(pcib, pos + PCIER_SLOT_CTL, ctrl, 2);
		staclr |= PCIEM_SLOT_STA_PDC;
		/* XXX - start timeout */
	}

	if (slotsta & PCIEM_SLOT_STA_PDC &&	/* presence change */
	    dinfo->cfg.hp.hp_slotcap & PCIEM_SLOT_CAP_PCP && /* power ctrlr */
	    !(slotsta & PCIEM_SLOT_STA_PDS)) {	/* not present */
		ctrl = pci_read_config(pcib, pos + PCIER_SLOT_CTL, 2);
		ctrl |= PCIEM_SLOT_CTL_PCC;
		pci_write_config(pcib, pos + PCIER_SLOT_CTL, ctrl, 2);
		staclr |= PCIEM_SLOT_STA_PDC;
	}

	if (slotsta & PCIEM_SLOT_STA_DLLSC) {
		if ((linksta & PCIEM_LINK_STA_DL_ACTIVE) &&
		    dinfo->cfg.hp.hp_cnt == 0) {
			dinfo->cfg.hp.hp_cnt = 1;
			/*
			 * Per 6.7.3.3, delay for 100ms after DLL Active
			 * before talking to device.
			 */
			callout_reset(&dinfo->cfg.hp.hp_co, hz / 10,
			    rescan_bus, dev);
		} else if (((linksta & PCIEM_LINK_STA_DL_ACTIVE) == 0) &&
		    dinfo->cfg.hp.hp_cnt == 1) {

			/*
			 * XXX - do we do the code here or when it is no
			 * longer present?
			 */
			SDT_PROBE(pci, pciehp, task, remove_children, 0, 0, 0, 0, 0);
			device_get_children(dev, &devlistp, &devcnt);
			for (i = 0; i < devcnt; i++)
				pci_delete_child(dev, devlistp[i]);
			free(devlistp, M_TEMP);
			dinfo->cfg.hp.hp_cnt = 0;
		} else
			SDT_PROBE(pci, pciehp, task, ignored, 0, 0, 0, 0, 0);

		staclr |= PCIEM_SLOT_STA_DLLSC;
	}

	/* we only care about bits we can clear after here */
	slotsta &= PCIEM_SLOT_STA_EMASK;

	/* log any status changes we didn't handle */
	if (slotsta & ~staclr)
		SDT_PROBE(pci, pciehp, task, other,
		    slotsta & ~staclr, 0, 0, 0, 0);

	/* always clear the sta reg as if we don't, we get an interrupt storm */
	pci_write_config(pcib, pos + PCIER_SLOT_STA, slotsta, 2);

	mtx_unlock(&Giant);
}

static int
pci_hotplug_intr(void *arg)
{
	device_t dev = arg;
	device_t pcib = device_get_parent(dev);
	struct pci_devinfo *dinfo;
	int pos;

	dinfo = device_get_ivars(pcib);
	pos = dinfo->cfg.pcie.pcie_location;
	    
	SDT_PROBE(pci, pciehp, intr, entry,
	    pci_read_config(pcib, pos + PCIER_SLOT_STA, 2),
	    pci_read_config(pcib, pos + PCIER_LINK_STA, 2),
	    0, 0, 0);
#if 0
	device_printf(dev, "Received interrupt!\n");
	pci_slot_status_print(pcib);
#endif
	taskqueue_enqueue_fast(taskqueue_fast, &dinfo->cfg.hp.hp_inttask);

	return (FILTER_HANDLED);
}

/*
 * XXX - Make sure the minimum allocations are done.
 *
 * Minimum allocations for ExpressCard:
 * 8 busses
 * 0 Prefetchable
 * 32Meg non-prefetchable
 * 4k I/O
 */
static void
pci_hotplug_setup(device_t dev)
{
	device_t pcib = device_get_parent(dev);
	struct pci_devinfo *dinfo;
	int cap, error, flags, msic, pos;
	uint16_t slotctl;

	dinfo = device_get_ivars(pcib);

	pos = dinfo->cfg.pcie.pcie_location;
	flags = pci_read_config(pcib, pos + PCIER_FLAGS, 2);
	cap = pci_read_config(pcib, pos + PCIER_SLOT_CAP, 4);
	dinfo->cfg.hp.hp_slotcap = cap;

	if (bootverbose)
		pci_slot_status_print(pcib);

	dinfo->cfg.hp.hp_cnt = 0;
	callout_init(&dinfo->cfg.hp.hp_co, 0);

	device_printf(dev, "Hot plug slot number %u\n",
	    PCIEM_SLOT_CAP_GETPSN(cap));

	if (!(cap & PCIEM_SLOT_CAP_NCCS))
		device_printf(dev, "WARNING! Command Completion Status not supported!");

#if 0
	/*
	 * IRQ is ignored as PCIe requires MSI.  It is possible that we could
	 * have PCIe devices on a PCI bus on a motherboard that doesn't
	 * support MSI, but I hope that we'll do the proper work in the bus
	 * code to support it.
	 */
	int irq;
	irq = (flags & PCIEM_FLAGS_IRQ) >> 9;
	device_printf(dev, "IRQ = %d\n", irq);
	device_printf(dev, "MSI count self %d parent %d\n",
	    pci_msi_count(dev), pci_msi_count(pcib));
	device_printf(dev, "MSI-X count self %d parent %d\n",
	    pci_msix_count(dev), pci_msix_count(pcib));
#endif
	msic = pci_msi_count(pcib);
	if (msic) {
		/* I've seen a device w/ 8 MSI, but the first one works. */
		msic = 1;
		if (pci_alloc_msi(pcib, &msic) == 0) {
			if (msic == 1) {
				device_printf(dev,
				    "Using %d MSI messages\n",
				    msic);
				dinfo->cfg.pcie.pcie_irq_spec =
				    hotplug_res_spec_msi;
			} else {
				device_printf(dev,
				    "Error: %d MSI messages, ABORTING.\n",
				    msic);
				pci_release_msi(dev);
				return;
			}
		}
	}
	error = bus_alloc_resources(pcib,
	    dinfo->cfg.pcie.pcie_irq_spec,
	    dinfo->cfg.pcie.pcie_res_irq);
	if (error) {
		device_printf(dev,
		    "couldn't allocate IRQ resources, %d, ABORTING.\n",
		    error);
		return;
	} else {
		error = bus_setup_intr(pcib,
		    dinfo->cfg.pcie.pcie_res_irq[0],
		    INTR_TYPE_AV | INTR_MPSAFE,
		    pci_hotplug_intr, NULL, dev,
		    &dinfo->cfg.pcie.pcie_intrhand[0]);
		if (error) {
			device_printf(dev, "couldn't set up IRQ resources, %d\n",
			    error);
		}
	}

	TASK_INIT(&dinfo->cfg.hp.hp_inttask, 0,
	    pci_hotplug_intr_task, dev);

	/* Select and enable events to interrupt upon */
	slotctl = pci_read_config(pcib, pos + PCIER_SLOT_CTL, 2);
	slotctl |= PCIEM_SLOT_CTL_PDCE | PCIEM_SLOT_CTL_HPIE;
	if (cap & PCIEM_SLOT_CAP_MRLSP)
		slotctl |= PCIEM_SLOT_CTL_MRLSCE;
	if (pci_read_config(pcib, pos + PCIER_LINK_CAP, 4) &
	    PCIEM_LINK_CAP_DL_ACTIVE)
		slotctl |= PCIEM_SLOT_CTL_DLLSCE;
	pci_write_config(pcib, pos + PCIER_SLOT_CTL, slotctl, 2);
	if (bootverbose) {
		device_printf(dev, "Enabled interrupts\n");
		pci_slot_control_print(pcib);
		pci_slot_status_print(pcib);
	}

	/* XXX - Do we check for presence? */
	if (pci_read_config(pcib, pos + PCIER_LINK_STA,
	    2) & PCIEM_LINK_STA_DL_ACTIVE) {
		dinfo->cfg.hp.hp_cnt = 1;
		/* XXX - should we wait 100ms? */
	}
}

void
pci_hotplug_init(device_t dev)
{
	device_t pcib = device_get_parent(dev);
	struct pci_devinfo *dinfo;
	int cap, flags, pos;

	dinfo = device_get_ivars(pcib);
	pos = dinfo->cfg.pcie.pcie_location;
	if (pos != 0) {
		flags = pci_read_config(pcib, pos + PCIER_FLAGS, 2);
		cap = pci_read_config(pcib, pos + PCIER_SLOT_CAP, 4);
		if (bootverbose)
			pci_slot_cap_print(pcib);
		if ((flags & PCIEM_FLAGS_SLOT &&
		    cap & PCIEM_SLOT_CAP_HPC)) {
			pci_hotplug_setup(dev);
		}
	}
}
