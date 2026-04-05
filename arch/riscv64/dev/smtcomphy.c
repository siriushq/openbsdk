/*	$OpenBSD: smtcomphy.c,v 1.1 2026/04/05 18:08:11 kettenis Exp $	*/
/*
 * Copyright (c) 2026 Mark Kettenis <kettenis@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <machine/intr.h>
#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_clock.h>
#include <dev/ofw/ofw_misc.h>
#include <dev/ofw/ofw_regulator.h>
#include <dev/ofw/fdt.h>

/* PHY registers */
#define PCIE_PU_ADDR_CLK_CFG(_lane)	(0x0008 + (_lane) * 0x0400)
#define  PLL_READY			(1U << 0)
#define  CFG_INTERNAL_TIMER_ADJ_MASK	(0xf << 7)
#define  CFG_INTERNAL_TIMER_ADJ_USB3	(0x2 << 7)
#define  CFG_SW_PHY_INIT_DONE		(1U << 11)
#define PCIE_PU_PLL_1			0x0048
#define  REF_100_WSSC			(1U << 12)
#define  FREF_SEL_MASK			(0x7 << 13)
#define  FREF_SEL_24M			(0x1 << 13)
#define  SSC_DEP_SEL_MASK		(0xf << 16)
#define  SSC_DEP_SEL_5000PPM		(0xa << 16)
#define USB3_TEST_CTRL			0x0068
#define PCIE_RCAL_RESULT		0x0084
#define  R_TUNE_DONE			(1U << 10)

/* APMU registers */
#define APMU_PMUA_USB_PHY_CTRL0		0x0110
#define  APMU_COMBO_PHY_SEL		(1U << 3)
#define APMU_PCIE_CLK_RES_CTRL_PORTA	0x03cc
#define  APMU_PCIE_APP_HOLD_PHY_RST	(1U << 30)

#define HREAD4(sc, reg)							\
	(bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, (reg)))
#define HWRITE4(sc, reg, val)						\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, (reg), (val))

struct smtcomphy_softc {
	struct device		sc_dev;
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
	struct regmap		*sc_apmu;

	int			sc_node;
	int			sc_num_lanes;

	struct phy_device	sc_pd;
};

int smtcomphy_match(struct device *, void *, void *);
void smtcomphy_attach(struct device *, struct device *, void *);

const struct cfattach smtcomphy_ca = {
	sizeof (struct smtcomphy_softc), smtcomphy_match, smtcomphy_attach
};

struct cfdriver smtcomphy_cd = {
	NULL, "smtcomphy", DV_DULL
};

int	smtcomphy_combo_init(struct smtcomphy_softc *sc);
int	smtcomphy_combo_enable(void *, uint32_t *);
int	smtcomphy_pcie_enable(void *, uint32_t *);

int
smtcomphy_match(struct device *parent, void *match, void *aux)
{
	struct fdt_attach_args *faa = aux;

	return OF_is_compatible(faa->fa_node, "spacemit,k1-combo-phy") ||
	    OF_is_compatible(faa->fa_node, "spacemit,k1-pcie-phy");
}

void
smtcomphy_attach(struct device *parent, struct device *self, void *aux)
{
	struct smtcomphy_softc *sc = (struct smtcomphy_softc *)self;
	struct fdt_attach_args *faa = aux;

	if (faa->fa_nreg < 1) {
		printf(": no registers\n");
		return;
	}

	sc->sc_iot = faa->fa_iot;
	if (bus_space_map(sc->sc_iot, faa->fa_reg[0].addr,
	    faa->fa_reg[0].size, 0, &sc->sc_ioh)) {
		printf(": can't map registers\n");
		return;
	}
	sc->sc_node = faa->fa_node;

	printf("\n");

	reset_deassert(sc->sc_node, "phy");

	if (OF_is_compatible(sc->sc_node, "spacemit,k1-combo-phy")) {
		if (smtcomphy_combo_init(sc))
			return;
		sc->sc_num_lanes = 1;
	} else {
		sc->sc_num_lanes = OF_getpropint(sc->sc_node, "num-lanes", 2);
	}

	sc->sc_pd.pd_node = sc->sc_node;
	sc->sc_pd.pd_cookie = sc;
	if (OF_is_compatible(faa->fa_node, "spacemit,k1-combo-phy"))
		sc->sc_pd.pd_enable = smtcomphy_combo_enable;
	else
		sc->sc_pd.pd_enable = smtcomphy_pcie_enable;
	phy_register(&sc->sc_pd);
}

int
smtcomphy_combo_init(struct smtcomphy_softc *sc)
{
	uint32_t apmu, val;

	apmu = OF_getpropint(sc->sc_node, "spacemit,apmu", 0);
	sc->sc_apmu = regmap_byphandle(apmu);
	if (sc->sc_apmu == NULL) {
		printf("%s: can't get apmu\n", sc->sc_dev.dv_xname);
		return -1;
	}

	val = regmap_read_4(sc->sc_apmu, APMU_PCIE_CLK_RES_CTRL_PORTA);
	val &= ~APMU_PCIE_APP_HOLD_PHY_RST;
	regmap_write_4(sc->sc_apmu, APMU_PCIE_CLK_RES_CTRL_PORTA, val);

	val = HREAD4(sc, PCIE_RCAL_RESULT);
	if (val & R_TUNE_DONE)
		return 0;

	/* Firmware should have calibrated the PHY for us. */
	printf("%s: not calibrated\n", sc->sc_dev.dv_xname);
	return -1;
}

void
smtcomphy_pll_init_common(struct smtcomphy_softc *sc)
{
	uint32_t val;
	int lane, timo;

	val = HREAD4(sc, PCIE_PU_PLL_1);
	val &= ~REF_100_WSSC;
	val &= ~FREF_SEL_MASK;
	val |= FREF_SEL_24M;
	HWRITE4(sc, PCIE_PU_PLL_1, val);

	for (lane = 0; lane < sc->sc_num_lanes; lane++) {
		val = HREAD4(sc, PCIE_PU_ADDR_CLK_CFG(lane));
		val |= CFG_SW_PHY_INIT_DONE;
		HWRITE4(sc, PCIE_PU_ADDR_CLK_CFG(lane), val);
	}

	for (timo = 1000; timo > 0; timo--) {
		if (HREAD4(sc, PCIE_PU_ADDR_CLK_CFG(0)) & PLL_READY)
			break;
		delay(500);
	}
	if (timo == 0)
		printf("%s: PLL lock timeout\n", sc->sc_dev.dv_xname);
}

void
smtcomphy_pll_init_usb3(struct smtcomphy_softc *sc)
{
	uint32_t val;

	val = HREAD4(sc, PCIE_PU_ADDR_CLK_CFG(0));
	val &= ~CFG_INTERNAL_TIMER_ADJ_MASK;
	val |= CFG_INTERNAL_TIMER_ADJ_USB3;
	HWRITE4(sc, PCIE_PU_ADDR_CLK_CFG(0), val);

	val = HREAD4(sc, PCIE_PU_PLL_1);
	val &= ~SSC_DEP_SEL_MASK;
	val |= SSC_DEP_SEL_5000PPM;
	HWRITE4(sc, PCIE_PU_PLL_1, val);

	smtcomphy_pll_init_common(sc);
}

int
smtcomphy_combo_enable(void *cookie, uint32_t *cells)
{
	struct smtcomphy_softc *sc = cookie;
	uint32_t type = cells[0];
	uint32_t val, oval;

	/* We only support PCIE and USB3. */
	if (type != PHY_TYPE_PCIE && type != PHY_TYPE_USB3)
		return EINVAL;

	/*
	 * Select the desired function; only change if not already set
	 * to the desired value.
	 */
	val = oval = regmap_read_4(sc->sc_apmu, APMU_PMUA_USB_PHY_CTRL0);
	if (type == PHY_TYPE_USB3)
		val |= APMU_COMBO_PHY_SEL;
	else
		val &= ~APMU_COMBO_PHY_SEL;
	if (val != oval)
		regmap_write_4(sc->sc_apmu, APMU_PMUA_USB_PHY_CTRL0, val);

	if (type == PHY_TYPE_PCIE)
		return smtcomphy_pcie_enable(cookie, cells);

	HWRITE4(sc, USB3_TEST_CTRL, 0);
	smtcomphy_pll_init_usb3(sc);

	return 0;
}

int
smtcomphy_pcie_enable(void *cookie, uint32_t *cells)
{
	/* No PCIe support yet. */
	return EINVAL;
}
