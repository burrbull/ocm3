
use crate::device::{RCC,FLASH};
use crate::rcc;
use self::rcc::{Rccf1};
//use crate::flash::{FlashLatency,Ocm3CommonFlash};


/* I2S3SRC: I2S3 clock source */
pub use crate::device::rcc::cfgr2::I2S3SRCW as I2S3Src;
/* I2S2SRC: I2S2 clock source */
pub use crate::device::rcc::cfgr2::I2S2SRCW as I2S2Src;

/* PREDIV1SRC: PREDIV1 entry clock source */
pub use crate::device::rcc::cfgr2::PREDIV1SRCW as Prediv1Src;
/* PLL3MUL: PLL3 multiplication factor */
pub use crate::device::rcc::cfgr2::PLL3MULW as Pll3Mul;
/* PLL2MUL: PLL2 multiplication factor */
pub use crate::device::rcc::cfgr2::PLL2MULW as Pll2Mul;
/* PREDIV: PREDIV division factor */
pub use crate::device::rcc::cfgr2::PREDIVW as Prediv1;
/* PREDIV2: PREDIV2 division factor */
pub use crate::device::rcc::cfgr2::PREDIV2W as Prediv2;

pub trait RccF1Cld {

	/// RCC Set the PLL2 Multiplication Factor.
	///
	/// This only has effect when the PLL is disabled.
	///
	/// * `mul : Pll2Mul` - PLL multiplication factor @ref rcc_cfgr_pmf
    fn set_pll2_multiplication_factor(&mut self, mul: Pll2Mul);
    
	/// RCC Set the PLL3 Multiplication Factor.
	///
	/// This only has effect when the PLL is disabled.
	///
	/// * `mul: Pll3Mul` - PLL multiplication factor @ref rcc_cfgr_pmf
    fn set_pll3_multiplication_factor(&mut self, mul: Pll3Mul);
    
    fn set_prediv1(&mut self, prediv: Prediv1);

    fn set_prediv2(&mut self, prediv: Prediv2);

    fn set_prediv1_source(&mut self, rccsrc: Prediv1Src);
}

trait RccCldSetup {
	/// RCC Set System Clock PLL at 72MHz from HSE at 25MHz
	fn clock_setup_in_hse_25mhz_out_72mhz(&mut self) -> Frequencies;
}

impl RccF1Cld for RCC {

    fn set_pll2_multiplication_factor(&mut self, mul: Pll2Mul) {
		self.cfgr     .modify(|_,w| w
			.pll2mul() .variant(mul)
		);
	}

    fn set_pll3_multiplication_factor(&mut self, mul: Pll3Mul) {
		self.cfgr     .modify(|_,w| w
			.pll3mul() .variant(mul)
		);
	}

    fn set_prediv1(&mut self, prediv: Prediv1) {
		self.cfgr2     .modify(|_,w| w
			.prediv()   .variant(prediv)
		);
	}

    fn set_prediv2(&mut self, prediv: Prediv2) {
		self.cfgr2     .modify(|_,w| w
			.prediv2()   .variant(prediv)
		);
	}

    fn set_prediv1_source(&mut self, rccsrc: Prediv1Src) {
		self.cfgr2     .modify(|_,w| w
			.prediv1src()   .variant(rccsrc)
		);
	}
}
/*
impl RccCldSetup for RCC {
    fn clock_setup_in_hse_25mhz_out_72mhz(&mut self) -> Frequencies {
		/* Enable external high-speed oscillator 25MHz. */
		self.osc_on(&Osc::HSE);
		self.wait_for_osc_ready(&Osc::HSE);
		self.set_sysclk_source(SysClk::HSE);

		/*
		 * Sysclk runs with 72MHz -> 2 waitstates.
		 * 0WS from 0-24MHz
		 * 1WS from 24-48MHz
		 * 2WS from 48-72MHz
		 */
		unsafe { (*FLASH.get()).set_ws(FlashLatency::TWO); }

		/*
		 * Set prescalers for AHB, ADC, ABP1, ABP2.
		 * Do this before touching the PLL (TODO: why?).
		 */
		self.set_hpre(HPre::DIV1);    /* Set. 72MHz Max. 72MHz */
		self.set_adcpre(AdcPre::DIV6);  /* Set. 12MHz Max. 14MHz */
		self.set_ppre1(PPre1::DIV2);     /* Set. 36MHz Max. 36MHz */
		self.set_ppre2(PPre2::DIV1);    /* Set. 72MHz Max. 72MHz */

		/* Set pll2 prediv and multiplier */
		self.set_prediv2(Prediv2::DIV5);
		self.set_pll2_multiplication_factor(Pll2Mul::MUL8);

		/* Enable PLL2 oscillator and wait for it to stabilize */
		self.osc_on(&Osc::PLL2);
		self.wait_for_osc_ready(&Osc::PLL2);

		/* Set pll1 prediv/multiplier, prediv1 src, and usb predivider */
		self.set_pllxtpre(PllXtPre::DIV1);
		self.set_prediv1_source(Prediv1Src::PLL2);
		self.set_prediv1(Prediv1::DIV5);
		self.set_pll_multiplication_factor(PllMul::MUL9);
		self.set_pll_source(PllSrc::PREDIV1);
		self.set_usbpre(UsbPre::DIV3);

		/* enable PLL1 and wait for it to stabilize */
		self.osc_on(&Osc::PLL);
		self.wait_for_osc_ready(&Osc::PLL);

		/* Select PLL as SYSCLK source. */
		self.set_sysclk_source(SysClk::PLL);

		/* Set the peripheral clock frequencies used */
		Frequencies {
			ahb:  72000000,
			apb1: 36000000,
			apb2: 72000000
		}
	}
}
*/
