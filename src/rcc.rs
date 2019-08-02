
use crate::device::RCC;
use crate::device::FLASH;
use crate::flash::{FlashLatency,FlashExt};

#[derive(Clone, Copy, PartialEq)]
pub enum Osc {
    PLL,
    /*PLL2,
    PLL3,*/
    HSE,
    HSI,
    LSE,
    LSI
}

/* USBPRE: USB prescaler (RCC_CFGR[22]) */
pub use crate::device::rcc::cfgr::USBPREW as UsbPre;
/* PLLMUL: PLL multiplication factor */
pub use crate::device::rcc::cfgr::PLLMULW as PllMul;
/* PLLXTPRE: HSE divider for PLL entry */
pub use crate::device::rcc::cfgr::PLLXTPREW as PllXtPre;
/* PLLSRC: PLL entry clock source */
pub use crate::device::rcc::cfgr::PLLSRCW as PllSrc;
/* ADCPRE: ADC prescaler */
pub use crate::device::rcc::cfgr::ADCPREW as AdcPre;
/* PPRE2: APB high-speed prescaler (APB2) */
pub use crate::device::rcc::cfgr::PPRE1W as PPre2;
/* PPRE1: APB low-speed prescaler (APB1) */
pub use crate::device::rcc::cfgr::PPRE1W as PPre1;
/* HPRE: AHB prescaler */
pub use crate::device::rcc::cfgr::HPREW as HPre;
/* SWS: System clock switch status */
pub use crate::device::rcc::cfgr::SWSR as SysClkStatus;
/* SW: System clock switch */
pub use crate::device::rcc::cfgr::SWW as SysClk;

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum Mco {
    NOCLK     = 0,
    SYSCLK    = 4,
    HSI       = 5,
    HSE       = 6,
    PLL_DIV2  = 7,
    PLL2      = 8,
    PLL3_DIV2 = 9,
    XT1       = 10,
    PLL3      = 11
}

#[derive(Clone, Copy, PartialEq)]
pub enum RtcSel {
    NOCLK     = 0,
    LSE       = 1,
    LSI       = 2,
    HSE       = 3
}




pub struct Frequencies {
    pub ahb: u32,
    pub apb1: u32,
    pub apb2: u32
}

/// Set the default clock frequencies
pub const RCC_FREQUENCIES : Frequencies = Frequencies {
    ahb:  8000000,
    apb1: 8000000,
    apb2: 8000000
};


pub trait RccF1Ext {
    /// RCC Clear the Oscillator Ready Interrupt Flag

    /// Clear the interrupt flag that was set when a clock oscillator became ready to
    /// use.
    /// 
    /// * `osc: Osc` - Oscillator ID
    fn osc_ready_int_clear(&mut self, osc: Osc);
    
    /// RCC Enable the Oscillator Ready Interrupt
    ///
    /// * `osc: Osc` - Oscillator ID
    
    fn osc_ready_int_enable(&mut self, osc: Osc);
    /// RCC Disable the Oscillator Ready Interrupt
    ///
    /// * `osc: Osc` - Oscillator ID
    
    fn osc_ready_int_disable(&mut self, osc: Osc);
    /// RCC Read the Oscillator Ready Interrupt Flag
    ///
    /// * `osc: Osc` - Oscillator ID
    ///
    /// Returns `bool` value for flag set.
    fn osc_ready_int_flag(&self, osc: Osc) -> bool;
    
    /// brief RCC Clear the Clock Security System Interrupt Flag
    fn css_int_clear(&mut self);
    
    /// RCC Read the Clock Security System Interrupt Flag
    ///
    /// Returns `bool` value for flag set.
    fn css_int_flag(&self) -> bool;
    
    fn is_osc_ready(&self, osc: Osc) -> bool;
    
    fn wait_for_osc_ready(&self, osc: Osc) {
        while !self.is_osc_ready(osc) {}
    }
    
    /// RCC Turn on an Oscillator.
    ///
    /// Enable an oscillator and power on. Each oscillator requires an amount of time
    /// to settle to a usable state. Refer to datasheets for time delay information. A
    /// status flag is available to indicate when the oscillator becomes ready (see
    /// @ref rcc_osc_ready_int_flag and @ref rcc_wait_for_osc_ready).
    ///
    /// **Note**: The LSE clock is in the backup domain and cannot be enabled until the
    /// backup domain write protection has been removed (see @ref
    /// pwr_disable_backup_domain_write_protect).
    ///
    /// * `osc: Osc` - Oscillator ID
    fn osc_on(&mut self, osc: Osc);
    
    /// RCC Turn off an Oscillator.
    ///
    /// Disable an oscillator and power off.
    ///
    /// **Note**: An oscillator cannot be turned off if it is selected as the system clock.
    ///
    /// **Note**: The LSE clock is in the backup domain and cannot be disabled until the
    /// backup domain write protection has been removed (see
    /// @ref pwr_disable_backup_domain_write_protect) or the backup domain has been
    /// (see reset @ref rcc_backupdomain_reset).
    ///
    /// * `osc: Osc` - Oscillator ID
    fn osc_off(&mut self, osc: Osc);

    /// RCC Enable the Clock Security System.
    fn css_enable(&mut self);
    
    /// RCC Disable the Clock Security System.
    fn css_disable(&mut self);
    
    /// RCC Set the Source for the System Clock.
    ///
    /// * `clk: SysClk` - System Clock Selection
    fn set_sysclk_source(&mut self, clk: SysClk);
    
    /// RCC Set the PLL Multiplication Factor.
    ///
    /// **Note**: This only has effect when the PLL is disabled.
    ///
    /// * `mul: PllMul` - PLL multiplication factor
    fn set_pll_multiplication_factor(&mut self, mul: PllMul);
    
    /// RCC Set the PLL Clock Source.
    ///
    /// **Note**: This only has effect when the PLL is disabled.
    ///
    /// * `pllsrc: PllSrc` - PLL clock source
    fn set_pll_source(&mut self, pllsrc: PllSrc);
    
    /// RCC Set the HSE Frequency Divider used as PLL Clock Source.
    ///
    /// **Note**: This only has effect when the PLL is disabled.
    ///
    /// * `pllxtpre: PllXtPre` - HSE division factor
    fn set_pllxtpre(&mut self, pllxtpre: PllXtPre);
    
    /// RCC RTC Clock Enabled Flag
    ///
    /// Returns `true` if the RTC Clock is enabled.
    fn rtc_clock_enabled_flag(&self) -> bool;
    
    /// RCC Enable the RTC clock
    fn enable_rtc_clock(&mut self);
    
    /// RCC Set the Source for the RTC clock
    ///
    /// * `clock_source: Osc` - RTC clock source. Only HSE/128, LSE and LSI.
    fn set_rtc_clock_source(&mut self, clock_source: Osc);
    
    /// ADC Setup the A/D Clock
    ///
    /// The ADC's have a common clock prescale setting.
    ///
    /// * `adcpre: AdcPre` - Prescale divider
    fn set_adcpre(&mut self, adcpre: AdcPre);
    
    /// RCC Set the APB2 Prescale Factor.
    ///
    /// * `ppre2: PPre2` - APB2 prescale factor
    fn set_ppre2(&mut self, ppre2: PPre2);
    
    /// RCC Set the APB1 Prescale Factor.
    ///
    /// **Note**: The APB1 clock frequency must not exceed 36MHz.
    ///
    /// * `ppre1: PPre1` - APB1 prescale factor
    fn set_ppre1(&mut self, ppre1: PPre1);
    
    /// RCC Set the AHB Prescale Factor.
    ///
    /// * `hpre: HPre` - AHB prescale factor
    fn set_hpre(&mut self, hpre: HPre);

    /// RCC Set the USB Prescale Factor.
    ///
    /// The prescale factor can be set to 1 (no prescale) for use when the PLL clock is
    /// 48MHz, or 1.5 to generate the 48MHz USB clock from a 64MHz PLL clock.
    ///
    /// **Note**: This bit cannot be reset while the USB clock is enabled.
    ///
    /// * `usbpre: UsbPre` - USB prescale factor
    fn set_usbpre(&mut self, usbpre: UsbPre);

    /// RCC Get the System Clock Source.
    ///
    /// Returns `SysClkStatus` - System clock source:
    /// * 00 indicates HSE
    /// * 01 indicates LSE
    /// * 02 indicates PLL
    fn system_clock_source(&mut self) -> SysClkStatus;
    
    /// RCC Reset the Backup Domain
    ///
    /// The backup domain registers are reset to disable RTC controls and clear user
    /// data.
    fn backupdomain_reset(&mut self);
}

impl RccF1Ext for RCC {

    fn osc_ready_int_clear(&mut self, osc: Osc) {
        let reg = &self.cir;
        match osc {
            Osc::PLL => { reg.modify(|_,w| w
                    .pllrdyc() .set_bit() );
            },
            /*
            Osc::PLL2 => { reg.modify(|_,w| w
                    .pll2rdyc() .set_bit() );
            },
            Osc::PLL3 => { reg.modify(|_,w| w
                    .pll3rdyc() .set_bit() );
            },
            */
            Osc::HSE => { reg.modify(|_,w| w
                    .hserdyc() .set_bit() );
            },
            Osc::HSI => { reg.modify(|_,w| w
                    .hsirdyc() .set_bit() );
            },
            Osc::LSE => { reg.modify(|_,w| w
                    .lserdyc() .set_bit() );
            },
            Osc::LSI => { reg.modify(|_,w| w
                    .lsirdyc() .set_bit() );
            }
        }
    }
    
    fn osc_ready_int_enable(&mut self, osc: Osc) {
        let reg = &self.cir;
        match osc {
            Osc::PLL => { reg.modify(|_,w| w
                    .pllrdyie() .set_bit() );
            },
            /*
            Osc::PLL2 => { reg.modify(|_,w| w
                    .pll2rdyie) .set_bit() );
            },
            Osc::PLL3 => { reg.modify(|_,w| w
                    .pll3rdyie() .set_bit() );
            },
            */
            Osc::HSE => { reg.modify(|_,w| w
                    .hserdyie() .set_bit() );
            },
            Osc::HSI => { reg.modify(|_,w| w
                    .hsirdyie() .set_bit() );
            },
            Osc::LSE => { reg.modify(|_,w| w
                    .lserdyie() .set_bit() );
            },
            Osc::LSI => { reg.modify(|_,w| w
                    .lsirdyie() .set_bit() );
            }
        }
    }
    
    fn osc_ready_int_disable(&mut self, osc: Osc) {
        let reg = &self.cir;
        match osc {
            Osc::PLL => { reg.modify(|_,w| w
                    .pllrdyie() .clear_bit() );
            },
            /*
            Osc::PLL2 => { reg.modify(|_,w| w
                    .pll2rdyie) .clear_bit() );
            },
            Osc::PLL3 => { reg.modify(|_,w| w
                    .pll3rdyie() .clear_bit() );
            },
            */
            Osc::HSE => { reg.modify(|_,w| w
                    .hserdyie() .clear_bit() );
            },
            Osc::HSI => { reg.modify(|_,w| w
                    .hsirdyie() .clear_bit() );
            },
            Osc::LSE => { reg.modify(|_,w| w
                    .lserdyie() .clear_bit() );
            },
            Osc::LSI => { reg.modify(|_,w| w
                    .lsirdyie() .clear_bit() );
            }
        }
    }

    
    fn osc_ready_int_flag(&self, osc: Osc) -> bool {
        let reg = &self.cir;
        match osc {
            Osc::PLL  => reg .read() .pllrdyf() .bit_is_set(),
            /*
            Osc::PLL2 => reg .read() .pll2rdyf() .bit_is_set(),
            Osc::PLL3 => reg .read() .pll3rdyf() .bit_is_set(),
            */
            Osc::HSE  => reg .read() .hserdyf() .bit_is_set(),
            Osc::HSI  => reg .read() .hsirdyf() .bit_is_set(),
            Osc::LSE  => reg .read() .lserdyf() .bit_is_set(),
            Osc::LSI  => reg .read() .lsirdyf() .bit_is_set()
        }
    }

    fn css_int_clear(&mut self) {
        self.cir       .modify(|_,w| w
            .cssc() .set_bit()
        );
    }
    
    fn css_int_flag(&self) -> bool {
        self.cir .read() .cssf() .bit_is_set()
    }
    
    fn is_osc_ready(&self, osc: Osc) -> bool {
        match osc {
            Osc::PLL  => self.cr .read() .pllrdy() .bit_is_set(),
            /*
            Osc::PLL2 => self.cr .read() .pll2rdy() .bit_is_set(),
            Osc::PLL3 => self.cr .read() .pll3rdy() .bit_is_set(),
            */
            Osc::HSE  => self.cr .read() .hserdy() .bit_is_set(),
            Osc::HSI  => self.cr .read() .hsirdy() .bit_is_set(),
            Osc::LSE  => self.bdcr.read().lserdy() .bit_is_set(),
            Osc::LSI  => self.csr .read().lsirdy() .bit_is_set()
        }
    }
    
    fn osc_on(&mut self, osc: Osc) {
        match osc {
            Osc::PLL => { self.cr.modify(|_,w| w
                    .pllon() .set_bit() );
            },
            /*
            Osc::PLL2 => { self.cr.modify(|_,w| w
                    .pll2on) .set_bit() );
            },
            Osc::PLL3 => { self.cr.modify(|_,w| w
                    .pll3on() .set_bit() );
            },
            */
            Osc::HSE => { self.cr .modify(|_,w| w
                    .hseon() .set_bit() );
            },
            Osc::HSI => { self.cr .modify(|_,w| w
                    .hsion() .set_bit() );
            },
            Osc::LSE => { self.bdcr.modify(|_,w| w
                    .lseon() .set_bit() );
            },
            Osc::LSI => { self.csr.modify(|_,w| w
                    .lsion() .set_bit() );
            }
        }
    }
    
    fn osc_off(&mut self, osc: Osc) {
        match osc {
            Osc::PLL => { self.cr.modify(|_,w| w
                    .pllon() .clear_bit() );
            },
            /*
            Osc::PLL2 => { self.cr.modify(|_,w| w
                    .pll2on) .clear_bit() );
            },
            Osc::PLL3 => { self.cr.modify(|_,w| w
                    .pll3on() .clear_bit() );
            },
            */
            Osc::HSE => { self.cr .modify(|_,w| w
                    .hseon() .clear_bit() );
            },
            Osc::HSI => { self.cr .modify(|_,w| w
                    .hsion() .clear_bit() );
            },
            Osc::LSE => { self.bdcr.modify(|_,w| w
                    .lseon() .clear_bit() );
            },
            Osc::LSI => { self.csr.modify(|_,w| w
                    .lsion() .clear_bit() );
            }
        }
    }
    
    fn css_enable(&mut self) {
        self.cr      .modify(|_,w| w
            .csson() .set_bit()
        );
    }

    fn css_disable(&mut self) {
        self.cr      .modify(|_,w| w
            .csson() .clear_bit()
        );
    }
    

    fn set_sysclk_source(&mut self, clk: SysClk) {
        self.cfgr   .modify(|_,w| w
            .sw()   .variant(clk)
        );
    }


    fn set_pll_multiplication_factor(&mut self, mul: PllMul) {
        self.cfgr     .modify(|_,w| w
            .pllmul() .variant(mul)
        );
    }

    fn set_pll_source(&mut self, pllsrc: PllSrc) {
        self.cfgr     .modify(|_,w| w
            .pllsrc() .variant(pllsrc)
        );
    }


    fn set_pllxtpre(&mut self, pllxtpre: PllXtPre) {
        self.cfgr     .modify(|_,w| w
            .pllxtpre() .variant(pllxtpre)
        );
    }

    fn rtc_clock_enabled_flag(&self) -> bool {
        self.bdcr .read() .rtcen() .bit_is_set()
    }

    fn enable_rtc_clock(&mut self) {
        self.bdcr     .modify(|_,w| w
            .rtcen()  .set_bit()
        );
    }

    fn set_rtc_clock_source(&mut self, clock_source: Osc) {
        match clock_source {
            Osc::LSE => {
                /* Turn the LSE on and wait while it stabilises. */
                self.bdcr      .modify(|_,w| w
                    .lseon()   .set_bit()
                );
                while self.bdcr .read() .lserdy().bit_is_clear() {}
                
                /* Choose LSE as the RTC clock source. */
                self.bdcr    .modify(|_,w| w
                    .rtcsel()   .bits(RtcSel::LSE as u8)
                );
            }
            Osc::LSI => {
                /* Turn the LSI on and wait while it stabilises. */
                self.csr       .modify(|_,w| w
                    .lsion()   .set_bit()
                );
                while self.csr .read() .lsirdy().bit_is_clear() {}

                /* Choose LSI as the RTC clock source. */
                self.bdcr    .modify(|_,w| w
                    .rtcsel()   .bits(RtcSel::LSI as u8)
                );
            }
            Osc::HSE => {
                /* Turn the HSE on and wait while it stabilises. */
                self.cr        .modify(|_,w| w
                    .hseon()   .set_bit()
                );
                while self.cr .read() .hserdy().bit_is_clear() {}

                /* Choose HSE as the RTC clock source. */
                self.bdcr    .modify(|_,w| w
                    .rtcsel()   .bits(RtcSel::HSE as u8)
                );
            }
            Osc::HSI => {
                /* Unusable clock source, here to prevent warnings. */
                /* Turn off clock sources to RTC. */
                self.bdcr    .modify(|_,w| w
                    .rtcsel()   .bits(RtcSel::NOCLK as u8)
                );
            }
            _ => {}
        }
    }

    fn set_adcpre(&mut self, adcpre: AdcPre) {
        self.cfgr     .modify(|_,w| w
            .adcpre() .variant(adcpre)
        );
    }

    fn set_ppre2(&mut self, ppre2: PPre2) {
        self.cfgr     .modify(|_,w| w
            .ppre2()  .variant(ppre2)
        );
    }


    fn set_ppre1(&mut self, ppre1: PPre1) {
        self.cfgr     .modify(|_,w| w
            .ppre1()  .variant(ppre1)
        );
    }

    fn set_hpre(&mut self, hpre: HPre) {
        self.cfgr     .modify(|_,w| w
            .hpre()   .variant(hpre)
        );
    }

    fn set_usbpre(&mut self, usbpre: UsbPre) {
        self.cfgr     .modify(|_,w| w
            .usbpre()   .variant(usbpre)
        );
    }

    fn system_clock_source(&mut self) -> SysClkStatus {
        /* Return the clock source which is used as system clock. */
        self.cfgr.  read(). sws()
    }

    fn backupdomain_reset(&mut self) {
        /* Set the backup domain software reset. */
        self.bdcr   .modify(|_,w| w
            .bdrst()   .set_bit()
        );

        /* Clear the backup domain software reset. */
        self.bdcr   .modify(|_,w| w
            .bdrst()   .clear_bit()
        );
    }


}




/// These functions are setting up the whole clock system for the most common
/// input clock and output clock configurations
pub trait RccSetup {
    /*/// RCC Set System Clock PLL at 64MHz from HSI
    fn clock_setup_in_hsi_out_64mhz(&mut self, flash : &mut FLASH) -> Frequencies;
    */
    /*/// RCC Set System Clock PLL at 48MHz from HSI
    fn clock_setup_in_hsi_out_48mhz(&mut self, flash : &mut FLASH) -> Frequencies;
    */
    /// RCC Set System Clock PLL at 24MHz from HSI
    fn clock_setup_in_hsi_out_24mhz(&mut self, flash : &mut FLASH) -> Frequencies;
    
    /*/// RCC Set System Clock PLL at 24MHz from HSE at 8MHz
    fn clock_setup_in_hse_8mhz_out_24mhz(&mut self, flash : &mut FLASH) -> Frequencies;
    */
    /// RCC Set System Clock PLL at 72MHz from HSE at 8MHz
    fn clock_setup_in_hse_8mhz_out_72mhz(&mut self, flash : &mut FLASH) -> Frequencies;
    
    /// RCC Set System Clock PLL at 72MHz from HSE at 12MHz
    fn clock_setup_in_hse_12mhz_out_72mhz(&mut self, flash : &mut FLASH) -> Frequencies;
    
    /// RCC Set System Clock PLL at 72MHz from HSE at 16MHz
    fn clock_setup_in_hse_16mhz_out_72mhz(&mut self, flash : &mut FLASH) -> Frequencies;
}

impl RccSetup for RCC {
    /*fn clock_setup_in_hsi_out_64mhz(&mut self, flash : &mut FLASH) -> Frequencies {
        /* Enable internal high-speed oscillator. */
        self.osc_on(Osc::HSI);
        self.wait_for_osc_ready(Osc::HSI);

        /* Select HSI as SYSCLK source. */
        self.set_sysclk_source(SysClk::HSI);

        /*
         * Set prescalers for AHB, ADC, ABP1, ABP2.
         * Do this before touching the PLL (TODO: why?).
         */
        self.set_hpre(HPre::DIV1);   /* Set. 64MHz Max. 72MHz */
        self.set_adcpre(AdcPre::DIV8); /* Set.  8MHz Max. 14MHz */
        self.set_ppre1(PPre1::DIV2);    /* Set. 32MHz Max. 36MHz */
        self.set_ppre2(PPre2::DIV1);   /* Set. 64MHz Max. 72MHz */

        /*
         * Sysclk is running with 64MHz -> 2 waitstates.
         * 0WS from 0-24MHz
         * 1WS from 24-48MHz
         * 2WS from 48-72MHz
         */
        flash.set_ws(FlashLatency::TWO);

        /*
         * Set the PLL multiplication factor to 16.
         * 8MHz (internal) * 16 (multiplier) / 2 (PLLSRC_HSI_CLK_DIV2) = 64MHz
         */
        self.set_pll_multiplication_factor(PllMul::MUL16);

        /* Select HSI/2 as PLL source. */
        self.set_pll_source(PllSrc::HSI_DIV2);

        /* Enable PLL oscillator and wait for it to stabilize. */
        self.osc_on(Osc::PLL);
        self.wait_for_osc_ready(Osc::PLL);

        /* Select PLL as SYSCLK source. */
        self.set_sysclk_source(SysClk::PLL);

        /* Set the peripheral clock frequencies used */
        Frequencies {
            ahb:  64000000,
            apb1: 32000000,
            apb2: 64000000
        }
    }*/

    /*fn clock_setup_in_hsi_out_48mhz(&mut self, flash : &mut FLASH) -> Frequencies {
        /* Enable internal high-speed oscillator. */
        self.osc_on(Osc::HSI);
        self.wait_for_osc_ready(Osc::HSI);

        /* Select HSI as SYSCLK source. */
        self.set_sysclk_source(SysClk::HSI);

        /*
         * Set prescalers for AHB, ADC, ABP1, ABP2.
         * Do this before touching the PLL (TODO: why?).
         */
        self.set_hpre(HPre::DIV1);    /*Set.48MHz Max.72MHz */
        self.set_adcpre(AdcPre::DIV8);    /*Set. 6MHz Max.14MHz */
        self.set_ppre1(PPre1::DIV2);    /*Set.24MHz Max.36MHz */
        self.set_ppre2(PPre2::DIV1);    /*Set.48MHz Max.72MHz */
        self.set_usbpre(UsbPre::DIV1);  /*Set.48MHz Max.48MHz */

        /*
         * Sysclk runs with 48MHz -> 1 waitstates.
         * 0WS from 0-24MHz
         * 1WS from 24-48MHz
         * 2WS from 48-72MHz
         */
        flash.set_ws(FlashLatency::ONE);

        /*
         * Set the PLL multiplication factor to 12.
         * 8MHz (internal) * 12 (multiplier) / 2 (PLLSRC_HSI_CLK_DIV2) = 48MHz
         */
        self.set_pll_multiplication_factor(PllMul::MUL12);

        /* Select HSI/2 as PLL source. */
        self.set_pll_source(PllSrc::HSI_DIV2);

        /* Enable PLL oscillator and wait for it to stabilize. */
        self.osc_on(Osc::PLL);
        self.wait_for_osc_ready(Osc::PLL);

        /* Select PLL as SYSCLK source. */
        self.set_sysclk_source(SysClk::PLL);

        /* Set the peripheral clock frequencies used */
        Frequencies {
            ahb:  48000000,
            apb1: 24000000,
            apb2: 48000000
        }
    }*/

    fn clock_setup_in_hsi_out_24mhz(&mut self, flash : &mut FLASH) -> Frequencies {
        /* Enable internal high-speed oscillator. */
        self.osc_on(Osc::HSI);
        self.wait_for_osc_ready(Osc::HSI);

        /* Select HSI as SYSCLK source. */
        self.set_sysclk_source(SysClk::HSI);

        /*
         * Set prescalers for AHB, ADC, ABP1, ABP2.
         * Do this before touching the PLL (TODO: why?).
         */
        self.set_hpre(HPre::DIV1); /* Set. 24MHz Max. 24MHz */
        self.set_adcpre(AdcPre::DIV2); /* Set. 12MHz Max. 12MHz */
        self.set_ppre1(PPre1::DIV1); /* Set. 24MHz Max. 24MHz */
        self.set_ppre2(PPre2::DIV1); /* Set. 24MHz Max. 24MHz */

        /*
         * Sysclk is (will be) running with 24MHz -> 2 waitstates.
         * 0WS from 0-24MHz
         * 1WS from 24-48MHz
         * 2WS from 48-72MHz
         */
        flash.set_ws(FlashLatency::WS0);

        /*
         * Set the PLL multiplication factor to 6.
         * 8MHz (internal) * 6 (multiplier) / 2 (PLLSRC_HSI_CLK_DIV2) = 24MHz
         */
        self.set_pll_multiplication_factor(PllMul::MUL6);

        /* Select HSI/2 as PLL source. */
        self.set_pll_source(PllSrc::HSI_DIV2);

        /* Enable PLL oscillator and wait for it to stabilize. */
        self.osc_on(Osc::PLL);
        self.wait_for_osc_ready(Osc::PLL);

        /* Select PLL as SYSCLK source. */
        self.set_sysclk_source(SysClk::PLL);

        /* Set the peripheral clock frequencies used */
        Frequencies {
            ahb:  24000000,
            apb1: 24000000,
            apb2: 24000000
        }
    }

    /*fn clock_setup_in_hse_8mhz_out_24mhz(&mut self, flash : &mut FLASH) -> Frequencies {
        /* Enable internal high-speed oscillator. */
        self.osc_on(Osc::HSI);
        self.wait_for_osc_ready(Osc::HSI);

        /* Select HSI as SYSCLK source. */
        self.set_sysclk_source(SysClk::HSI);

        /* Enable external high-speed oscillator 8MHz. */
        self.osc_on(Osc::HSE);
        self.wait_for_osc_ready(Osc::HSE);
        self.set_sysclk_source(SysClk::HSE);

        /*
         * Set prescalers for AHB, ADC, ABP1, ABP2.
         * Do this before touching the PLL (TODO: why?).
         */
        self.set_hpre(HPre::DIV1);    /* Set. 24MHz Max. 72MHz */
        self.set_adcpre(AdcPre::DIV2);  /* Set. 12MHz Max. 14MHz */
        self.set_ppre1(PPre1::DIV1);    /* Set. 24MHz Max. 36MHz */
        self.set_ppre2(PPre2::DIV1);    /* Set. 24MHz Max. 72MHz */

        /*
         * Sysclk runs with 24MHz -> 0 waitstates.
         * 0WS from 0-24MHz
         * 1WS from 24-48MHz
         * 2WS from 48-72MHz
         */
        flash.set_ws(FlashLatency::ZERO);

        /*
         * Set the PLL multiplication factor to 3.
         * 8MHz (external) * 3 (multiplier) = 24MHz
         */
        self.set_pll_multiplication_factor(PllMul::MUL3);

        /* Select HSE as PLL source. */
        self.set_pll_source(PllSrc::HSE_DIV_PREDIV);

        /*
         * External frequency undivided before entering PLL
         * (only valid/needed for HSE).
         */
        self.set_pllxtpre(PllXtPre::DIV1);

        /* Enable PLL oscillator and wait for it to stabilize. */
        self.osc_on(Osc::PLL);
        self.wait_for_osc_ready(Osc::PLL);

        /* Select PLL as SYSCLK source. */
        self.set_sysclk_source(SysClk::PLL);

        /* Set the peripheral clock frequencies used */
        Frequencies {
            ahb:  24000000,
            apb1: 24000000,
            apb2: 24000000
        }
    }*/

    fn clock_setup_in_hse_8mhz_out_72mhz(&mut self, flash : &mut FLASH) -> Frequencies {
        /* Enable internal high-speed oscillator. */
        self.osc_on(Osc::HSI);
        self.wait_for_osc_ready(Osc::HSI);

        /* Select HSI as SYSCLK source. */
        self.set_sysclk_source(SysClk::HSI);

        /* Enable external high-speed oscillator 8MHz. */
        self.osc_on(Osc::HSE);
        self.wait_for_osc_ready(Osc::HSE);
        self.set_sysclk_source(SysClk::HSE);

        /*
         * Set prescalers for AHB, ADC, ABP1, ABP2.
         * Do this before touching the PLL (TODO: why?).
         */
        self.set_hpre(HPre::DIV1);    /* Set. 72MHz Max. 72MHz */
        self.set_adcpre(AdcPre::DIV8);  /* Set.  9MHz Max. 14MHz */
        self.set_ppre1(PPre1::DIV2);     /* Set. 36MHz Max. 36MHz */
        self.set_ppre2(PPre2::DIV1);    /* Set. 72MHz Max. 72MHz */

        /*
         * Sysclk runs with 72MHz -> 2 waitstates.
         * 0WS from 0-24MHz
         * 1WS from 24-48MHz
         * 2WS from 48-72MHz
         */
        flash.set_ws(FlashLatency::WS2);

        /*
         * Set the PLL multiplication factor to 9.
         * 8MHz (external) * 9 (multiplier) = 72MHz
         */
        self.set_pll_multiplication_factor(PllMul::MUL9);

        /* Select HSE as PLL source. */
        self.set_pll_source(PllSrc::HSE_DIV_PREDIV);

        /*
         * External frequency undivided before entering PLL
         * (only valid/needed for HSE).
         */
        self.set_pllxtpre(PllXtPre::DIV1);

        /* Enable PLL oscillator and wait for it to stabilize. */
        self.osc_on(Osc::PLL);
        self.wait_for_osc_ready(Osc::PLL);

        /* Select PLL as SYSCLK source. */
        self.set_sysclk_source(SysClk::PLL);

        /* Set the peripheral clock frequencies used */
        Frequencies {
            ahb:  72000000,
            apb1: 36000000,
            apb2: 72000000
        }
    }

    fn clock_setup_in_hse_12mhz_out_72mhz(&mut self, flash : &mut FLASH) -> Frequencies {
        /* Enable internal high-speed oscillator. */
        self.osc_on(Osc::HSI);
        self.wait_for_osc_ready(Osc::HSI);

        /* Select HSI as SYSCLK source. */
        self.set_sysclk_source(SysClk::HSI);

        /* Enable external high-speed oscillator 16MHz. */
        self.osc_on(Osc::HSE);
        self.wait_for_osc_ready(Osc::HSE);
        self.set_sysclk_source(SysClk::HSE);

        /*
         * Set prescalers for AHB, ADC, ABP1, ABP2.
         * Do this before touching the PLL (TODO: why?).
         */
        self.set_hpre(HPre::DIV1);    /* Set. 72MHz Max. 72MHz */
        self.set_adcpre(AdcPre::DIV6);  /* Set. 12MHz Max. 14MHz */
        self.set_ppre1(PPre1::DIV2);     /* Set. 36MHz Max. 36MHz */
        self.set_ppre2(PPre2::DIV1);    /* Set. 72MHz Max. 72MHz */

        /*
         * Sysclk runs with 72MHz -> 2 waitstates.
         * 0WS from 0-24MHz
         * 1WS from 24-48MHz
         * 2WS from 48-72MHz
         */
        flash.set_ws(FlashLatency::WS2);

        /*
         * Set the PLL multiplication factor to 9.
         * 12MHz (external) * 6 (multiplier) / 1 (PLLXTPRE_HSE_CLK) = 72MHz
         */
        self.set_pll_multiplication_factor(PllMul::MUL6);

        /* Select HSI as PLL source. */
        self.set_pll_source(PllSrc::HSE_DIV_PREDIV);

        /*
         * Divide external frequency by 2 before entering PLL
         * (only valid/needed for HSE).
         */
        self.set_pllxtpre(PllXtPre::DIV1);

        /* Enable PLL oscillator and wait for it to stabilize. */
        self.osc_on(Osc::PLL);
        self.wait_for_osc_ready(Osc::PLL);

        /* Select PLL as SYSCLK source. */
        self.set_sysclk_source(SysClk::PLL);

        /* Set the peripheral clock frequencies used */
        Frequencies {
            ahb:  72000000,
            apb1: 36000000,
            apb2: 72000000
        }
    }

    fn clock_setup_in_hse_16mhz_out_72mhz(&mut self, flash : &mut FLASH) -> Frequencies {
        /* Enable internal high-speed oscillator. */
        self.osc_on(Osc::HSI);
        self.wait_for_osc_ready(Osc::HSI);

        /* Select HSI as SYSCLK source. */
        self.set_sysclk_source(SysClk::HSI);

        /* Enable external high-speed oscillator 16MHz. */
        self.osc_on(Osc::HSE);
        self.wait_for_osc_ready(Osc::HSE);
        self.set_sysclk_source(SysClk::HSE);

        /*
         * Set prescalers for AHB, ADC, ABP1, ABP2.
         * Do this before touching the PLL (TODO: why?).
         */
        self.set_hpre(HPre::DIV1);    /* Set. 72MHz Max. 72MHz */
        self.set_adcpre(AdcPre::DIV6);  /* Set. 12MHz Max. 14MHz */
        self.set_ppre1(PPre1::DIV2);     /* Set. 36MHz Max. 36MHz */
        self.set_ppre2(PPre2::DIV1);    /* Set. 72MHz Max. 72MHz */

        /*
         * Sysclk runs with 72MHz -> 2 waitstates.
         * 0WS from 0-24MHz
         * 1WS from 24-48MHz
         * 2WS from 48-72MHz
         */
        flash.set_ws(FlashLatency::WS2);

        /*
         * Set the PLL multiplication factor to 9.
         * 16MHz (external) * 9 (multiplier) / 2 (PLLXTPRE_HSE_CLK_DIV2) = 72MHz
         */
        self.set_pll_multiplication_factor(PllMul::MUL9);

        /* Select HSI as PLL source. */
        self.set_pll_source(PllSrc::HSE_DIV_PREDIV);

        /*
         * Divide external frequency by 2 before entering PLL
         * (only valid/needed for HSE).
         */
        self.set_pllxtpre(PllXtPre::DIV2);

        /* Enable PLL oscillator and wait for it to stabilize. */
        self.osc_on(Osc::PLL);
        self.wait_for_osc_ready(Osc::PLL);

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

#[derive(Clone, Copy, PartialEq)]
pub enum Per {
    Ahb(u8),
    Apb2(u8),
    Apb1(u8)
}

pub mod en {
    use super::Per;
    /* AHB peripherals */
    pub const DMA1    : Per = Per::Ahb(0);/*VNC*/
    pub const DMA2    : Per = Per::Ahb(1);/*VNC*/
    pub const SRAM    : Per = Per::Ahb(2);/*VNC*/
    pub const FLTF    : Per = Per::Ahb(4);/*VNC*/
    pub const CRC     : Per = Per::Ahb(6);/*VNC*/
    pub const FSMC    : Per = Per::Ahb(8);/*VN-*/
    pub const SDIO    : Per = Per::Ahb(10);/*-N-*/
    pub const OTGFS   : Per = Per::Ahb(12);/*--C*/
    pub const ETHMAC  : Per = Per::Ahb(14);/*--C*/
    pub const ETHMACTX: Per = Per::Ahb(15);/*--C*/
    pub const ETHMACRX: Per = Per::Ahb(16);/*--C*/

    /* APB2 peripherals */
    pub const AFIO  : Per = Per::Apb2(0);/*VNC*/
    pub const GPIOA : Per = Per::Apb2(2);/*VNC*/
    pub const GPIOB : Per = Per::Apb2(3);/*VNC*/
    pub const GPIOC : Per = Per::Apb2(4);/*VNC*/
    pub const GPIOD : Per = Per::Apb2(5);/*VNC*/
    pub const GPIOE : Per = Per::Apb2(6);/*VNC*/
    pub const GPIOF : Per = Per::Apb2(7);/*VN-*/
    pub const GPIOG : Per = Per::Apb2(8);/*VN-*/
    pub const ADC1  : Per = Per::Apb2(9);/*VNC*/
    pub const ADC2  : Per = Per::Apb2(10);/*-NC*/
    pub const TIM1  : Per = Per::Apb2(11);/*VNC*/
    pub const SPI1  : Per = Per::Apb2(12);/*VNC*/
    pub const TIM8  : Per = Per::Apb2(13);/*-N-*/
    pub const USART1: Per = Per::Apb2(14);/*VNC*/
    pub const ADC3  : Per = Per::Apb2(15);/*-N-*/
    pub const TIM15 : Per = Per::Apb2(16);/*V--*/
    pub const TIM16 : Per = Per::Apb2(17);/*V--*/
    pub const TIM17 : Per = Per::Apb2(18);/*V--*/
    pub const TIM9  : Per = Per::Apb2(19);/*-N-*/
    pub const TIM10 : Per = Per::Apb2(20);/*-N-*/
    pub const TIM11 : Per = Per::Apb2(21);/*-N-*/

    /* APB1 peripherals */
    pub const TIM2  : Per = Per::Apb1(0);/*VNC*/
    pub const TIM3  : Per = Per::Apb1(1);/*VNC*/
    pub const TIM4  : Per = Per::Apb1(2);/*VNC*/
    pub const TIM5  : Per = Per::Apb1(3);/*VNC*/
    pub const TIM6  : Per = Per::Apb1(4);/*VNC*/
    pub const TIM7  : Per = Per::Apb1(5);/*VNC*/
    pub const TIM12 : Per = Per::Apb1(6);/*VN-*/
    pub const TIM13 : Per = Per::Apb1(7);/*VN-*/
    pub const TIM14 : Per = Per::Apb1(8);/*VN-*/
    pub const WWDG  : Per = Per::Apb1(11);/*VNC*/
    pub const SPI2  : Per = Per::Apb1(14);/*VNC*/
    pub const SPI3  : Per = Per::Apb1(15);/*VNC*/
    pub const USART2: Per = Per::Apb1(17);/*VNC*/
    pub const USART3: Per = Per::Apb1(18);/*VNC*/
    pub const UART4 : Per = Per::Apb1(19);/*VNC*/
    pub const UART5 : Per = Per::Apb1(20);/*VNC*/
    pub const I2C1  : Per = Per::Apb1(21);/*VNC*/
    pub const I2C2  : Per = Per::Apb1(22);/*VNC*/
    pub const USB   : Per = Per::Apb1(23);/*-N-*/
    pub const CAN   : Per = Per::Apb1(25);/*-N-*/
    pub const CAN1  : Per = Per::Apb1(25);/*--C*/
    pub const CAN2  : Per = Per::Apb1(26);/*--C*/
    pub const BKP   : Per = Per::Apb1(27);/*VNC*/
    pub const PWR   : Per = Per::Apb1(28);/*VNC*/
    pub const DAC   : Per = Per::Apb1(29);/*VNC*/
    pub const CEC   : Per = Per::Apb1(30);/*V--*/
}

pub mod rst {
    use super::Per;
    /* AHB peripherals */
    pub const OTGFS : Per = Per::Ahb(12);/*--C*/
    pub const ETHMAC: Per = Per::Ahb(14);/*--C*/

    /* APB2 peripherals */
    pub const AFIO  : Per = Per::Apb2(0);/*VNC*/
    pub const GPIOA : Per = Per::Apb2(2);/*VNC*/
    pub const GPIOB : Per = Per::Apb2(3);/*VNC*/
    pub const GPIOC : Per = Per::Apb2(4);/*VNC*/
    pub const GPIOD : Per = Per::Apb2(5);/*VNC*/
    pub const GPIOE : Per = Per::Apb2(6);/*VNC*/
    pub const GPIOF : Per = Per::Apb2(7);/*VN-*/
    pub const GPIOG : Per = Per::Apb2(8);/*VN-*/
    pub const ADC1  : Per = Per::Apb2(9);/*VNC*/
    pub const ADC2  : Per = Per::Apb2(10);/*-NC*/
    pub const TIM1  : Per = Per::Apb2(11);/*VNC*/
    pub const SPI1  : Per = Per::Apb2(12);/*VNC*/
    pub const TIM8  : Per = Per::Apb2(13);/*-N-*/
    pub const USART1: Per = Per::Apb2(14);/*VNC*/
    pub const ADC3  : Per = Per::Apb2(15);/*-N-*/
    pub const TIM15 : Per = Per::Apb2(16);/*V--*/
    pub const TIM16 : Per = Per::Apb2(17);/*V--*/
    pub const TIM17 : Per = Per::Apb2(18);/*V--*/
    pub const TIM9  : Per = Per::Apb2(19);/*-N-*/
    pub const TIM10 : Per = Per::Apb2(20);/*-N-*/
    pub const TIM11 : Per = Per::Apb2(21);/*-N-*/

    /* APB1 peripherals */
    pub const TIM2  : Per = Per::Apb1(0);/*VNC*/
    pub const TIM3  : Per = Per::Apb1(1);/*VNC*/
    pub const TIM4  : Per = Per::Apb1(2);/*VNC*/
    pub const TIM5  : Per = Per::Apb1(3);/*VNC*/
    pub const TIM6  : Per = Per::Apb1(4);/*VNC*/
    pub const TIM7  : Per = Per::Apb1(5);/*VNC*/
    pub const TIM12 : Per = Per::Apb1(6);/*VN-*/
    pub const TIM13 : Per = Per::Apb1(7);/*VN-*/
    pub const TIM14 : Per = Per::Apb1(8);/*VN-*/
    pub const WWDG  : Per = Per::Apb1(11);/*VNC*/
    pub const SPI2  : Per = Per::Apb1(14);/*VNC*/
    pub const SPI3  : Per = Per::Apb1(15);/*VNC*/
    pub const USART2: Per = Per::Apb1(17);/*VNC*/
    pub const USART3: Per = Per::Apb1(18);/*VNC*/
    pub const UART4 : Per = Per::Apb1(19);/*VNC*/
    pub const UART5 : Per = Per::Apb1(20);/*VNC*/
    pub const I2C1  : Per = Per::Apb1(21);/*VNC*/
    pub const I2C2  : Per = Per::Apb1(22);/*VNC*/
    pub const USB   : Per = Per::Apb1(23);/*-N-*/
    pub const CAN   : Per = Per::Apb1(24);/*-N-*/
    pub const CAN1  : Per = Per::Apb1(24);/*--C*/
    pub const CAN2  : Per = Per::Apb1(25);/*--C*/
    pub const BKP   : Per = Per::Apb1(27);/*VNC*/
    pub const PWR   : Per = Per::Apb1(28);/*VNC*/
    pub const DAC   : Per = Per::Apb1(29);/*VNC*/
    pub const CEC   : Per = Per::Apb1(30);/*V--*/
}

bitortype!(AhbEn, u32);
bitortype!(Apb2En, u32);
bitortype!(Apb1En, u32);

pub enum Enr {
    Ahb(AhbEn),
    Apb2(Apb2En),
    Apb1(Apb1En),
}

pub mod enr {
    // RCC_AHBENR values
    use super::AhbEn;
    pub const ETHMACENRX: AhbEn = AhbEn(1 << 16);
    pub const ETHMACENTX: AhbEn = AhbEn(1 << 15);
    pub const ETHMACEN: AhbEn = AhbEn(1 << 14);
    pub const OTGFSEN : AhbEn = AhbEn(1 << 12);
    pub const SDIOEN  : AhbEn = AhbEn(1 << 10);
    pub const FSMCEN  : AhbEn = AhbEn(1 << 8);
    pub const CRCEN   : AhbEn = AhbEn(1 << 6);
    pub const FLITFEN : AhbEn = AhbEn(1 << 4);
    pub const SRAMEN  : AhbEn = AhbEn(1 << 2);
    pub const DMA2EN  : AhbEn = AhbEn(1 << 1);
    pub const DMA1EN  : AhbEn = AhbEn(1 << 0);
    // RCC_APB2ENR values
    use super::Apb2En;
    pub const TIM17EN : Apb2En = Apb2En(1 << 18);
    pub const TIM16EN : Apb2En = Apb2En(1 << 17);
    pub const TIM15EN : Apb2En = Apb2En(1 << 16);
    pub const ADC3EN  : Apb2En = Apb2En(1 << 15); /* (XX) */
    pub const USART1EN: Apb2En = Apb2En(1 << 14);
    pub const TIM8EN  : Apb2En = Apb2En(1 << 13); /* (XX) */
    pub const SPI1EN  : Apb2En = Apb2En(1 << 12);
    pub const TIM1EN  : Apb2En = Apb2En(1 << 11);
    pub const ADC2EN  : Apb2En = Apb2En(1 << 10);
    pub const ADC1EN  : Apb2En = Apb2En(1 << 9);
    pub const IOPGEN  : Apb2En = Apb2En(1 << 8);  /* (XX) */
    pub const IOPFEN  : Apb2En = Apb2En(1 << 7);  /* (XX) */
    pub const IOPEEN  : Apb2En = Apb2En(1 << 6);
    pub const IOPDEN  : Apb2En = Apb2En(1 << 5);
    pub const IOPCEN  : Apb2En = Apb2En(1 << 4);
    pub const IOPBEN  : Apb2En = Apb2En(1 << 3);
    pub const IOPAEN  : Apb2En = Apb2En(1 << 2);
    pub const AFIOEN  : Apb2En = Apb2En(1 << 0);
    // RCC_APB1ENR values
    use super::Apb1En;
    pub const DACEN   : Apb1En = Apb1En(1 << 29);
    pub const PWREN   : Apb1En = Apb1En(1 << 28);
    pub const BKPEN   : Apb1En = Apb1En(1 << 27);
    pub const CAN2EN  : Apb1En = Apb1En(1 << 26); /* (**) */
    pub const CAN1EN  : Apb1En = Apb1En(1 << 25); /* (**) */
    pub const CANEN   : Apb1En = Apb1En(1 << 25); /* (XX) Alias for CAN1EN */
    pub const USBEN   : Apb1En = Apb1En(1 << 23); /* (XX) */
    pub const I2C2EN  : Apb1En = Apb1En(1 << 22);
    pub const I2C1EN  : Apb1En = Apb1En(1 << 21);
    pub const UART5EN : Apb1En = Apb1En(1 << 20);
    pub const UART4EN : Apb1En = Apb1En(1 << 19);
    pub const USART3EN: Apb1En = Apb1En(1 << 18);
    pub const USART2EN: Apb1En = Apb1En(1 << 17);
    pub const SPI3EN  : Apb1En = Apb1En(1 << 15);
    pub const SPI2EN  : Apb1En = Apb1En(1 << 14);
    pub const WWDGEN  : Apb1En = Apb1En(1 << 11);
    pub const TIM7EN  : Apb1En = Apb1En(1 << 5);
    pub const TIM6EN  : Apb1En = Apb1En(1 << 4);
    pub const TIM5EN  : Apb1En = Apb1En(1 << 3);
    pub const TIM4EN  : Apb1En = Apb1En(1 << 2);
    pub const TIM3EN  : Apb1En = Apb1En(1 << 1);
    pub const TIM2EN  : Apb1En = Apb1En(1 << 0);
}

pub trait RccExt {
    /// RCC Enable Peripheral Clocks.
    ///
    /// Enable the clock on particular peripherals. There are three registers
    /// involved, each one controlling the enabling of clocks associated with the
    /// AHB, APB1 and APB2 respectively. Several peripherals could be enabled
    /// simultaneously *only if they are controlled by the same register*.
    ///
    /// * `en : Enr` - Enum of Logical OR of all enables to be set
    /// For available constants, see [rcc::enr](enr)
    fn peripheral_enable_clock(&mut self, en : Enr);

    /// RCC Disable Peripheral Clocks.
    ///
    /// Enable the clock on particular peripherals. There are three registers
    /// involved, each one controlling the enabling of clocks associated with
    /// the AHB, APB1 and APB2 respectively. Several peripherals could be disabled
    /// simultaneously *only if they are controlled by the same register*.
    ///
    /// * `en : Enr` - Enum of Logical OR of all enables to be used for disabling.
    /// For available constants, see [rcc::enr](enr)
    fn peripheral_disable_clock(&mut self, en : Enr);

    /// RCC Reset Peripherals.
    ///
    /// Reset particular peripherals. There are three registers involved, each one
    /// controlling reset of peripherals associated with the AHB, APB1 and APB2
    /// respectively. Several peripherals could be reset simultaneously *only if
    /// they are controlled by the same register*.
    ///
    /// * `en : Enr` - Enum of Logical OR of all resets.
    /// For available constants, see [rcc::enr](enr)
    fn peripheral_reset(&mut self, reset : Enr);

    /// RCC Remove Reset on Peripherals.
    ///
    /// Remove the reset on particular peripherals. There are three registers
    /// involved, each one controlling reset of peripherals associated with the AHB,
    /// APB1 and APB2 respectively. Several peripherals could have the reset removed
    /// simultaneously *only if they are controlled by the same register*.
    ///
    /// * `en : Enr` - Enum of Logical OR of all resets to be removed:
    /// For available constants, see [rcc::enr](enr)
    ///
    fn peripheral_clear_reset(&mut self, clear_reset : Enr);

    /// Enable Peripheral Clock in running mode.
    ///
    /// Enable the clock on particular peripheral.
    ///
    /// * `clken : RegBit` - Peripheral RCC
    ///
    /// For available constants, see [rcc::en](en) ([rcc::en::USART1](en::USART1) for example)
    fn periph_clock_enable(&mut self, clken : Per);

    /// Disable Peripheral Clock in running mode.
    /// Disable the clock on particular peripheral.
    ///
    /// * `clken : RegBit` - Peripheral RCC
    ///
    /// For available constants, see [rcc::en](en) ([rcc::en::USART1](en::USART1) for example)
    fn periph_clock_disable(&mut self, clken : Per);
    
    /// Reset Peripheral, pulsed
    ///
    /// Reset particular peripheral, and restore to working state.
    ///
    /// * `rst : RegBit` - Peripheral reset
    ///
    /// For available constants, see [rcc::rst](rst) ([rcc::rst::USART1](rst::USART1) for example)
    fn periph_reset_pulse(&mut self, rst : Per);
    
    /// Reset Peripheral, hold
    ///
    /// Reset particular peripheral, and hold in reset state.
    ///
    /// * `rst : RegBit` - Peripheral reset
    ///
    /// For available constants, see [rcc::rst](rst) ([rcc::rst::USART1](rst::USART1) for example)
    fn periph_reset_hold(&mut self, rst : Per);

    /// Reset Peripheral, release
    ///
    /// Restore peripheral from reset state to working state.
    ///
    /// * `rst : RegBit` - Peripheral reset
    ///
    /// For available constants, see [rcc::rst](rst) ([rcc::rst::USART1](rst::USART1) for example)
    fn periph_reset_release(&mut self, rst : Per);

    /// Select the source of Microcontroller Clock Output
    ///
    /// Exact sources available depend on your target.  On devices with multiple
    /// MCO pins, this function controls MCO1
    ///
    /// * `mcosrc : Mco` - the unshifted source bits
    fn set_mco(&mut self, mcosrc : Mco);

    /// RCC Enable Bypass.
    /// Enable an external clock to bypass the internal clock (high speed and low
    /// speed clocks only). The external clock must be enabled (see @ref rcc_osc_on)
    /// and the internal clock must be disabled (see @ref rcc_osc_off) for this to
    /// have effect.
    ///
    /// **Note**: The LSE clock is in the backup domain and cannot be bypassed until the
    /// backup domain write protection has been removed (see @ref
    /// pwr_disable_backup_domain_write_protect).
    ///
    /// * `osc: Osc` - Oscillator ID. Only HSE and LSE have effect.
    fn osc_bypass_enable(&mut self, osc: Osc);

    /// RCC Disable Bypass.
    /// Re-enable the internal clock (high speed and low speed clocks only). The
    /// internal clock must be disabled (see @ref rcc_osc_off) for this to have
    /// effect.
    ///
    /// **Note**: The LSE clock is in the backup domain and cannot have bypass removed
    /// until the backup domain write protection has been removed (see @ref
    /// pwr_disable_backup_domain_write_protect) or the backup domain has been reset
    /// (see @ref rcc_backupdomain_reset).
    ///
    /// * `osc: Osc` - Oscillator ID. Only HSE and LSE have effect.
    fn osc_bypass_disable(&mut self, osc: Osc);
}

impl RccExt for RCC {
    fn peripheral_enable_clock(&mut self, en : Enr) {
        match en {
            Enr::Ahb(e) => { self.ahbenr .modify(|r,w| unsafe { w
                    .bits( r.bits() | e.value() )
                });
            },
            Enr::Apb1(e) => { self.apb1enr .modify(|r,w| unsafe { w
                    .bits( r.bits() | e.value() )
                });
            },
            Enr::Apb2(e) => { self.apb2enr .modify(|r,w| unsafe { w
                    .bits( r.bits() | e.value() )
                });
            }
        }
    }
    fn peripheral_disable_clock(&mut self, en : Enr) {
        match en {
            Enr::Ahb(e) => { self.ahbenr .modify(|r,w| unsafe { w
                    .bits( r.bits() & !e.value() )
                });
            },
            Enr::Apb1(e) => { self.apb1enr .modify(|r,w| unsafe { w
                    .bits( r.bits() & !e.value() )
                });
            },
            Enr::Apb2(e) => { self.apb2enr .modify(|r,w| unsafe { w
                    .bits( r.bits() & !e.value() )
                });
            }
        }
    }
    fn peripheral_reset(&mut self, reset : Enr) {
        match reset {
            Enr::Apb1(rt) => { self.apb1rstr .modify(|r,w| unsafe { w
                    .bits( r.bits() | rt.value() )
                });
            },
            Enr::Apb2(rt) => { self.apb2rstr .modify(|r,w| unsafe { w
                    .bits( r.bits() | rt.value() )
                });
            },
            _ => {
                unreachable!()
            }
        }
    }
    fn peripheral_clear_reset(&mut self, clear_reset : Enr) {
        match clear_reset {
            Enr::Apb1(cr) => { self.apb1rstr .modify(|r,w| unsafe { w
                    .bits( r.bits() & !cr.value() )
                });
            },
            Enr::Apb2(cr) => { self.apb2rstr .modify(|r,w| unsafe { w
                    .bits( r.bits() & !cr.value() )
                });
            },
            _ => {
                unreachable!()
            }
        }
    }
    
    fn periph_clock_enable(&mut self, clken : Per) {
        match clken {
            Per::Ahb(e) => {
                self.ahbenr.modify(|r,w| unsafe { w
                    .bits( r.bits() | (1 << e) )
                });
            },
            Per::Apb1(e) => {
                self.apb1enr.modify(|r,w| unsafe { w
                    .bits( r.bits() | (1 << e) )
                });
            },
            Per::Apb2(e) => {
                self.apb2enr.modify(|r,w| unsafe { w
                    .bits( r.bits() | (1 << e) )
                });
            }
        }
    }

    fn periph_clock_disable(&mut self, clken : Per) {
        match clken {
            Per::Ahb(e) => {
                self.ahbenr.modify(|r,w| unsafe { w
                    .bits( r.bits() & !(1 << e) )
                });
            },
            Per::Apb1(e) => {
                self.apb1enr.modify(|r,w| unsafe { w
                    .bits( r.bits() & !(1 << e) )
                });
            },
            Per::Apb2(e) => {
                self.apb2enr.modify(|r,w| unsafe { w
                    .bits( r.bits() & !(1 << e) )
                });
            }
        }
    }
    
    fn periph_reset_pulse(&mut self, rst : Per) {
        match rst {
            Per::Apb1(rt) => {
                self.apb1rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() | (1 << rt) )
                });
            },
            Per::Apb2(rt) => {
                self.apb2rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() | (1 << rt) )
                });
            },
            _ => {},
        }
    }
    
    fn periph_reset_hold(&mut self, rst : Per) {
        match rst {
            Per::Apb1(rt) => {
                self.apb1rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() | (1 << rt) )
                });
                self.apb1rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() & !(1 << rt) )
                });
            },
            Per::Apb2(rt) => {
                self.apb2rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() | (1 << rt) )
                });
                self.apb2rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() & !(1 << rt) )
                });
            },
            _ => {},
        }
    }

    fn periph_reset_release(&mut self, rst : Per) {
        match rst {
            Per::Apb1(rs) => {
                self.apb1rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() & !(1 << rs) )
                });
            },
            Per::Apb2(rs) => {
                self.apb2rstr  .modify(|r,w| unsafe { w
                    .bits( r.bits() & !(1 << rs) )
                });
            },
            _ => {},
        }
    }

    fn set_mco(&mut self, mcosrc : Mco) {
        self.cfgr     .modify(|_,w| w
            .pllmul() .bits( mcosrc as u8 )
        );
    }

    fn osc_bypass_enable(&mut self, osc: Osc) {
        match osc {
            Osc::HSE => { self.cr.modify(|_,w| w
                    .hsebyp() .set_bit() );
            },
            Osc::LSE => {
                //let reg = &self.csr;
                let reg = &self.bdcr;
                reg.modify(|_,w| w
                    .lsebyp() .set_bit() );
            },
            /* Do nothing, only HSE/LSE allowed here. */
            _ => {}
        }
    }

    fn osc_bypass_disable(&mut self, osc: Osc) {
        match osc {
            Osc::HSE => { self.cr.modify(|_,w| w
                    .hsebyp() .clear_bit() );
            },
            Osc::LSE => {
                //let reg = &self.csr;
                let reg = &self.bdcr;
                reg.modify(|_,w| w
                    .lsebyp() .clear_bit() );
            },
            /* Do nothing, only HSE/LSE allowed here. */
            _ => {}
        }
    }
}
