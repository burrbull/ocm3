use crate::device::{RTC,RCC,PWR};

use crate::rcc::{self,RccExt,RccF1Ext,Osc};
use crate::pwr::{PwrExt};

/// RTC Interrupt Flags
#[derive(Clone, Copy, PartialEq)]
pub enum RtcFlag {
    /// Counter Second Flag
    SEC,
    /// Alarm Event Flag
    ALR,
    /// Counter Overflow Flag
    OW
}


pub trait RtcExt {
    /// RTC Set Operational from the Off state.
    ///
    /// Power up the backup domain clocks, enable write access to the backup domain,
    /// select the clock source, clear the RTC registers and enable the RTC.
    ///
    /// * `clock_source : Osc` - RTC clock source. Only the values HSE, LSE
    ///     and LSI are permitted.
    fn awake_from_off(&mut self, rcc : &mut RCC, pwr : &mut PWR, clock_source : Osc);

    /// RTC Enter Configuration Mode.
    ///
    /// Prime the RTC for configuration changes by giving access to the prescaler,
    /// and counter and alarm registers.
    fn enter_config_mode(&mut self);

    /// RTC Leave Configuration Mode.
    ///
    /// Revert the RTC to operational state.
    fn exit_config_mode(&mut self);

    /// RTC Set the Alarm Time.
    ///
    /// * `alarm_time : u32` - time at which the alarm event is triggered.
    fn set_alarm_time(&mut self, alarm_time : u32);

    /// RTC Enable the Alarm.
    fn enable_alarm(&mut self);

    /// RTC Disable the Alarm.
    fn disable_alarm(&mut self);

    /// RTC Set the prescaler Value
    ///
    /// * `prescale_val : u32` - 20 bit prescale divider.
    fn set_prescale_val(&mut self, prescale_val : u32);

    /// RTC return the Counter Value
    ///
    /// Returns `u32` - the 32 bit counter value.
    fn get_counter_val(&mut self) -> u32;

    /// RTC return the prescaler Value
    ///
    /// Returns `u32` - the 20 bit prescale divider.
    fn get_prescale_div_val(&mut self) -> u32;
/*
    /// RTC return the Alarm Value
    ///
    /// Returns `u32`: the 32 bit alarm value.
    fn get_alarm_val(&mut self) -> u32;
*/
    /// RTC set the Counter
    ///
    /// `counter_val : u32` - 32 bit time setting for the counter.
    fn set_counter_val(&mut self, counter_val : u32);

    /// RTC Enable Interrupt
    ///
    /// * `flag_val : RtcFlag` - The flag to enable.
    fn interrupt_enable(&mut self, flag_val : RtcFlag);

    /// RTC Disable Interrupt
    ///
    /// * `flag_val : RtcFlag` - The flag to disable.
    fn interrupt_disable(&mut self, flag_val : RtcFlag);

    /// RTC Clear an Interrupt Flag
    ///
    /// * `flag_val : RtcFlag` - The flag to clear.
    fn clear_flag(&mut self, flag_val : RtcFlag);

    /// RTC Return a Flag Setting
    ///
    /// * `flag_val : RtcFlag` - The flag to check.
    ///
    /// Returns `u32` - a nonzero value if the flag is set, zero otherwise.
    fn check_flag(&self, flag_val : RtcFlag) -> bool;

    /// RTC Start RTC after Standby Mode.
    ///
    /// Enable the backup domain clocks, enable write access to the backup
    /// domain and the RTC, and synchronise the RTC register access.
    fn awake_from_standby(&mut self, rcc : &mut RCC, pwr : &mut PWR);

    /// RTC Configuration on Wakeup
    ///
    /// Enable the backup domain clocks and write access to the backup domain.
    /// If the RTC has not been enabled, set the clock source and prescaler value.
    /// The parameters are not used if the RTC has already been enabled.
    ///
    /// * `clock_source : Osc` - RTC clock source. Only HSE, LSE
    ///     and LSI are permitted.
    /// * `prescale_val : u32` - 20 bit prescale divider.
    fn auto_awake(&mut self, rcc : &mut RCC, pwr : &mut PWR, clock_source : Osc, prescale_val : u32);
}

impl RtcExt for RTC {
    fn awake_from_off(&mut self, rcc : &mut RCC, pwr : &mut PWR, clock_source : Osc) {
        /* Enable power and backup interface clocks. */
        rcc.periph_clock_enable(rcc::en::PWR);
        rcc.periph_clock_enable(rcc::en::BKP);

        /* Enable access to the backup registers and the RTC. */
        pwr.disable_backup_domain_write_protect();

        /* Set the clock source */
        rcc.set_rtc_clock_source(clock_source);

        /* Clear the RTC Control Register */
        self.crh .reset();
        self.crl .reset();

        /* Enable the RTC. */
        rcc.enable_rtc_clock();

        /* Clear the Registers */
        self.enter_config_mode();
        self.prlh .write(|w| w);
        self.prll .write(|w| w);
        self.cnth .reset();
        self.cntl .reset();
        self.alrh .write(|w| w);
        self.alrl .write(|w| w);
        self.exit_config_mode();

        /* Wait for the RSF bit in RTC_CRL to be set by hardware. */
        self.crl  .modify(|_,w| w
            .rsf()   .clear_bit()
        );
        while self.crl.read().rsf().bit_is_clear() {};
    }

    fn enter_config_mode(&mut self) {
        /* Wait until the RTOFF bit is 1 (no RTC register writes ongoing). */
        while self.crl.read().rtoff().bit_is_clear() {};
        
        /* Enter configuration mode. */
        self.crl  .modify(|_,w| w
            .cnf()   .set_bit()
        );
    }

    fn exit_config_mode(&mut self) { 
        /* Exit configuration mode. */
        self.crl  .modify(|_,w| w
            .cnf()   .clear_bit()
        );
        
        /* Wait until the RTOFF bit is 1 (our RTC register write finished). */
        while self.crl.read().rtoff().bit_is_clear() {};
    }

    fn set_alarm_time(&mut self, alarm_time : u32) {
        self.enter_config_mode();
        self.alrl  .write(|w| unsafe { w
            .bits ( alarm_time & 0x0000ffff )
        });
        self.alrh  .write(|w| unsafe { w
            .bits ( (alarm_time & 0xffff0000) >> 16 )
        });
        self.exit_config_mode();
    }

    fn enable_alarm(&mut self) {
        self.enter_config_mode();
        self.crh  .modify(|_,w| w
            .alrie()   .set_bit()
        );
        self.exit_config_mode();
    }

    fn disable_alarm(&mut self) {
        self.enter_config_mode();
        self.crh  .modify(|_,w| w
            .alrie()   .clear_bit()
        );
        self.exit_config_mode();
    }

    fn set_prescale_val(&mut self, prescale_val : u32) {
        self.enter_config_mode();
        self.prll  .write(|w| unsafe { w /* PRL[15:0] */
            .bits ( prescale_val & 0x0000ffff )
        });
        self.prlh  .write(|w| unsafe { w /* PRL[19:16] */
            .bits ( (prescale_val & 0x000f0000) >> 16 )
        });
        self.exit_config_mode();
    }

    fn get_counter_val(&mut self) -> u32 {
        (self.cnth.read().bits() << 16) | self.cntl.read().bits()
    }

    fn get_prescale_div_val(&mut self) -> u32 {
        (self.divh.read().bits() << 16) | self.divl.read().bits()
    }

    /*fn get_alarm_val(&self) -> u32 {
        (self.alrh.read().bits() << 16) | self.alrl.read().bits()
    }*/

    fn set_counter_val(&mut self, counter_val : u32) {
        self.enter_config_mode();
        self.cnth  .write(|w| unsafe { w /* CNT[31:16] */
            .bits ( (counter_val & 0xffff0000) >> 16 )
        });
        self.cntl  .write(|w| unsafe { w /* CNT[15:0] */
            .bits ( counter_val & 0x0000ffff )
        });        
        self.exit_config_mode();
    }

    fn interrupt_enable(&mut self, flag_val : RtcFlag) {
        self.enter_config_mode();

        /* Set the correct interrupt enable. */
        match flag_val {
            RtcFlag::SEC => {
                self.crh .modify(|_,w| w
                    .secie() .set_bit()
                )
            },
            RtcFlag::ALR => {
                self.crh .modify(|_,w| w
                    .alrie() .set_bit()
                )
            },
            RtcFlag::OW => {
                self.crh .modify(|_,w| w
                    .owie() .set_bit()
                )
            }
        };
        self.exit_config_mode();
    }

    fn interrupt_disable(&mut self, flag_val : RtcFlag) {
        self.enter_config_mode();

        /* Disable the correct interrupt enable. */
        match flag_val {
            RtcFlag::SEC => {
                self.crh .modify(|_,w| w
                    .secie() .clear_bit()
                )
            },
            RtcFlag::ALR => {
                self.crh .modify(|_,w| w
                    .alrie() .clear_bit()
                )
            },
            RtcFlag::OW => {
                self.crh .modify(|_,w| w
                    .owie() .clear_bit()
                )
            }
        };
        self.exit_config_mode();
    }

    fn clear_flag(&mut self, flag_val : RtcFlag) {
        /* Configuration mode not needed. */

        /* Clear the correct flag. */
        match flag_val {
            RtcFlag::SEC => {
                self.crl .modify(|_,w| w
                    .secf() .clear_bit()
                )
            },
            RtcFlag::ALR => {
                self.crl .modify(|_,w| w
                    .alrf() .clear_bit()
                )
            },
            RtcFlag::OW => {
                self.crl .modify(|_,w| w
                    .owf() .clear_bit()
                )
            }
        };
    }

    fn check_flag(&self, flag_val : RtcFlag) -> bool {
        /* Read correct flag. */
        match flag_val {
            RtcFlag::SEC => { self.crl .read(). secf() .bit_is_set() },
            RtcFlag::ALR => { self.crl .read(). alrf() .bit_is_set() },
            RtcFlag::OW =>  { self.crl .read(). owf()  .bit_is_set() }
        }
    }

    fn awake_from_standby(&mut self, rcc : &mut RCC, pwr : &mut PWR) {
        /* Enable power and backup interface clocks. */
        rcc.periph_clock_enable(rcc::en::PWR);
        rcc.periph_clock_enable(rcc::en::BKP);

        /* Enable access to the backup registers and the RTC. */
        pwr.disable_backup_domain_write_protect();

        /* Wait for the RSF bit in RTC_CRL to be set by hardware. */
        self.crl  .modify(|_,w| w
            .rsf()   .clear_bit()
        );
        while self.crl.read().rsf().bit_is_clear() {};

        /* Wait for the last write operation to finish. */
        /* TODO: Necessary? */
        while self.crl.read().rtoff().bit_is_clear() {};
    }

    fn auto_awake(&mut self, rcc : &mut RCC, pwr : &mut PWR, clock_source : Osc, prescale_val : u32) {
        /* Enable power and backup interface clocks. */
        rcc.periph_clock_enable(rcc::en::PWR);
        rcc.periph_clock_enable(rcc::en::BKP);

        let flag = rcc.rtc_clock_enabled_flag();

        if flag {
            self.awake_from_standby(rcc, pwr);
        } else {
            self.awake_from_off(rcc, pwr, clock_source);
            self.set_prescale_val(prescale_val);
        }
    }

}
