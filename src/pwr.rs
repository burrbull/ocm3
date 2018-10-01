use crate::device::PWR;

/// PVD level selection
#[derive(Clone, Copy, PartialEq)]
pub enum PowerVDlevel {
    V22 = 0,
    V23 = 1,
    V24 = 2,
    V25 = 3,
    V26 = 4,
    V27 = 5,
    V28 = 6,
    V29 = 7
}

pub trait PwrExt {
    /// Disable Backup Domain Write Protection.
    ///
    /// This allows backup domain registers to be changed. These registers are write
    /// protected after a reset.
    fn disable_backup_domain_write_protect(&mut self);

    /// Re-enable Backup Domain Write Protection.
    ///
    /// This protects backup domain registers from inadvertent change.
    fn enable_backup_domain_write_protect(&mut self);

    /// Enable Power Voltage Detector.
    ///
    /// This provides voltage level threshold detection. The result of detection is
    /// provided in the power voltage detector output flag (see @ref pwr_voltage_high)
    /// or by setting the EXTI16 interrupt (see datasheet for configuration details).
    ///
    /// * `pvd_level : PowerVDlevel`
    fn enable_power_voltage_detect(&mut self, pvd_level : PowerVDlevel);

    /// Disable Power Voltage Detector.
    fn disable_power_voltage_detect(&mut self);

    /// Clear the Standby Flag.
    ///
    /// This is set when the processor returns from a standby mode.
    fn clear_standby_flag(&mut self);

    /// Clear the Wakeup Flag.
    ///
    /// This is set when the processor receives a wakeup signal.
    fn clear_wakeup_flag(&mut self);

    /// Set Standby Mode in Deep Sleep.
    fn set_standby_mode(&mut self);

    /// Set Stop Mode in Deep Sleep.
    fn set_stop_mode(&mut self);

    /// Voltage Regulator On in Stop Mode.
    fn voltage_regulator_on_in_stop(&mut self);

    /// Voltage Regulator Low Power in Stop Mode.
    fn voltage_regulator_low_power_in_stop(&mut self);

    /// Enable Wakeup Pin.
    ///
    /// The wakeup pin is used for waking the processor from standby mode.
    fn enable_wakeup_pin(&mut self);

    /// Release Wakeup Pin.
    ///
    /// The wakeup pin is used for general purpose I/O.
    fn disable_wakeup_pin(&mut self);

    /// Get Voltage Detector Output.
    ///
    /// The voltage detector threshold must be set when the power voltage detector is
    /// enabled, see @ref pwr_enable_power_voltage_detect.
    ///
    /// Returns `bool`: true if the power voltage is above the preset voltage
    /// threshold.
    fn voltage_high(&self) -> bool;

    /// Get Standby Flag.
    ///
    /// The standby flag is set when the processor returns from a standby state. It is
    /// cleared by software (see @ref pwr_clear_standby_flag).
    ///
    /// Returns `bool`: true if the processor was in standby state.
    fn get_standby_flag(&self) -> bool;

    /// Get Wakeup Flag.
    ///
    /// The wakeup flag is set when a wakeup event has been received. It is
    /// cleared by software (see @ref pwr_clear_wakeup_flag).
    ///
    /// Returns `bool`: true if a wakeup event was received.
    fn get_wakeup_flag(&self) -> bool;
}



impl PwrExt for PWR {
    fn disable_backup_domain_write_protect(&mut self) {
        self.cr       .modify(|_,w| w
            .dbp()    .set_bit()
        );
    }

    fn enable_backup_domain_write_protect(&mut self) {
        self.cr       .modify(|_,w| w
            .dbp()    .clear_bit()
        );
    }

    fn enable_power_voltage_detect(&mut self, pvd_level : PowerVDlevel) {
        self.cr       .modify(|_,w| unsafe { w
            .pvde()    .set_bit()
            .pls()    .bits( pvd_level as u8 )
        });
    }

    fn disable_power_voltage_detect(&mut self) {
        self.cr       .modify(|_,w| w
            .pvde()    .clear_bit()
        );
    }

    fn clear_standby_flag(&mut self) {
        self.cr    .write(|w| w
            .csbf()  .set_bit()
        );
    }

    fn clear_wakeup_flag(&mut self) {
        self.cr    .write(|w| w
            .cwuf()  .set_bit()
        );
    }

    fn set_standby_mode(&mut self) {
        self.cr       .modify(|_,w| w
            .pdds()    .set_bit()
        );
    }

    fn set_stop_mode(&mut self) {
        self.cr       .modify(|_,w| w
            .pdds()    .clear_bit()
        );
    }

    fn voltage_regulator_on_in_stop(&mut self) {
        self.cr       .modify(|_,w| w
            .lpds()    .clear_bit()
        );
    }

    fn voltage_regulator_low_power_in_stop(&mut self) {
        self.cr       .modify(|_,w| w
            .lpds()    .set_bit()
        );
    }

    fn enable_wakeup_pin(&mut self) {
        self.csr       .modify(|_,w| w
            .ewup()    .set_bit()
        );
    }

    fn disable_wakeup_pin(&mut self) {
        self.csr       .modify(|_,w| w
            .ewup()    .clear_bit()
        );
    }

    fn voltage_high(&self) -> bool {
        self.csr .read()
            .pvdo() .bit_is_set()
    }

    fn get_standby_flag(&self) -> bool {
        self.csr .read()
            .sbf() .bit_is_set()
    }

    fn get_wakeup_flag(&self) -> bool {
        self.csr .read()
            .wuf() .bit_is_set()
    }
}
