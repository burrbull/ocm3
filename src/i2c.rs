
#[derive(Clone, Copy, PartialEq)]
pub enum DutyCycle {
    DIV2 = 0,
    DIV16_9 = 1
}

/// I2C Read/Write bit
#[derive(Clone, Copy, PartialEq)]
pub enum RW {
    Write = 0,
    Read = 1
}

/// I2C speed modes
#[derive(Clone, Copy, PartialEq)]
pub enum Speed {
    Sm100k,
    Fm400k,
    Fmp1m,
    Unknown
}


pub trait I2cExt {
    /// I2C Peripheral Enable.
    fn peripheral_enable(&mut self);
    
    /// I2C Peripheral Disable.
    ///
    /// This must not be reset while in Master mode until a communication has finished.
    /// In Slave mode, the peripheral is disabled only after communication has ended.
    fn peripheral_disable(&mut self);

    /// I2C Send Start Condition.
    ///
    /// If in Master mode this will cause a restart condition to occur at the end of the
    /// current transmission. If in Slave mode, this will initiate a start condition
    /// when the current bus activity is completed.
    fn send_start(&mut self);

    /// I2C Send Stop Condition.
    ///
    /// After the current byte transfer this will initiate a stop condition if in Master
    /// mode, or simply release the bus if in Slave mode.
    fn send_stop(&mut self);

    /// I2C Clear Stop Flag.
    ///
    /// Clear the "Send Stop" flag in the I2C config register
    fn clear_stop(&mut self);

    /// I2C Set the 7 bit Slave Address for the Peripheral.
    ///
    /// This sets an address for Slave mode operation, in 7 bit form.
    ///
    /// * `slave : u8` - Slave address 0...127.
    fn set_own_7bit_slave_address(&mut self, slave : u8);

    /// I2C Set the 10 bit Slave Address for the Peripheral.
    ///
    /// This sets an address for Slave mode operation, in 10 bit form.
    ///
    /// @todo add "I2C_OAR1(i2c) |= (1 << 14);" as above
    ///
    /// * `slave : u16` - Slave address 0...1023.
    fn set_own_10bit_slave_address(&mut self, slave : u16);

    /// I2C Set the secondary 7 bit Slave Address for the Peripheral.
    ///
    /// This sets a secondary address for Slave mode operation, in 7 bit form.
    ///
    /// * `slave : u8` - Slave address 0...127.
    fn set_own_7bit_slave_address_two(&mut self, slave : u8);

    /// I2C Enable dual addressing mode for the Peripheral.
    ///
    /// Both OAR1 and OAR2 are recognised in 7-bit addressing mode.
    fn enable_dual_addressing_mode(&mut self);

    /// I2C Disable dual addressing mode for the Peripheral.
    ///
    /// Only OAR1 is recognised in 7-bit addressing mode.
    fn disable_dual_addressing_mode(&mut self);

    /// I2C Set Peripheral Clock Frequency.
    ///
    /// Set the peripheral clock frequency: 2MHz to 36MHz (the APB frequency). Note
    /// that this is <b> not </b> the I2C bus clock. This is set in conjunction with
    /// the Clock Control register to generate the Master bus clock, see @ref
    /// i2c_set_ccr
    ///
    /// * `freq : u8` - Clock Frequency Setting.
    fn set_clock_frequency(&mut self, freq : u8);

    /// I2C Send Data.
    ///
    /// * `data : u8` - Byte to send.
    fn send_data(&mut self, data : u8);

    /// I2C Set Fast Mode.
    ///
    /// Set the clock frequency to the high clock rate mode (up to 400kHz). The actual
    /// clock frequency must be set with @ref i2c_set_clock_frequency
    fn set_fast_mode(&mut self);

    /// I2C Set Standard Mode.
    ///
    /// Set the clock frequency to the standard clock rate mode (up to 100kHz). The
    /// actual clock frequency must be set with @ref i2c_set_clock_frequency
    fn set_standard_mode(&mut self);

    /// I2C Set Bus Clock Frequency.
    ///
    /// Set the bus clock frequency. This is a 12 bit number (0...4095) calculated
    /// from the formulae given in the STM32F1 reference manual in the description
    /// of the CCR field. It is a divisor of the peripheral clock frequency
    /// @ref i2c_set_clock_frequency modified by the fast mode setting
    /// @ref i2c_set_fast_mode
    ///
    /// @todo provide additional API assitance to set the clock, eg macros
    ///
    /// * `freq : u16` - Bus Clock Frequency Setting 0...4095.
    fn set_ccr(&mut self, freq : u16);

    /// I2C Set the Rise Time.
    ///
    /// Set the maximum rise time on the bus according to the I2C specification, as 1
    /// more than the specified rise time in peripheral clock cycles. This is a 6 bit
    /// number.
    ///
    /// @todo provide additional APIP assistance.
    ///
    /// * `trise : u8` - Rise Time Setting 0...63.
    fn set_trise(&mut self, trise : u8);

    /// I2C Send the 7-bit Slave Address.
    ///
    /// * `slave : u8` - Slave address 0...1023.
    /// * `readwrite : RW` - Single bit to instruct slave to receive or send.
    fn send_7bit_address(&mut self, slave : u8, readwrite : RW);

    /// I2C Get Data.
    fn get_data(&mut self) -> u8;

    /// I2C Enable Interrupt
    ///
    /// * `interrupt : u32` - Interrupt to enable.
    fn enable_interrupt(&mut self, interrupt : u32);

    /// I2C Disable Interrupt
    ///
    /// * `interrupt : u32` - Interrupt to disable.
    fn disable_interrupt(&mut self, interrupt : u32);

    /// I2C Enable ACK
    ///
    /// Enables acking of own 7/10 bit address
    fn enable_ack(&mut self);

    /// I2C Disable ACK
    ///
    /// Disables acking of own 7/10 bit address
    fn disable_ack(&mut self);

    /// I2C NACK Next Byte
    ///
    /// Causes the I2C controller to NACK the reception of the next byte
    fn nack_next(&mut self);

    /// I2C NACK Next Byte
    ///
    /// Causes the I2C controller to NACK the reception of the current byte
    fn nack_current(&mut self);

    /// I2C Set clock duty cycle
    ///
    /// * `dutycycle : DutyCycle` - I2C duty cycle.
    fn set_dutycycle(&mut self, dutycycle : DutyCycle);

    /// I2C Enable DMA
    fn enable_dma(&mut self);

    /// I2C Disable DMA
    fn disable_dma(&mut self);

    /// I2C Set DMA last transfer
    fn set_dma_last_transfer(&mut self);

    /// I2C Clear DMA last transfer
    fn clear_dma_last_transfer(&mut self);
}

pub trait I2cReadWrite7Ext : I2cExt {
    fn write7(&mut self, addr : u8, data : &[u8], n : usize);
    
    fn read7(&mut self, addr : u8, res : &mut [u8], n : usize);
    
    /// Run a write/read transaction to a given 7bit i2c address
    /// If both write & read are provided, the read will use repeated start.
    /// Both write and read are optional
    /// There are likely still issues with repeated start/stop condtions!
    /// * `addr : u8` - addr 7 bit i2c device address
    /// * `w : &[u8]` - buffer of data to write
    /// * `wn : usize` - length of w
    /// * `r : &mut [u8]` - destination buffer to read into
    /// * `rn : usize` - number of bytes to read (r should be at least this long)
    fn transfer7(&mut self, addr : u8, w : &[u8], wn : usize, r : &mut [u8], rn : usize) {
        if wn != 0 {
            self.write7(addr, w, wn);
        }
        if rn != 0 {
            self.read7(addr, r, rn);
        } else {
            self.send_stop();
        }
    }
}

pub trait I2cSetSpeed : I2cExt {
    /// Set the i2c communication speed.
    /// * `speed : Speed` - one of the listed speed modes @ref i2c_speeds
    /// * `clock_megahz : u32` - i2c peripheral clock speed in MHz. Usually, rcc_apb1_frequency / 1e6
    fn set_speed(&mut self, speed : Speed, clock_megahz : u32);
}


macro_rules! impl_i2c {
    ($I2Cx:ty) => (

    impl I2cExt for $I2Cx {
        fn peripheral_enable(&mut self) {
            self.cr1      .modify(|_,w| w
                .pe()     .set_bit()
            );
        }
        
        fn peripheral_disable(&mut self) {
            self.cr1      .modify(|_,w| w
                .pe()     .clear_bit()
            );
        }

        fn send_start(&mut self) {
            self.cr1      .modify(|_,w| w
                .start()  .set_bit()
            );
        }

        fn send_stop(&mut self) {
            self.cr1      .modify(|_,w| w
                .stop()   .set_bit()
            );
        }

        fn clear_stop(&mut self) {
            self.cr1      .modify(|_,w| w
                .stop()   .clear_bit()
            );
        }

        fn set_own_7bit_slave_address(&mut self, slave : u8) {
            /* Datasheet: always keep 1 by software. */
            self.oar1     .modify(|r,w| unsafe { w //????
                .bits( r.bits() | (1 << 14) )
                .add7()   .bits( slave )
            });
        }

        fn set_own_10bit_slave_address(&mut self, slave : u16) {
            self.oar1     .modify(|r,w| unsafe { w //????
                .bits( r.bits() | (slave as u32) | (1 << 14) )
                .addmode()   .set_bit( )
            });
        }

        fn set_own_7bit_slave_address_two(&mut self, slave : u8) {
            self.oar2     .modify(|_,w| unsafe { w
                .add2()   .bits( slave )
            });
        }

        fn enable_dual_addressing_mode(&mut self) {
            self.oar2     .modify(|_,w| w
                .endual() .set_bit()
            );
        }

        fn disable_dual_addressing_mode(&mut self) {
            self.oar2     .modify(|_,w| w
                .endual() .clear_bit()
            );
        }

        fn set_clock_frequency(&mut self, freq : u8) {
            self.cr2      .modify(|_,w| unsafe { w
                .freq()   .bits( freq )
            });
        }

        fn send_data(&mut self, data : u8) {
            self.cr2      .modify(|_,w| unsafe { w
                .freq()   .bits( data )
            });
        }

        fn set_fast_mode(&mut self) {
            self.ccr     .modify(|_,w| w
                .f_s()   .set_bit()
            );
        }

        fn set_standard_mode(&mut self) {
            self.ccr     .modify(|_,w| w
                .f_s()   .clear_bit()
            );
        }

        fn set_ccr(&mut self, freq : u16) {
            self.ccr      .modify(|_,w| unsafe { w
                .ccr()    .bits( freq )
            });
        }

        fn set_trise(&mut self, trise : u8) {
            self.trise      .write(|w| unsafe { w
                .trise()    .bits( trise )
            });
        }

        fn send_7bit_address(&mut self, slave : u8, readwrite : RW) {
            //let bits = ((slave as u16) << 1) as u8 ) | ( readwrite as u8)
            let bits = (slave << 1) | ( readwrite as u8);
            self.dr     .write(|w| unsafe { w
                .dr()   .bits( bits )
            });
        }

        fn get_data(&mut self) -> u8 {
            self.dr.read().dr().bits()
        }

        fn enable_interrupt(&mut self, interrupt : u32) {
            self.cr2     .modify(|r,w| unsafe { w
                .bits( r.bits() | interrupt)
            });
        }

        fn disable_interrupt(&mut self, interrupt : u32) {
            self.cr2     .modify(|r,w| unsafe { w
                .bits( r.bits() & !interrupt)
            });
        }

        fn enable_ack(&mut self) {
            self.cr1     .modify(|_,w| w
                .ack()   .set_bit()
            );
        }

        fn disable_ack(&mut self) {
            self.cr1     .modify(|_,w| w
                .ack()   .clear_bit()
            );
        }

        fn nack_next(&mut self) {
            self.cr1     .modify(|_,w| w
                .pos()   .set_bit()
            );
        }

        fn nack_current(&mut self) {
            self.cr1     .modify(|_,w| w
                .pos()   .clear_bit()
            );
        }

        fn set_dutycycle(&mut self, dutycycle : DutyCycle) {
            self.ccr     .modify(|_,w| w
                .duty()  .bit((dutycycle as u8) == 1)
            );
        }

        fn enable_dma(&mut self) {
            self.cr2     .modify(|_,w| w
                .dmaen() .set_bit()
            );
        }

        fn disable_dma(&mut self) {
            self.cr2     .modify(|_,w| w
                .dmaen() .clear_bit()
            );
        }

        fn set_dma_last_transfer(&mut self) {
            self.cr2     .modify(|_,w| w
                .last()  .set_bit()
            );
        }

        fn clear_dma_last_transfer(&mut self) {
            self.cr2     .modify(|_,w| w
                .last()  .clear_bit()
            );
        }
    }

    impl I2cReadWrite7Ext for $I2Cx {

        fn write7(&mut self, addr : u8, data : &[u8], n : usize) {
            while self.sr2.read().busy().bit_is_set() {}

            self.send_start();

            /* Wait for master mode selected */
            while self.sr1.read().addr().bit_is_clear() &&
                (self.sr2.read().msl().bit_is_set() || self.sr2.read().busy().bit_is_set()) {}

            self.send_7bit_address(addr, RW::Write);

            /* Waiting for address is transferred. */
            while self.sr1.read().addr().bit_is_clear() {}

            /* Clearing ADDR condition sequence. */
            let _ = self.sr2.read().bits();
            
            for i in 0..n {
                self.send_data(data[i]);
                while self.sr1.read().btf().bit_is_clear() {}
            }
        }

        fn read7(&mut self, addr : u8, res : &mut [u8], n : usize) {
            self.send_start();
            self.enable_ack();

            /* Wait for master mode selected */
            while self.sr1.read().addr().bit_is_clear() &&
                (self.sr2.read().msl().bit_is_set() || self.sr2.read().busy().bit_is_set()) {}

            self.send_7bit_address(addr, RW::Read);

            /* Waiting for address is transferred. */
            while self.sr1.read().addr().bit_is_clear() {}
            /* Clearing ADDR condition sequence. */
            let _ = self.sr2.read().bits();
            
            for i in 0..n {
                if i == n - 1 {
                    self.disable_ack();
                }
                while self.sr1.read().rx_ne().bit_is_clear() {}
                res[i] = self.get_data();
            }
            self.send_stop();
        }

    }

    impl I2cSetSpeed for $I2Cx {
        fn set_speed(&mut self, speed : Speed, clock_megahz : u32) {
            self.set_clock_frequency(clock_megahz as u8);
            match speed {
                Speed::Fm400k => {
                    self.set_fast_mode();
                    self.set_ccr((clock_megahz * 5 / 6) as u16);
                    self.set_trise((clock_megahz + 1) as u8);
                },
                /* fall back to standard mode */
                Speed::Sm100k => {
                    self.set_standard_mode();
                    /* x Mhz / (100kHz * 2) */
                    self.set_ccr((clock_megahz * 5) as u16);
                    /* Sm mode, (100kHz) freqMhz + 1 */
                    self.set_trise((clock_megahz + 1) as u8);
                },
                _ => {}
            }
        }
    }

    )
}


use crate::device::{I2C1,I2C2};
impl_i2c!(I2C1);
impl_i2c!(I2C2);
