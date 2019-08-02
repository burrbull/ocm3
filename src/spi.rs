//! Devices can have up to three SPI peripherals. The common 4-wire full-duplex
//! mode of operation is supported, along with 3-wire variants using unidirectional
//! communication modes or half-duplex bidirectional communication. A variety of
//! options allows many of the SPI variants to be supported. Multimaster operation
//! is also supported. A CRC can be generated and checked in hardware.
//! 
//! @note Some JTAG pins need to be remapped if SPI is to be used.
//! 
//! @note The I2S protocol shares the SPI hardware so the two protocols cannot be
//! used at the same time on the same peripheral.
//! 
//! Example: 1Mbps, positive clock polarity, leading edge trigger, 8-bit words,
//! LSB first.
//! ```
//! spi.init_master(1000000, spi::ClockPolarity::IdleLow,
//!         spi::ClockPhase::CaptureOnFirstTransition, spi::DataFrame::Bit8,
//!         spi::LsbFirst::Lsb);
//! spi.write(0x55);        // 8-bit write
//! spi.write(0xaa88);    // 16-bit write
//! reg8 = spi.read();        // 8-bit read
//! reg16 = spi.read();        // 16-bit read
//! ```
//! 
//! @todo need additional functions to aid ISRs in retrieving status


#[derive(Clone, Copy, PartialEq)]
pub enum BaudRatePsc {
    Div2    = 0,
    Div4    = 1,
    Div8    = 2,
    Div16    = 3,
    Div32    = 4,
    Div64    = 5,
    Div128    = 6,
    Div256    = 7
}

/// SPI data frame format
#[derive(Clone, Copy, PartialEq)]
pub enum DataFrame {
    Bit8, // Bit 11 0
    Bit16 // Bit 11 1
}

/// SPI clock polarity
use crate::hal::spi::Polarity as ClockPolarity;
/*pub enum ClockPolarity {
    TO_0_WHEN_IDLE,
    TO_1_WHEN_IDLE
}*/


/// SPI clock phase
use crate::hal::spi::Phase as ClockPhase;
/*pub enum SpiClockPhase {
    TRANSITION_1 = 0,
    TRANSITION_2 = 1
}*/

use crate::hal::spi::Mode;

fn mode_from_u8(mode : u8) -> Mode {
    match mode {
        0 => Mode { polarity : ClockPolarity::IdleLow, phase: ClockPhase::CaptureOnFirstTransition },
        1 => Mode { polarity : ClockPolarity::IdleLow, phase: ClockPhase::CaptureOnSecondTransition },
        2 => Mode { polarity : ClockPolarity::IdleHigh, phase: ClockPhase::CaptureOnFirstTransition },
        3 => Mode { polarity : ClockPolarity::IdleHigh, phase: ClockPhase::CaptureOnSecondTransition },
        _ => unreachable!()
    }
            /*.cpol()   .bit((mode >> 1)!=0)
            .cpha()   .bit((mode & 1)!=0)*/
}


/// SPI lsb/msb first
#[derive(Clone, Copy, PartialEq)]
pub enum LsbFirst {
    Msb, // Bit 7 0
    Lsb  // Bit 7 1
}


trait SpiExt {
    /// Configure the SPI as Master.
    ///
    /// The SPI peripheral is configured as a master with communication parameters
    /// baudrate, data format 8/16 bits, frame format lsb/msb first, clock polarity
    /// and phase. The SPI enable, CRC enable and CRC next controls are not affected.
    /// These must be controlled separately.
    ///
    /// To support multiple masters (dynamic switching between master and slave)
    /// you must set SSOE to 0 and select either software or hardware control of
    /// the NSS pin.
    ///
    /// * `br : BaudRatePsc` - Baudrate.
    /// * `cpol : ClockPolarity` - Clock polarity.
    /// * `cpha : ClockPhase` - Clock Phase.
    /// * `dff : DataFrame` - Data frame format 8/16 bits.
    /// *`lsbfirst : LsbFirst` - Frame format lsb/msb first.
    ///
    /// Returns int. Error code.
    fn init_master(&mut self, br : BaudRatePsc, cpol : ClockPolarity, cpha : ClockPhase,
            dff : DataFrame, lsbfirst : LsbFirst);

    /// SPI Set Data Frame Format to 8 bits
    fn set_dff_8bit(&mut self);

    /// SPI Set Data Frame Format to 16 bits
    fn set_dff_16bit(&mut self);
}

trait SpiF1Ext {
// TODO: Error handling?
    /// SPI Enable.
    ///
    /// The SPI peripheral is enabled.
    ///
    /// @todo Error handling?
    fn enable(&mut self);

// TODO: Error handling?
    /// SPI Disable.
    ///
    /// The SPI peripheral is disabled.
    ///
    fn disable(&mut self);

    /// SPI Clean Disable.
    ///
    /// Disable the SPI peripheral according to the procedure in section 23.3.8 of the
    /// reference manual.  This prevents corruption of any ongoing transfers and
    /// prevents the BSY flag from becoming unreliable.
    ///
    /// Returns data `u16` - 8 or 16 bit data from final read.
    fn clean_disable(&mut self) -> u16;

    /// SPI Data Write.
    ///
    /// Data is written to the SPI interface.
    ///
    /// * `data : u16` - 8 or 16 bit data to be written.
    fn write(&mut self, data : u16);

    /// SPI Data Write with Blocking.
    ///
    /// Data is written to the SPI interface after the previous write transfer has
    /// finished.
    ///
    /// * `data : u16` - 8 or 16 bit data to be written.
    fn send(&mut self, data : u16);

    /// SPI Data Read.
    ///
    /// Data is read from the SPI interface after the incoming transfer has finished.
    ///
    /// Returns data `u16` - 8 or 16 bit data.
    fn read(&mut self) -> u16;

    /// SPI Data Write and Read Exchange.
    ///
    /// Data is written to the SPI interface, then a read is done after the incoming
    /// transfer has finished.
    ///
    /// * `data : u16` - 8 or 16 bit data to be written.
    ///
    /// Returns data `u16` - 8 or 16 bit data.
    fn xfer(&mut self, data : u16) -> u16;

    /// SPI Set Bidirectional Simplex Mode.
    ///
    /// The SPI peripheral is set for bidirectional transfers in two-wire simplex mode
    /// (using a clock wire and a bidirectional data wire).
    fn set_bidirectional_mode(&mut self);

    /// SPI Set Unidirectional Mode.
    ///
    /// The SPI peripheral is set for unidirectional transfers. This is used in full
    /// duplex mode or when the SPI is placed in two-wire simplex mode that uses a
    /// clock wire and a unidirectional data wire.
    fn set_unidirectional_mode(&mut self);

    /// SPI Set Bidirectional Simplex Receive Only Mode.
    ///
    /// The SPI peripheral is set for bidirectional transfers in two-wire simplex mode
    /// (using a clock wire and a bidirectional data wire), and is placed in a receive
    /// state.
    fn set_bidirectional_receive_only_mode(&mut self);

    /// SPI Set Bidirectional Simplex Receive Only Mode.
    ///
    /// The SPI peripheral is set for bidirectional transfers in two-wire simplex mode
    /// (using a clock wire and a bidirectional data wire), and is placed in a transmit
    /// state.
    fn set_bidirectional_transmit_only_mode(&mut self);

    /// SPI Enable the CRC.
    ///
    /// The SPI peripheral is set to use a CRC field for transmit and receive.
    fn enable_crc(&mut self);

    /// SPI Disable the CRC.
    fn disable_crc(&mut self);

    /// SPI Next Transmit is a Data Word
    ///
    /// The next transmission to take place is a data word from the transmit buffer.
    /// This must be called before transmission to distinguish between sending
    /// of a data or CRC word.
    fn set_next_tx_from_buffer(&mut self);

    /// SPI Next Transmit is a CRC Word
    ///
    /// The next transmission to take place is a crc word from the hardware crc unit.
    /// This must be called before transmission to distinguish between sending
    /// of a data or CRC word.
    fn set_next_tx_from_crc(&mut self);

    /// SPI Set Full Duplex (3-wire) Mode
    fn set_full_duplex_mode(&mut self);

    /// SPI Set Receive Only Mode for Simplex (2-wire) Unidirectional
    /// Transfers
    fn set_receive_only_mode(&mut self);

    /// SPI Disable Slave Management by Hardware
    ///
    /// In slave mode the NSS hardware input is used as a select enable for the slave.
    fn disable_software_slave_management(&mut self);
    
    /// SPI Enable Slave Management by Software
    ///
    /// In slave mode the NSS hardware input is replaced by an internal software
    /// enable/disable of the slave (@ref spi_set_nss_high).
    fn enable_software_slave_management(&mut self);

    /// SPI Set the Software NSS Signal High
    ///
    /// In slave mode, and only when software slave management is used, this replaces
    /// the NSS signal with a slave select enable signal.
    ///
    /// @todo these should perhaps be combined with an SSM enable as it is meaningless
    /// otherwise
    fn set_nss_high(&mut self);

    /// SPI Set the Software NSS Signal Low
    ///
    /// In slave mode, and only when software slave management is used, this replaces
    /// the NSS signal with a slave select disable signal.
    fn set_nss_low(&mut self);

    /// SPI Set to Send LSB First
    fn send_lsb_first(&mut self);

    /// SPI Set to Send MSB First
    fn send_msb_first(&mut self);

    /// SPI Set the Baudrate Prescaler
    ///
    /// @todo Why is this specification different to the spi_init_master baudrate
    /// values?
    ///
    /// * `baudrate : BaudRatePsc` - Baudrate prescale value.
    fn set_baudrate_prescaler(&mut self, baudrate : BaudRatePsc);

    /// SPI Set to Master Mode
    fn set_master_mode(&mut self);

    /// SPI Set to Slave Mode
    fn set_slave_mode(&mut self);

    /// SPI Set the Clock Polarity to High when Idle
    fn set_clock_polarity_1(&mut self);

    /// SPI Set the Clock Polarity to Low when Idle
    fn set_clock_polarity_0(&mut self);

    /// SPI Set the Clock Phase to Capture on Trailing Edge
    fn set_clock_phase_1(&mut self);

    /// SPI Set the Clock Phase to Capture on Leading Edge
    fn set_clock_phase_0(&mut self);

    /// SPI Enable the Transmit Buffer Empty Interrupt
    fn enable_tx_buffer_empty_interrupt(&mut self);

    /// SPI Disable the Transmit Buffer Empty Interrupt
    fn disable_tx_buffer_empty_interrupt(&mut self);

    /// SPI Enable the Receive Buffer Ready Interrupt
    fn enable_rx_buffer_not_empty_interrupt(&mut self);

    /// SPI Disable the Receive Buffer Ready Interrupt
    fn disable_rx_buffer_not_empty_interrupt(&mut self);

    /// SPI Enable the Error Interrupt
    fn enable_error_interrupt(&mut self);

    /// SPI Disable the Error Interrupt
    fn disable_error_interrupt(&mut self);

    /// SPI Set the NSS Pin as an Output
    ///
    /// Normally used in master mode to allows the master to place all devices on the
    /// SPI bus into slave mode. Multimaster mode is not possible.
    fn enable_ss_output(&mut self);

    /// SPI Set the NSS Pin as an Input
    ///
    /// In master mode this allows the master to sense the presence of other masters. If
    /// NSS is then pulled low the master is placed into slave mode. In slave mode NSS
    /// becomes a slave enable.
    fn disable_ss_output(&mut self);

    /// SPI Enable Transmit Transfers via DMA
    ///
    /// This allows transmissions to proceed unattended using DMA to move data to the
    /// transmit buffer as it becomes available. The DMA channels provided for each
    /// SPI peripheral are given in the Technical Manual DMA section.
    fn enable_tx_dma(&mut self);

    /// SPI Disable Transmit Transfers via DMA
    fn disable_tx_dma(&mut self);

    /// SPI Enable Receive Transfers via DMA
    ///
    /// This allows received data streams to proceed unattended using DMA to move data
    /// from the receive buffer as data becomes available. The DMA channels provided
    /// for each SPI peripheral are given in the Technical Manual DMA section.
    fn enable_rx_dma(&mut self);

    /// SPI Disable Receive Transfers via DMA
    fn disable_rx_dma(&mut self);

    /// SPI Standard Mode selection
    /// @details Set SPI standard Modes
    /// Mode | CPOL | CPHA
    /// ---- | ---- | ----
    ///  0   |  0   |  0
    ///  1   |  0   |  1
    ///  2   |  1   |  0
    ///  3   |  1   |  1
    /// * `mode : u8` - Standard SPI mode (0, 1, 2, 3)
    /// @sa spi_set_clock_phase_0 spi_set_clock_phase_1
    /// @sa spi_set_clock_polarity_0 spi_set_clock_polarity_1
    fn set_standard_mode(&mut self, mode : u8);
}

macro_rules! impl_spi_f1 {
    ($SPIx:ty) => (

    impl SpiF1Ext for $SPIx {
    // TODO: Error handling?
        fn enable(&mut self) {
            self.cr1     .modify(|_,w| w
                .spe()   .set_bit()
            );
        }

    // TODO: Error handling?
        fn disable(&mut self) {
            self.cr1     .modify(|_,w| w
                .spe()   .clear_bit()
            );
            let _ = self.cr1.read().bits();
        }

        fn clean_disable(&mut self) -> u16 {
            // Wait to receive last data
            while self.sr.read().rxne().bit_is_clear() {}
            
            let data : u16 = self.dr.read().dr().bits();

            // Wait to transmit last data
            while self.sr.read().txe().bit_is_clear() {}

            // Wait until not busy
            while self.sr.read().bsy().bit_is_set() {}

            self.cr1     .modify(|_,w| w
                .spe()   .clear_bit()
            );
            data
        }

        fn write(&mut self, data : u16) {
            // Write data (8 or 16 bits, depending on DFF) into DR.
            self.dr     .write(|w| w
                .dr()   .bits( data )
            );
        }

        fn send(&mut self, data : u16) {
            // Wait for transfer finished.
            while self.sr.read().txe().bit_is_clear() {}

            // Write data (8 or 16 bits, depending on DFF) into DR.
            self.dr     .write(|w| w
                .dr()   .bits( data )
            );
        }

        fn read(&mut self) -> u16 {
            // Wait for transfer finished.
            while self.sr.read().rxne().bit_is_clear() {}

            // Read the data (8 or 16 bits, depending on DFF bit) from DR.
            self.dr.read().dr().bits()
        }

        fn xfer(&mut self, data : u16) -> u16 {
            self.write(data);

            // Wait for transfer finished.
            while self.sr.read().rxne().bit_is_clear() {}

            // Read the data (8 or 16 bits, depending on DFF bit) from DR.
            self.dr.read().dr().bits()
        }

        fn set_bidirectional_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .bidimode()   .set_bit()
            );
        }

        fn set_unidirectional_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .bidimode()   .clear_bit()
            );
        }

        fn set_bidirectional_receive_only_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .bidimode()   .set_bit()
            );
            self.cr1     .modify(|_,w| w
                .bidioe()   .clear_bit()
            );
        }

        fn set_bidirectional_transmit_only_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .bidimode()   .set_bit()
            );
            self.cr1     .modify(|_,w| w
                .bidioe()   .set_bit()
            );
        }

        fn enable_crc(&mut self) {
            self.cr1     .modify(|_,w| w
                .crcen()   .set_bit()
            );
        }

        fn disable_crc(&mut self) {
            self.cr1     .modify(|_,w| w
                .crcen()   .clear_bit()
            );
        }

        fn set_next_tx_from_buffer(&mut self) {
            self.cr1     .modify(|_,w| w
                .crcnext()   .clear_bit()
            );
        }

        fn set_next_tx_from_crc(&mut self) {
            self.cr1     .modify(|_,w| w
                .crcnext()   .set_bit()
            );
        }

        fn set_full_duplex_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .rxonly()   .clear_bit()
            );
        }

        fn set_receive_only_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .rxonly()   .set_bit()
            );
        }

        fn disable_software_slave_management(&mut self) {
            self.cr1     .modify(|_,w| w
                .ssm()   .set_bit()
            );
        }
        
        fn enable_software_slave_management(&mut self) {
            self.cr1     .modify(|_,w| w
                .ssm()   .set_bit()
            );
            // allow slave select to be an input
            self.cr2     .modify(|_,w| w
                .ssoe()   .clear_bit()
            );
        }

        fn set_nss_high(&mut self) {
            self.cr1     .modify(|_,w| w
                .ssi()   .set_bit()
            );
        }

        fn set_nss_low(&mut self) {
            self.cr1     .modify(|_,w| w
                .ssi()   .clear_bit()
            );
        }

        fn send_lsb_first(&mut self) {
            self.cr1     .modify(|_,w| w
                .lsbfirst()   .set_bit()
            );
        }

        fn send_msb_first(&mut self) {
            self.cr1     .modify(|_,w| w
                .lsbfirst()   .clear_bit()
            );
        }

        fn set_baudrate_prescaler(&mut self, baudrate : BaudRatePsc) {
            self.cr1     .modify(|_,w| w
                .br()   .bits( baudrate as u8 )
            );
        }

        fn set_master_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .mstr()   .set_bit()
            );
        }

        fn set_slave_mode(&mut self) {
            self.cr1     .modify(|_,w| w
                .mstr()   .clear_bit()
            );
        }

        fn set_clock_polarity_1(&mut self) {
            self.cr1     .modify(|_,w| w
                .cpol()   .set_bit()
            );
        }

        fn set_clock_polarity_0(&mut self) {
            self.cr1     .modify(|_,w| w
                .cpol()   .clear_bit()
            );
        }

        fn set_clock_phase_1(&mut self) {
            self.cr1     .modify(|_,w| w
                .cpha()   .set_bit()
            );
        }

        fn set_clock_phase_0(&mut self) {
            self.cr1     .modify(|_,w| w
                .cpha()   .clear_bit()
            );
        }

        fn enable_tx_buffer_empty_interrupt(&mut self) {
            self.cr2    .modify(|_,w| w
                .txeie()   .set_bit()
            );
        }

        fn disable_tx_buffer_empty_interrupt(&mut self) {
            self.cr2    .modify(|_,w| w
                .txeie()   .clear_bit()
            );
        }

        fn enable_rx_buffer_not_empty_interrupt(&mut self) {
            self.cr2    .modify(|_,w| w
                .rxneie()   .set_bit()
            );
        }

        fn disable_rx_buffer_not_empty_interrupt(&mut self) {
            self.cr2    .modify(|_,w| w
                .rxneie()   .clear_bit()
            );
        }

        fn enable_error_interrupt(&mut self) {
            self.cr2    .modify(|_,w| w
                .errie()   .set_bit()
            );
        }

        fn disable_error_interrupt(&mut self) {
            self.cr2    .modify(|_,w| w
                .errie()   .clear_bit()
            );
        }

        fn enable_ss_output(&mut self) {
            self.cr2    .modify(|_,w| w
                .ssoe()   .set_bit()
            );
        }

        fn disable_ss_output(&mut self) {
            self.cr2    .modify(|_,w| w
                .ssoe()   .clear_bit()
            );
        }

        fn enable_tx_dma(&mut self) {
            self.cr2    .modify(|_,w| w
                .txdmaen()   .set_bit()
            );
        }

        fn disable_tx_dma(&mut self) {
            self.cr2    .modify(|_,w| w
                .txdmaen()   .clear_bit()
            );
        }

        fn enable_rx_dma(&mut self) {
            self.cr2    .modify(|_,w| w
                .rxdmaen()   .set_bit()
            );
        }

        fn disable_rx_dma(&mut self) {
            self.cr2    .modify(|_,w| w
                .rxdmaen()   .clear_bit()
            );
        }

        fn set_standard_mode(&mut self, mode : u8) {
            let mode = mode_from_u8(mode);
            self.cr1    .modify(|_,w| w
                .cpol()   .bit ( mode.polarity == ClockPolarity::IdleHigh )        // Set CPOL value.
                .cpha()   .bit ( mode.phase == ClockPhase::CaptureOnSecondTransition )        // Set CPHA value.
            );
        }
    }
    )
}


macro_rules! impl_spi {
    ($SPIx:ty) => (

    impl SpiExt for $SPIx {
        fn init_master(&mut self, br : BaudRatePsc, cpol : ClockPolarity, cpha : ClockPhase,
                dff : DataFrame, lsbfirst : LsbFirst) {

            // Reset all bits omitting SPE, CRCEN and CRCNEXT bits.
            let spe = self.cr1.read().spe().bit_is_set();
            let crcen = self.cr1.read().crcen().bit_is_set();
            let crcnext = self.cr1.read().crcnext().bit_is_set();
            
            self.cr2    .modify(|_,w| w
                .ssoe() .set_bit()
            );
            
            self.cr1      .write(|w| w
                .spe()    .bit( spe )
                .crcen()  .bit( crcen )
                .crcnext().bit( crcnext )
                .mstr()   .set_bit()                // Configure SPI as master.
                .br()     .bits ( br as u8 )        // Set baud rate bits.
                .cpol()   .bit ( cpol == ClockPolarity::IdleHigh )        // Set CPOL value.
                .cpha()   .bit ( cpha == ClockPhase::CaptureOnSecondTransition )        // Set CPHA value.
                .dff()    .bit ( dff == DataFrame::Bit16 )        // Set data format (8 or 16 bits).
                .lsbfirst().bit ( lsbfirst == LsbFirst::Lsb )    // Set frame format (LSB- or MSB-first).
            );
        }

        fn set_dff_8bit(&mut self) {
            self.cr1    .modify(|_,w| w
                .dff()  .clear_bit()
            );
        }

        fn set_dff_16bit(&mut self) {
            self.cr1    .modify(|_,w| w
                .dff()  .set_bit()
            );
        }
    }
    )
}



use crate::device::{SPI1, SPI2};
impl_spi!(SPI1);
impl_spi!(SPI2);
impl_spi_f1!(SPI1);
impl_spi_f1!(SPI2);
