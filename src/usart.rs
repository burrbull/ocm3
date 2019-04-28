#[derive(Clone, Copy, PartialEq)]
pub enum StopBits {
    Stop1   = 0,     /* 1 stop bit */
    Stop0_5 = 1,     /* 0.5 stop bits */
    Stop2   = 2,     /* 2 stop bits */
    Stop1_5 = 3     /* 1.5 stop bits */
}

/// USART Parity Selection
#[derive(Clone, Copy, PartialEq)]
pub enum Parity {
    None,
    Even,
    Odd
}

/// USART Tx/Rx Mode Selection
#[derive(Clone, Copy, PartialEq)]
pub enum Mode {
    Rx,
    Tx,
    TxRx
}

/// USART Hardware Flow Control Selection
#[derive(Clone, Copy, PartialEq)]
pub enum FlowControl {
    None,
    Rts,
    Cts,
    RtsCts
}


pub trait UsartBaudExt {
    /// USART Set Baudrate.
    ///
    /// The baud rate is computed from the APB high-speed prescaler clock (for
    /// USART1/6) or the APB low-speed prescaler clock (for other USARTs). These values
    /// must be correctly set before calling this function (refer to the
    /// rcc_clock_setup-* functions in RCC).
    ///
    /// * `baud: u32` - Baud rate specified in Hz.
    fn set_baudrate(&self, baud: u32, freq: crate::rcc::Frequencies);
}

#[cfg(any(feature="stm32f2",feature="stm32f4"))]
macro_rules! impl_usart_baud_f24_u16 {
    ($USARTx:ty) => (
    impl UsartBaudExt for $USARTx {
        fn set_baudrate(&self, baud: u32, freq: crate::rcc::Frequencies) {
            let clock = freq.apb2;

            // Yes it is as simple as that. The reference manual is
            // talking about fractional calculation but it seems to be only
            // marketing babble to sound awesome. It is nothing else but a
            // simple divider to generate the correct baudrate.
            //
            // **Note**: We round() the value rather than floor()ing it, for more
            // accurate divisor selection.
            self.brr    .write(|w| unsafe {w
                .bits( ((2 * clock) + baud) / (2 * baud) )
            });
        }
    }
)}

#[cfg(any(feature="stm32f2",feature="stm32f4"))]
macro_rules! impl_usart_baud_f24 {
    ($USARTx:ty) => (
    impl UsartBaudExt for $USARTx {
        fn set_baudrate(&self, baud: u32, freq: crate::rcc::Frequencies) {
            let clock = freq.apb1;

            // Yes it is as simple as that. The reference manual is
            // talking about fractional calculation but it seems to be only
            // marketing babble to sound awesome. It is nothing else but a
            // simple divider to generate the correct baudrate.
            //
            // **Note**: We round() the value rather than floor()ing it, for more
            // accurate divisor selection.
            self.brr    .write(|w| unsafe {w
                .bits( ((2 * clock) + baud) / (2 * baud) )
            });
        }
    }
)}

#[cfg(not(any(feature="stm32f2",feature="stm32f4")))]
macro_rules! impl_usart_baud_u1 {
    ($USARTx:ty) => (
    impl UsartBaudExt for $USARTx {
        fn set_baudrate(&self, baud: u32, freq: crate::rcc::Frequencies) {
            let clock = freq.apb2;
            self.brr    .write(|w| unsafe { w
                .bits( ((2 * clock) + baud) / (2 * baud) )
            });
        }
    }
)}

#[cfg(not(any(feature="stm32f2",feature="stm32f4")))]
macro_rules! impl_usart_baud {
    ($USARTx:ty) => (
    impl UsartBaudExt for $USARTx {
        fn set_baudrate(&self, baud: u32, freq: crate::rcc::Frequencies) {
            let clock = freq.apb1;
            self.brr    .write(|w| unsafe {w
                .bits( ((2 * clock) + baud) / (2 * baud) )
            });
        }
    }
)}

pub trait UsartExt {
    /// USART Set Word Length.
    ///
    /// The word length is set to 8 or 9 bits. Note that the last bit will be a parity
    /// bit if parity is enabled, in which case the data length will be 7 or 8 bits
    /// respectively.
    ///
    /// * `bits: u32` - Word length in bits 8 or 9.
    fn set_databits(&mut self, bits: u32);

    /// USART Set Stop Bit(s).
    ///
    /// The stop bits are specified as 0.5, 1, 1.5 or 2.
    ///
    /// * `stopbits: StopBits` - Stop bits.
    fn set_stopbits(&mut self, stopbits: StopBits);

    /// USART Set Parity.
    ///
    /// The parity bit can be selected as none, even or odd.
    ///
    /// * `parity: Parity` - Parity.
    fn set_parity(&mut self, parity: Parity);

    /// USART Set Rx/Tx Mode.
    ///
    /// The mode can be selected as Rx only, Tx only or Rx+Tx.
    ///
    /// * `mode: Mode` - Mode.
    fn set_mode(&mut self, mode: Mode);

    /// USART Set Hardware Flow Control.
    ///
    /// The flow control bit can be selected as none, RTS, CTS or RTS+CTS.
    ///
    /// * `flowcontrol: FlowControl` - Flowcontrol.
    fn set_flow_control(&mut self, flowcontrol: FlowControl);

    /// USART Enable.
    fn enable(&mut self);

    /// USART Disable.
    ///
    /// At the end of the current frame, the USART is disabled to reduce power.
    fn disable(&mut self);

    /// USART Send Data Word with Blocking
    ///
    /// Blocks until the transmit data buffer becomes empty then writes the next data
    /// word for transmission.
    ///
    /// * `data: u16`.
    fn send_blocking(&mut self, data: u16);

    /// USART Read a Received Data Word with Blocking.
    ///
    /// Wait until a data word has been received then return the word.
    ///
    /// Returns `u16` data word.
    fn recv_blocking(&mut self) -> u16;

    /// USART Receiver DMA Enable.
    ///
    /// DMA is available on:
    /// * USART1 Rx DMA1 channel 5.
    /// * USART2 Rx DMA1 channel 6.
    /// * USART3 Rx DMA1 channel 3.
    /// * UART4 Rx DMA2 channel 3.
    fn enable_rx_dma(&mut self);

    /// USART Receiver DMA Disable.
    fn disable_rx_dma(&mut self);

    /// USART Transmitter DMA Enable.
    ///
    /// DMA is available on:
    /// * USART1 Tx DMA1 channel 4.
    /// * USART2 Tx DMA1 channel 7.
    /// * USART3 Tx DMA1 channel 2.
    /// * UART4 Tx DMA2 channel 5.
    fn enable_tx_dma(&mut self);

    /// USART Transmitter DMA Disable.
    fn disable_tx_dma(&mut self);

    /// USART Receiver Interrupt Enable.
    fn enable_rx_interrupt(&mut self);


    /// USART Receiver Interrupt Disable.
    fn disable_rx_interrupt(&mut self);

    /// USART Transmitter Interrupt Enable.
    fn enable_tx_interrupt(&mut self);

    /// USART Transmitter Interrupt Disable.
    fn disable_tx_interrupt(&mut self);

    /// USART Error Interrupt Enable.
    fn enable_error_interrupt(&mut self);

    /// USART Error Interrupt Disable.
    fn disable_error_interrupt(&mut self);
}


pub trait UsartF123Ext {
    /// USART Send a Data Word.
    ///
    /// * `data: u16`
    fn send(&mut self, data: u16);

    /// USART Read a Received Data Word.
    ///
    /// If parity is enabled the MSB (bit 7 or 8 depending on the word length) is the
    /// parity bit.
    ///
    /// Returns `u16` data word.
    fn recv(&self) -> u16;

    /// USART Wait for Transmit Data Buffer Empty
    ///
    /// Blocks until the transmit data buffer becomes empty and is ready to accept the
    /// next data word.
    fn wait_send_ready(&mut self);

    /// USART Wait for Received Data Available
    ///
    /// Blocks until the receive data buffer holds a valid received data word.
    fn wait_recv_ready(&mut self);

    /// USART Read a Status Flag.
    ///
    /// * `flag: u32` - Status register flag.
    ///
    /// Returns `bool` flag set.
    fn get_flag(&self, flag: u32) -> bool;
}


macro_rules! impl_usart {
    ($USARTx:ty) => (

    impl UsartExt for $USARTx {
        fn set_databits(&mut self, bits: u32) {
            self.cr1     .modify(|_,w| w
                .m()     .bit(!(bits == 8))
            );
        }
        
        fn set_stopbits(&mut self, stopbits: StopBits) {
            self.cr2      .modify(|_,w| unsafe { w
                .stop()   .bits( stopbits as u8 )
            });
        }
        
        fn set_parity(&mut self, parity: Parity) {
            match parity {
                Parity::None => {
                    self.cr1     .modify(|_,w| w
                        .pce()     .clear_bit()
                    );
                },
                Parity::Even => {
                    self.cr1     .modify(|_,w| w
                        .pce()     .set_bit()
                        .ps()    .clear_bit()
                    );
                },
                Parity::Odd => {
                    self.cr1     .modify(|_,w| w
                        .pce()     .set_bit()
                        .ps()    .set_bit()
                    );
                }
            };
        }
        
        fn set_mode(&mut self, mode: Mode) {
            match mode {
                Mode::Rx => {
                    self.cr1     .modify(|_,w| w
                        .re()     .set_bit()
                    );
                },
                Mode::Tx => {
                    self.cr1     .modify(|_,w| w
                        .te()     .set_bit()
                    );
                },
                Mode::TxRx => {
                    self.cr1     .modify(|_,w| w
                        .re()     .set_bit()
                        .te()     .set_bit()
                    );
                }
            };
        }
        
        fn set_flow_control(&mut self, flowcontrol: FlowControl) {
            match flowcontrol {
                FlowControl::None => {
                    self.cr3     .modify(|_,w| w
                        .rtse()     .clear_bit()
                        .ctse()     .clear_bit()
                    );
                },
                FlowControl::Rts => {
                    self.cr3     .modify(|_,w| w
                        .rtse()     .set_bit()
                    );
                },
                FlowControl::Cts => {
                    self.cr3     .modify(|_,w| w
                        .ctse()     .set_bit()
                    );
                },
                FlowControl::RtsCts => {
                    self.cr3     .modify(|_,w| w
                        .rtse()     .set_bit()
                        .ctse()     .set_bit()
                    );
                }
            };
        }

        /// USART Enable.
        fn enable(&mut self) {
            self.cr1     .modify(|_,w| w
                .ue()     .set_bit()
            );
        }

        fn disable(&mut self) {
            self.cr1     .modify(|_,w| w
                .ue()     .clear_bit()
            );
        }

        fn send_blocking(&mut self, data: u16) {
            self.wait_send_ready();
            self.send(data);
        }

        fn recv_blocking(&mut self) -> u16 {
            self.wait_recv_ready();
            self.recv()
        }

        fn enable_rx_dma(&mut self) {
            self.cr3     .modify(|_,w| w
                .dmar()     .set_bit()
            );
        }

        fn disable_rx_dma(&mut self) {
            self.cr3     .modify(|_,w| w
                .dmar()     .clear_bit()
            );
        }

        fn enable_tx_dma(&mut self) {
            self.cr3     .modify(|_,w| w
                .dmat()     .set_bit()
            );
        }

        fn disable_tx_dma(&mut self) {
            self.cr3     .modify(|_,w| w
                .dmat()     .clear_bit()
            );
        }

        fn enable_rx_interrupt(&mut self) {
            self.cr1     .modify(|_,w| w
                .rxneie()    .set_bit()
            );
        }

        fn disable_rx_interrupt(&mut self) {
            self.cr1     .modify(|_,w| w
                .rxneie()    .clear_bit()
            );
        }

        fn enable_tx_interrupt(&mut self) {
            self.cr1     .modify(|_,w| w
                .txeie()    .set_bit()
            );
        }

        fn disable_tx_interrupt(&mut self) {
            self.cr1     .modify(|_,w| w
                .txeie()    .clear_bit()
            );
        }

        fn enable_error_interrupt(&mut self) {
            self.cr3     .modify(|_,w| w
                .eie()     .set_bit()
            );
        }

        fn disable_error_interrupt(&mut self) {
            self.cr3     .modify(|_,w| w
                .eie()     .clear_bit()
            );
        }
    }
    )
}

macro_rules! impl_usartf123 {
    ($USARTx:ty) => (

    impl UsartF123Ext for $USARTx {
        fn send(&mut self, data: u16) {
            /* Send data. */
            self.dr    .write(|w| unsafe { w
                .dr() .bits( data )
            });
        }

        fn recv(&self) -> u16 {
            /* Receive data. */
            self.dr.read().dr().bits()
        }

        fn wait_send_ready(&mut self) {
            /* Wait until the data has been transferred into the shift register. */
            while self.sr.read().txe().bit_is_clear() {}
        }

        fn wait_recv_ready(&mut self) {
            /* Wait until the data is ready to be received. */
            while self.sr.read().rxne().bit_is_clear() {}
        }

        fn get_flag(&self, flag: u32) -> bool {
            (self.sr.read().bits() & flag) != 0
        }
    }

    )
}

use crate::device::{USART1,USART2};
impl_usart!(USART1);
impl_usart!(USART2);
impl_usartf123!(USART1);
impl_usartf123!(USART2);

#[cfg(not(any(feature="stm32f2",feature="stm32f4")))]
impl_usart_baud_u1!(USART1);
#[cfg(not(any(feature="stm32f2",feature="stm32f4")))]
impl_usart_baud!(USART2);

#[cfg(any(feature="stm32f2",feature="stm32f4"))]
impl_usart_baud_f24_u16!(USART1);
#[cfg(any(feature="stm32f2",feature="stm32f4"))]
impl_usart_baud_f24!(USART2);
