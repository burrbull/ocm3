//! DMA Control System in the STM32 series of ARM Cortex
//! Microcontrollers by ST Microelectronics.
//!
//! Up to two DMA controllers are supported. 12 DMA channels are allocated 7 to
//! the first DMA controller and 5 to the second. Each channel is connected to
//! between 3 and 6 hardware peripheral DMA signals in a logical OR arrangement.
//!
//! DMA transfers can be configured to occur between peripheral and memory in
//! any combination including memory to memory. Circular mode transfers are
//! also supported in transfers involving a peripheral. An arbiter is provided
//! to resolve priority DMA requests. Transfers can be made with 8, 16 or 32 bit
//! words. 

/// Channel priority level
pub use crate::device::dma1::ch::cr::PLW as Priority;
/// Memory size
///
/// Peripheral size
pub use crate::device::dma1::ch::cr::PSIZEW as DataSize;


bitortype!(Irq, u8);

/// DMA Interrupt Flag offset values
///
/// These are based on every interrupt flag and flag clear being at the same
/// relative location
pub mod irq {
    use super::Irq;
    /** Transfer Error Interrupt Flag */
    pub const TEIF: Irq =  Irq(1 << 3);
    /** Half Transfer Interrupt Flag */
    pub const HTIF: Irq =  Irq(1 << 2);
    /** Transfer Complete Interrupt Flag */
    pub const TCIF: Irq =  Irq(1 << 1);
    /** Global Interrupt Flag */
    pub const GIF: Irq =  Irq(1 << 0);
}

pub trait DmaInterruptExt {
    /// DMA Channel Clear Interrupt Flag
    ///
    /// The interrupt flag for the channel is cleared. More than one interrupt for the
    /// same channel may be cleared by using the logical OR of the interrupt flags.
    ///
    /// * `interrupts : Irq` - Logical OR of interrupt numbers: [dma::irq](irq)
    fn clear_interrupt_flags(&mut self, interrupts : Irq);

    /// DMA Channel Read Interrupt Flag
    ///
    /// The interrupt flag for the channel is returned.
    ///
    /// * `interrupt : Irq` - Interrupt number: [dma::irq](irq)
    /// @returns bool interrupt flag is set.
    fn get_interrupt_flag(&mut self, interrupt : Irq) -> bool;
}

pub trait DmaChannelResetExt {
    /// DMA Channel Reset
    ///
    /// The channel is disabled and configuration registers are cleared.
    fn reset(&mut self);
}

pub trait DmaChannelExt {

    /// DMA Channel Enable Memory to Memory Transfers
    ///
    /// Memory to memory transfers do not require a trigger to activate each transfer.
    /// Transfers begin immediately the channel has been enabled, and proceed without
    /// intervention.
    fn enable_mem2mem_mode(&mut self);

    /// DMA Channel Set Priority
    ///
    /// Channel Priority has four levels: low to very high. This has precedence over the
    /// hardware priority.
    ///
    /// * `prio : Priority` - Priority level
    fn set_priority(&mut self, prio : Priority);

    /// DMA Channel Set Memory Word Width
    ///
    /// Set the memory word width 8 bits, 16 bits, or 32 bits. Refer to datasheet for
    /// alignment information if the source and destination widths do not match.
    ///
    /// * `mem_size : DataSize` - Memory word width
    fn set_memory_size(&mut self, mem_size : DataSize);

    /// DMA Channel Set Peripheral Word Width
    ///
    /// Set the peripheral word width 8 bits, 16 bits, or 32 bits. Refer to datasheet
    /// for alignment information if the source and destination widths do not match, or
    /// if the peripheral does not support byte or half-word writes.
    ///
    /// * `peripheral_size : DataSize` - Peripheral word width
    fn set_peripheral_size(&mut self, peripheral_size : DataSize);

    /// DMA Channel Enable Memory Increment after Transfer
    ///
    /// Following each transfer the current memory address is incremented by
    /// 1, 2 or 4 depending on the data size set in @ref dma_set_memory_size. The
    /// value held by the base memory address register is unchanged.
    fn enable_memory_increment_mode(&mut self);

    /// DMA Channel Disable Memory Increment after Transfer
    fn disable_memory_increment_mode(&mut self);

    /// DMA Channel Enable Peripheral Increment after Transfer
    ///
    /// Following each transfer the current peripheral address is incremented by
    /// 1, 2 or 4 depending on the data size set in @ref dma_set_peripheral_size. The
    /// value held by the base peripheral address register is unchanged.
    fn enable_peripheral_increment_mode(&mut self);

    /// DMA Channel Disable Peripheral Increment after Transfer
    fn disable_peripheral_increment_mode(&mut self);

    /// DMA Channel Enable Memory Circular Mode
    ///
    /// After the number of bytes/words to be transferred has been completed, the
    /// original transfer block size, memory and peripheral base addresses are
    /// reloaded and the process repeats.
    ///
    /// **Note**: This cannot be used with memory to memory mode, which is explicitly
    /// disabled here.
    fn enable_circular_mode(&mut self);

    /// DMA Channel Enable Transfers from a Peripheral
    ///
    /// The data direction is set to read from a peripheral.
    fn set_read_from_peripheral(&mut self);

    /// DMA Channel Enable Transfers from Memory
    ///
    /// The data direction is set to read from memory.
    fn set_read_from_memory(&mut self);

    /// DMA Channel Enable Interrupt on Transfer Error
    fn enable_transfer_error_interrupt(&mut self);

    /// DMA Channel Disable Interrupt on Transfer Error
    fn disable_transfer_error_interrupt(&mut self);

    /// DMA Channel Enable Interrupt on Transfer Half Complete
    fn enable_half_transfer_interrupt(&mut self);

    /// DMA Channel Disable Interrupt on Transfer Half Complete
    fn disable_half_transfer_interrupt(&mut self);

    /// DMA Channel Enable Interrupt on Transfer Complete
    fn enable_transfer_complete_interrupt(&mut self);

    /// DMA Channel Disable Interrupt on Transfer Complete
    fn disable_transfer_complete_interrupt(&mut self);

    /// DMA Channel Enable
    fn enable(&mut self);

    /// DMA Channel Disable
    ///
    /// **Note**: The DMA channel registers retain their values when the channel is
    /// disabled.
    fn disable(&mut self);

}

pub trait DmaOtherExt {
    /// DMA Channel Set the Peripheral Address
    ///
    /// Set the address of the peripheral register to or from which data is to be
    /// transferred.  Refer to the documentation for the specific peripheral.
    ///
    /// **Note**: The DMA channel must be disabled before setting this address. This
    /// function has no effect if the channel is enabled.
    ///
    /// * `address : u32` - Peripheral Address.
    fn set_peripheral_address(&mut self, address : u32);

    /// DMA Channel Set the Base Memory Address
    ///
    /// **Note**: The DMA channel must be disabled before setting this address. This
    /// function has no effect if the channel is enabled.
    ///
    /// * `address : u32` - Memory Initial Address.
    fn set_memory_address(&mut self, address : u32);

    /// DMA Channel Set the Transfer Block Size
    ///
    /// **Note**: The DMA channel must be disabled before setting this count value. The
    /// count is not changed if the channel is enabled.
    ///
    /// * `number : u16` - Number of data words to transfer (65535
    /// maximum).
    fn set_number_of_data(&mut self, number : u16);
}

macro_rules! dmachannel {
    ($($DMAX:ident: ($dmaX:ident, $dmaXen:ident, $dmaXrst:ident, {
        $($CX:ident: (
            $teifX:ident,
            $htifX:ident,
            $tcifX:ident,
            $gifX:ident,
            $cteifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident
        ),)+
    }),)+) => {
        $(
            use crate::device_hal::dma::$dmaX;
            
            $(
                impl DmaInterruptExt for $dmaX::$CX {
                    fn clear_interrupt_flags(&mut self, interrupts : Irq) {
                        self.ifcr()   .write(|w| w
                            .$cteifX() .bit( irq::TEIF.value() & interrupts.value() > 0 )
                            .$chtifX() .bit( irq::HTIF.value() & interrupts.value() > 0 )
                            .$ctcifX() .bit( irq::TCIF.value() & interrupts.value() > 0 )
                            .$cgifX()  .bit( irq::GIF.value() & interrupts.value() > 0 )
                        );
                    }

                    fn get_interrupt_flag(&mut self, interrupt : Irq) -> bool {
                        match interrupt {
                            irq::TEIF => {
                                self.isr().$teifX() .bit_is_set()
                            },
                            irq::HTIF => {
                                self.isr().$htifX() .bit_is_set()
                            },
                            irq::TCIF => {
                                self.isr().$tcifX() .bit_is_set()
                            },
                            irq::GIF => {
                                self.isr().$gifX()  .bit_is_set()
                            },
                            _ => unreachable!()
                        }
                    }
                }

                impl DmaChannelResetExt for $dmaX::$CX {
                    fn reset(&mut self) {
                        /* Disable channel and reset config bits. */
                        self.ch().cr    .reset();
                        /* Reset data transfer number. */
                        self.ch().ndtr  .reset();
                        /* Reset peripheral address. */
                        self.ch().par   .reset();
                        /* Reset memory address. */
                        self.ch().mar   .reset();
                        /* Reset interrupt flags. */
                        self.ifcr()    .write(|w| w
                            .$cteifX() .set_bit()
                            .$chtifX() .set_bit()
                            .$ctcifX() .set_bit()
                            .$cgifX()  .set_bit()
                        );
                    }
                }

                impl DmaChannelExt for $dmaX::$CX {
                    fn enable_mem2mem_mode(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .mem2mem().set_bit()
                            .circ()   .clear_bit()
                        );
                    }

                    fn set_priority(&mut self, prio : Priority) {
                        self.ch().cr    .modify(|_,w| w
                            .pl()   .bits( prio as u8 )
                        );
                    }

                    fn set_memory_size(&mut self, mem_size : DataSize) {
                        self.ch().cr    .modify(|_,w| unsafe { w
                            .msize()   .bits( mem_size as u8 )
                        });
                    }

                    fn set_peripheral_size(&mut self, peripheral_size : DataSize) {
                        self.ch().cr    .modify(|_,w| unsafe { w
                            .psize()   .bits( peripheral_size as u8 )
                        });
                    }

                    fn enable_memory_increment_mode(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .minc() .set_bit()
                        );
                    }

                    fn disable_memory_increment_mode(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .minc() .clear_bit()
                        );
                    }

                    fn enable_peripheral_increment_mode(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .pinc() .set_bit()
                        );
                    }

                    fn disable_peripheral_increment_mode(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .pinc() .clear_bit()
                        );
                    }

                    fn enable_circular_mode(&mut self) {
                        self.ch().cr       .modify(|_,w| w
                            .circ()    .set_bit()
                            .mem2mem() .clear_bit()
                        );
                    }

                    fn set_read_from_peripheral(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .dir() .clear_bit()
                        );
                    }

                    fn set_read_from_memory(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .dir() .set_bit()
                        );
                    }

                    fn enable_transfer_error_interrupt(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .teie() .set_bit()
                        );
                    }

                    fn disable_transfer_error_interrupt(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .teie() .clear_bit()
                        );
                    }

                    fn enable_half_transfer_interrupt(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .htie() .set_bit()
                        );
                    }

                    fn disable_half_transfer_interrupt(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .htie() .clear_bit()
                        );
                    }

                    fn enable_transfer_complete_interrupt(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .tcie() .set_bit()
                        );
                    }

                    fn disable_transfer_complete_interrupt(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .tcie() .clear_bit()
                        );
                    }

                    fn enable(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .en() .set_bit()
                        );
                    }

                    fn disable(&mut self) {
                        self.ch().cr    .modify(|_,w| w
                            .en() .clear_bit()
                        );
                    }

                }

                impl DmaOtherExt for $dmaX::$CX {
                    fn set_peripheral_address(&mut self, address : u32) {
                        if self.ch().cr  .read().en().bit_is_clear() {
                            self.ch().par     .write(|w| unsafe { w
                                .bits( address )
                            });
                        }
                    }

                    fn set_memory_address(&mut self, address : u32) {
                        if self.ch().cr    .read().en().bit_is_clear() {
                            self.ch().mar     .write(|w| unsafe { w
                                .bits( address )
                            });
                        }
                    }

                    fn set_number_of_data(&mut self, number : u16) {
                        self.ch().ndtr    .write(|w| unsafe { w
                            .ndt()        .bits( number )
                        });
                    }
                }
            )+
        )+
    }
}


dmachannel! {
    DMA1: (dma1, dma1en, dma1rst, {
        C1: (
            teif1, htif1, tcif1, gif1,
            cteif1, chtif1, ctcif1, cgif1
        ),
        C2: (
            teif2, htif2, tcif2, gif2,
            cteif2, chtif2, ctcif2, cgif2
        ),
        C3: (
            teif3, htif3, tcif3, gif3,
            cteif3, chtif3, ctcif3, cgif3
        ),
        C4: (
            teif4, htif4, tcif4, gif4,
            cteif4, chtif4, ctcif4, cgif4
        ),
        C5: (
            teif5, htif5, tcif5, gif5,
            cteif5, chtif5, ctcif5, cgif5
        ),
        C6: (
            teif6, htif6, tcif6, gif6,
            cteif6, chtif6, ctcif6, cgif6
        ),
        C7: (
            teif7, htif7, tcif7, gif7,
            cteif7, chtif7, ctcif7, cgif7
        ),
    }),

    DMA2: (dma2, dma2en, dma2rst, {
        C1: (
            teif1, htif1, tcif1, gif1,
            cteif1, chtif1, ctcif1, cgif1
        ),
        C2: (
            teif2, htif2, tcif2, gif2,
            cteif2, chtif2, ctcif2, cgif2
        ),
        C3: (
            teif3, htif3, tcif3, gif3,
            cteif3, chtif3, ctcif3, cgif3
        ),
        C4: (
            teif4, htif4, tcif4, gif4,
            cteif4, chtif4, ctcif4, cgif4
        ),
        C5: (
            teif5, htif5, tcif5, gif5,
            cteif5, chtif5, ctcif5, cgif5
        ),
    }),
}
