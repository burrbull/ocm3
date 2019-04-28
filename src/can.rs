

/* LEC[2:0]: Last error code */
#[derive(Clone, Copy, PartialEq)]
pub enum CanLastError {
    No    = 0,
    Stuff = 1,
    Form  = 2,
    Ack   = 3,
    Rec   = 4,
    Dom   = 5,
    Crc   = 6,
    Soft  = 7
}


/* SJW[1:0]: Resynchronization jump width */
#[derive(Clone, Copy, PartialEq)]
pub enum CanSyncJumpWidth {
    TQ1 = 0,
    TQ2 = 1,
    TQ3 = 2,
    TQ4 = 3
}

/* TS2[2:0]: Time segment 2 */
#[derive(Clone, Copy, PartialEq)]
pub enum CanTimeSegment2 {
    TQ1 = 0,
    TQ2 = 1,
    TQ3 = 2,
    TQ4 = 3,
    TQ5 = 4,
    TQ6 = 5,
    TQ7 = 6,
    TQ8 = 7
}

/* TS1[3:0]: Time segment 1 */
#[derive(Clone, Copy, PartialEq)]
pub enum CanTimeSegment1 {
    TQ1  = 0,
    TQ2  = 1,
    TQ3  = 2,
    TQ4  = 3,
    TQ5  = 4,
    TQ6  = 5,
    TQ7  = 6,
    TQ8  = 7,
    TQ9  = 8,
    TQ10 = 9,
    TQ11 = 10,
    TQ12 = 11,
    TQ13 = 12,
    TQ14 = 13,
    TQ15 = 14,
    TQ16 = 15
}


/// Timeout for CAN INIT acknowledge
/// this value is difficult to define.
/// INIT is set latest after finishing the current transfer.
/// Assuming the lowest CAN speed of 100kbps one CAN frame may take about 1.6ms
/// WAIT loop timeout varies on compiler switches, optimization, CPU architecture
/// and CPU speed
///
/// The same timeout value is used for leaving INIT where the longest time is
/// 11 bits(110 us on 100 kbps).
const CAN_MSR_INAK_TIMEOUT: u32 = 0x0000FFFF;


use crate::device::CAN1;
//use crate::device::{CAN1 as CAN,CAN2};

union EightBytes {
    data8: [u8;8],
    data32: [u32;2]
}

trait CanExt {
    /// CAN Init
    ///
    /// Initialize the selected CAN peripheral block.
    ///
    /// `ttcm: bool` - Time triggered communication mode.
    /// `abom: bool` - Automatic bus-off management.
    /// `awum: bool` - Automatic wakeup mode.
    /// `nart: bool` - No automatic retransmission.
    /// `rflm: bool` - Receive FIFO locked mode.
    /// `txfp: bool` - Transmit FIFO priority.
    /// `sjw: u32` - Resynchronization time quanta jump width.
    /// `ts1: u32` - Time segment 1 time quanta width.
    /// `ts2: u32` - Time segment 2 time quanta width.
    /// `brp: u32` - Baud rate prescaler.
    ///
    /// Returns int 0 on success, 1 on initialization failure.
    fn init (&mut self, ttcm: bool, abom: bool, awum: bool, nart: bool,
             rflm: bool, txfp: bool,
             sjw: CanSyncJumpWidth,
             ts1: CanTimeSegment1,
             ts2: CanTimeSegment2,
             brp: u32/*, loopback: bool, silent: bool*/) -> isize;

    /// CAN Filter Init
    ///
    /// Initialize incoming message filter and assign to FIFO.
    ///
    /// `nr: u32` - ID number of the filter.
    /// `scale_32bit: bool` - true for single 32bit, false for dual 16bit
    /// `id_list_mode: bool` - true for id lists, false for id/mask
    /// `fr1: u32` - First filter register content.
    /// `fr2: u32` - Second filter register content.
    /// `fifo: u32` - FIFO id.
    /// `enable: bool` - Enable filter?
    fn filter_init (&mut self, nr: usize, scale_32bit: bool,
                 id_list_mode: bool, fr1: u32, fr2: u32,
                 fifo: usize, enable: bool);

    /// CAN Initialize a 16bit Message ID Mask Filter
    ///
    /// `nr: u32` - ID number of the filter.
    /// `id1: u16` - First message ID to filter.
    /// `mask1: u16` - First message ID bit mask.
    /// `id2: u16` - Second message ID to filter.
    /// `mask2: u16` - Second message ID bit mask.
    /// `fifo: u32` - FIFO id.
    /// `enable: bool` - Enable filter?
    fn filter_id_mask_16bit_init (&mut self, nr: usize, id1: u16,
                       mask1: u16, id2: u16,
                       mask2: u16, fifo: usize, enable: bool);

    /// CAN Initialize a 32bit Message ID Mask Filter
    ///
    /// `nr: u32` - ID number of the filter.
    /// `id: u32` - Message ID to filter.
    /// `mask: u32` - Message ID bit mask.
    /// `fifo: u32` - FIFO id.
    /// `enable: bool` - Enable filter?
    fn filter_id_mask_32bit_init (&mut self, nr: usize, id: u32,
                       mask: u32, fifo: usize, enable: bool);

    /// CAN Initialize a 16bit Message ID List Filter
    ///
    /// `nr: u32` - ID number of the filter.
    /// `id1: u16` - First message ID to match.
    /// `id2: u16` - Second message ID to match.
    /// `id3: u16` - Third message ID to match.
    /// `id4: u16` - Fourth message ID to match.
    /// `fifo: u32` - FIFO id.
    /// `enable: bool` - Enable filter?
    fn filter_id_list_16bit_init (&mut self, nr: usize,
                       id1: u16, id2: u16,
                       id3: u16, id4: u16,
                       fifo: usize, enable: bool);

    /// CAN Initialize a 32bit Message ID List Filter
    ///
    /// `nr: u32` - ID number of the filter.
    /// `id1: u32` - First message ID to match.
    /// `id2: u32` - Second message ID to match.
    /// `fifo: u32` - FIFO id.
    /// `enable: bool` - Enable filter?
    fn filter_id_list_32bit_init (&mut self, nr: usize,
                       id1: u32, id2: u32,
                       fifo: usize, enable: bool);

    /// CAN Enable IRQ
    ///
    /// `irq: u32` - IRQ bit(s).
    fn enable_irq (&mut self, irq: u32);

    /// CAN Disable IRQ
    ///
    /// `irq: u32` - IRQ bit(s).
    fn disable_irq (&mut self, irq: u32);

    /// CAN Transmit Message
    ///
    /// `id: u32` - Message ID.
    /// `ext: bool` - Extended message ID?
    /// `rtr: bool` - Request transmit?
    /// `length: u8` - Message payload length.
    /// `data: &[u8]` - Message payload data.
    ///
    /// Returns int 0, 1 or 2 on success and depending on which outgoing mailbox got
    /// selected. -1 if no mailbox was available and no transmission got queued.
    fn transmit (&mut self, id: u32, ext: bool, rtr: bool, length: u8, data: &[u8]) -> isize;
    
    /// CAN Release FIFO
    ///
    /// `canport: u32` - CAN block register base @ref can_reg_base.
    /// `fifo: u8` - FIFO id.
    fn fifo_release (&mut self, fifo: usize);

    /// CAN Receive Message
    ///
    /// `fifo: u8` - FIFO id.
    /// `release: bool` - Release the FIFO automatically after coping data out.
    ///
    /// Outputs:
    /// `id: u32` - Message ID.
    /// `ext: bool` - The message ID is extended?
    /// `rtr: bool` - Request of transmission?
    /// `fmi: u8` - ID of the matched filter.
    /// `length: u8` - Length of message payload.
    /// `data: [u8;8]` - Message payload data.
    /// `timestamp: Option<u16>` - Pointer to store the message timestamp.
    ///             Only valid on time triggered CAN. Use `None` to ignore.
    fn receive (&mut self, fifo: usize, release: bool, timestamp: Option<u16>) ->
        (u32, bool, bool, u8, u8, [u8;8], Option<u16>);
        
    fn available_mailbox (&self) -> bool;
}

impl CanExt for CAN1 {
    #[inline]
    fn init (&mut self, ttcm: bool, abom: bool, awum: bool, nart: bool,
             rflm: bool, txfp: bool,
             sjw: CanSyncJumpWidth,
             ts1: CanTimeSegment1,
             ts2: CanTimeSegment2,
             brp: u32/*, loopback: bool, silent: bool*/) -> isize {

        /* Exit from sleep mode. */
        self.mcr   .modify(|_,w| w
            .sleep()  .clear_bit()
        );

        /* Request initialization "enter". */
        self.mcr   .modify(|_,w| w
            .inrq()  .set_bit()
        );

        /* Wait for acknowledge. */
        for _ in 0..CAN_MSR_INAK_TIMEOUT {
            if self.msr.read().inak().bit_is_set() { break; }
        }

        /* Check the acknowledge. */
        if self.msr.read().inak().bit_is_clear() { return 1; }

        /* clear can timing bits */
        self.btr  .reset();

        /* Set the automatic bus-off management. */
        self.mcr    .modify(|_,w| w
            .ttcm() .bit( ttcm )
            .abom() .bit( abom )
            .awum() .bit( awum )
            .nart() .bit( nart )
            .rflm() .bit( rflm )
            .txfp() .bit( txfp )
            /*.silm() .bit( silent )
            .lbkm() .bit( loopback )*/
            
        );
        /* Set bit timings. */
        self.btr    .write(|w| unsafe { w
            .sjw() .bits( sjw as u8 )
            .ts1() .bits( ts1 as u8 )
            .ts2() .bits( ts2 as u8 )
            .brp() .bits( (brp - 1) as u16 )
        });

        /* Request initialization "leave". */
        self.mcr   .modify(|_,w| w
            .inrq()  .clear_bit()
        );

        /* Wait for acknowledge. */
        for _ in 0..CAN_MSR_INAK_TIMEOUT {
            if self.msr.read().inak().bit_is_set() { break; }
        }

        if self.msr.read().inak().bit_is_clear() { return 1; }

        0
    }

    #[inline]
    fn filter_init (&mut self, nr: usize, scale_32bit: bool,
                 id_list_mode: bool, fr1: u32, fr2: u32,
                 fifo: usize, enable: bool) {
        let filter_select_bit: u32 = 1u32 << nr;

        /* Request initialization "enter". */
        self.fmr   .modify(|_,w| w
            .finit()  .set_bit()
        );

        /* Deactivate the filter. */
        self.fa1r    .modify(|r,w| unsafe { w
            .bits ( r.bits() &  !filter_select_bit )
        });


        self.fs1r    .modify(|r,w| unsafe { w
            .bits ( if scale_32bit { /* Set 32-bit scale for the filter. */
                    r.bits() | filter_select_bit
                } else { /* Set 16-bit scale for the filter. */
                    r.bits() &  !filter_select_bit
                } )
        });

        self.fm1r    .modify(|r,w| unsafe { w
            .bits ( if id_list_mode { /* Set filter mode to ID list mode. */
                    r.bits() | filter_select_bit
                } else { /* Set filter mode to id/mask mode. */
                    r.bits() &  !filter_select_bit
                } )
        });
        
        /* Set the first filter register. */
        self.fb[nr].fr1   .write(|w| unsafe { w
            .bits( fr1 )
        });
        /* Set the second filter register. */
        self.fb[nr].fr2   .write(|w| unsafe { w
            .bits( fr2 )
        });

        /* Select FIFO0 or FIFO1 as filter assignement. */
        self.ffa1r    .modify(|r,w| unsafe { w
            .bits ( if fifo != 0 { /* FIFO1 */
                    r.bits() | filter_select_bit
                } else { /* FIFO0 */
                    r.bits() &  !filter_select_bit
                } )
        });

        if enable {
            self.fa1r    .modify(|r,w| unsafe { w
                .bits ( r.bits() |  filter_select_bit ) /* Activate filter. */
            });
        }

        /* Request initialization "leave". */
        self.fmr   .modify(|_,w| w
            .finit()  .clear_bit()
        );
    }

    #[inline]
    fn filter_id_mask_16bit_init (&mut self, nr: usize, id1: u16,
                       mask1: u16, id2: u16,
                       mask2: u16, fifo: usize, enable: bool) {
        self.filter_init(nr, false, false,
                ((mask1 as u32) << 16) | (id1 as u32),
                ((mask2 as u32) << 16) | (id2 as u32), fifo, enable);
    }

    #[inline]
    fn filter_id_mask_32bit_init (&mut self, nr: usize, id: u32,
                       mask: u32, fifo: usize, enable: bool) {
        self.filter_init(nr, true, false, id, mask, fifo, enable);
    }

    #[inline]
    fn filter_id_list_16bit_init (&mut self, nr: usize,
                       id1: u16, id2: u16,
                       id3: u16, id4: u16,
                       fifo: usize, enable: bool) {
        self.filter_init(nr, false, true,
                ((id1 as u32) << 16) | (id2 as u32),
                ((id3 as u32) << 16) | (id4 as u32), fifo, enable);
    }

    #[inline]
    fn filter_id_list_32bit_init (&mut self, nr: usize,
                       id1: u32, id2: u32,
                       fifo: usize, enable: bool) {
        self.filter_init(nr, true, true, id1, id2, fifo, enable);
    }

    #[inline]
    fn enable_irq (&mut self, irq: u32) {
        self.ier    .modify(|r,w| unsafe { w
                .bits ( r.bits() | irq )
        });
    }

    #[inline]
    fn disable_irq (&mut self, irq: u32) {
        self.ier    .modify(|r,w| unsafe { w
                .bits ( r.bits() & !irq )
        });
    }

    #[inline]
    fn transmit (&mut self, id: u32, ext: bool, rtr: bool, length: u8,
             data: &[u8]) -> isize {
        
        /* Check which transmit mailbox is empty if any. */
        let ret = if self.tsr.read().tme0().bit_is_set() {
            0
        } else if self.tsr.read().tme1().bit_is_set() {
            1
        } else if self.tsr.read().tme2().bit_is_set() {
            2
        } else {
            -1
        };

        /* If we have no empty mailbox return with an error. */
        if ret == -1 {
            return ret;
        }
        
        let mut tdxr = EightBytes { data8: [0u8;8] };
        for i in 0..(length as usize) {
            unsafe { tdxr.data8[i] = data[i]; }
        }
        let tx = &self.tx[ret as usize];
        tx.tir   .modify(|_,w| unsafe {
            if ext { w.exid().bits( id )   /* Set extended ID. */
            } else { w.stid().bits( id as u16) } /* Set standard ID. */
        });
        /* Set/clear remote transmission request bit. */
        if rtr { tx.tir.modify(|_,w| w.rtr().set_bit()); }
        tx.tdtr    .modify(|_,w| unsafe { w
            .dlc()  .bits( length )
        });
        /* Set the data. */
        tx.tdlr.write(|w| unsafe { w.bits( tdxr.data32[0] ) });
        tx.tdhr.write(|w| unsafe { w.bits( tdxr.data32[1] ) });
        /* Request transmission. */
        tx.tir.modify(|_,w| w.txrq().set_bit());

        return ret;
    }

    #[inline]
    fn fifo_release (&mut self, fifo: usize) {
        self.rfr[fifo]   .modify(|_,w| w
            .rfom()  .set_bit()
        );
    }
    
    #[inline]
    fn receive (&mut self, fifo: usize, release: bool, timestamp: Option<u16>) -> (u32, bool, bool, u8, u8, [u8;8], Option<u16>) {
        let mut tstamp = timestamp;
        let rdxr: EightBytes;
        
        let rx = &self.rx[fifo];
        /* Get type of CAN ID and CAN ID. */
        let ext = rx.rir.read().ide().bit_is_set();
        /* Get extended CAN ID. */
        let id = if ext { rx.rir.read().exid().bits()
        /* Get standard CAN ID. */
        } else { rx.rir.read().stid().bits() as u32 };
        /* Get remote transmit flag. */
        let rtr = rx.rir.read().rtr().bit_is_set();
        /* Get filter match ID. */
        let fmi = rx.rdtr.read().fmi().bits();
        /* Get data length. */
        let length = rx.rdtr.read().dlc().bits();
        /* accelerate reception by copying the CAN data from the controller
         * memory to the fast internal RAM
         */
        if let Some(_) = timestamp {
            tstamp = Some( rx.rdtr.read().time().bits() );
        }
        /* Get data. */
        rdxr = EightBytes { data32: [
            rx.rdlr.read().bits(), rx.rdhr.read().bits() ] };
        
        let data = unsafe{ rdxr.data8 };

        /* Release the FIFO. */
        if release {
            self.fifo_release(fifo);
        }
        (id, ext, rtr, fmi, length, data, tstamp)
    }

    #[inline]
    fn available_mailbox (&self) -> bool {
        self.tsr.read().tme0().bit_is_set() ||
        self.tsr.read().tme1().bit_is_set() ||
        self.tsr.read().tme2().bit_is_set()
    }
}
