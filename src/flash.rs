use core::ptr;
use crate::device::FLASH;

const FLASH_KEYR_KEY1 : u32 = 0x45670123;
const FLASH_KEYR_KEY2 : u32 = 0xcdef89ab;


pub use crate::device::flash::acr::LATENCYR as FlashLatency;

/* Read protection option byte protection enable key */
pub const FLASH_RDP_KEY : u16 = 0x00a5;



pub trait FlashExt {
    /// Enable the FLASH Prefetch Buffer
    ///
    /// This buffer is used for instruction fetches and is enabled by default after
    /// reset.
    ///
    /// Note carefully the clock restrictions under which the prefetch buffer may be
    /// enabled or disabled. Changes are normally made while the clock is running in
    /// the power-on low frequency mode before being set to a higher speed mode.
    /// See the reference manual for details.
    fn prefetch_buffer_enable(&mut self);

    /// Disable the FLASH Prefetch Buffer
    ///
    /// Note carefully the clock restrictions under which the prefetch buffer may be
    /// set to disabled. See the reference manual for details.
    fn prefetch_buffer_disable(&mut self);
    
    /// Set the Number of Wait States
    ///
    /// Used to match the system clock to the FLASH memory access time. See the
    /// reference manual for more information on clock speed ranges for each wait state.
    /// The latency must be changed to the appropriate value <b>before</b> any increase
    /// in clock speed, or <b>after</b> any decrease in clock speed.
    ///
    /// * `ws : FlashLatency`
    fn set_ws(&mut self, ws : FlashLatency);

    /// Unlock the Flash Program and Erase Controller
    ///
    /// This enables write access to the Flash memory. It is locked by default on
    /// reset.
    fn unlock(&mut self);

    /// Lock the Flash Program and Erase Controller
    ///
    /// Used to prevent spurious writes to FLASH.
    fn lock(&mut self);

    /// Clear the Programming Error Status Flag
    fn clear_pgerr_flag(&mut self);

    /// Clear the End of Operation Status Flag
    fn clear_eop_flag(&mut self);

    /// Clear the Write Protect Error Status Flag
    fn clear_wrprterr_flag(&mut self);

/*    /// Clear the Busy Status Flag
    fn clear_bsy_flag(&self);
*/
    /// Wait until Last Operation has Ended
    /// This loops indefinitely until an operation (write or erase) has completed by
    /// testing the busy flag.
    fn wait_for_last_operation(&mut self);

    /// Program a 32 bit Word to FLASH
    ///
    /// This performs all operations necessary to program a 32 bit word to FLASH memory.
    /// The program error flag should be checked separately for the event that memory
    /// was not properly erased.
    ///
    /// Status bit polling is used to detect end of operation.
    ///
    /// * `address : u32` - Full address of flash word to be programmed.
    /// * `data : u32 - data word to write
    fn program_word(&mut self, address : u32, data : u32);

    /// Unlock the Option Byte Access
    ///
    /// This enables write access to the option bytes. It is locked by default on
    /// reset.
    fn unlock_option_bytes(&mut self);

    /// Erase All Option Bytes
    ///
    /// This performs all operations necessary to erase the option bytes. These must
    /// first be fully erased before attempting to program them, therefore it is
    /// recommended to check these after an erase attempt.
    fn erase_option_bytes(&mut self);

    /// Program the Option Bytes
    ///
    /// This performs all operations necessary to program the option bytes.
    /// The write protect error flag should be checked separately for the event that
    /// an option byte had not been properly erased before calling this function.
    ///
    /// Only the lower 8 bits of the data is significant.
    ///
    /// * `address : u32` - Address of option byte from @ref flash_options.
    /// * `data : u16` - data value to write
    fn program_option_bytes(&mut self, address : u32, data : u16);
}


pub trait FlashF1Ext {
    /// Enable the FLASH Half Cycle Mode
    ///
    /// This mode is used for power saving during read access. It is disabled by default
    /// on reset.
    ///
    /// Note carefully the clock restrictions under which the half cycle mode may be
    /// enabled or disabled. This mode may only be used while the clock is running at
    /// 8MHz. See the reference manual for details.
    fn halfcycle_enable(&mut self);

    /// Disable the FLASH Half Cycle Mode
    fn halfcycle_disable(&mut self);
}

pub trait FlashF1LowHighExt {
    /// Clear All Status Flags
    ///
    /// Program error, end of operation, write protect error, busy. Both banks cleared.
    fn clear_status_flags(&mut self);

/*    /// Read All Status Flags
    ///
    /// The programming error, end of operation, write protect error and busy flags
    /// are returned in the order of appearance in the status register.
    ///
    /// Flags for the upper bank, where appropriate, are combined with those for
    /// the lower bank using bitwise OR, without distinction.
    ///
    /// @returns uint32_t. bit 0: busy, bit 2: programming error, bit 4: write protect
    /// error, bit 5: end of operation.
    fn get_status_flags(&self) -> u32;
*/
    /// Program a Half Word to FLASH
    ///
    /// This performs all operations necessary to program a 16 bit word to FLASH memory.
    /// The program error flag should be checked separately for the event that memory
    /// was not properly erased.
    ///
    /// Status bit polling is used to detect end of operation.
    ///
    /// * `address : u32` - Full address of flash half word to be programmed.
    /// * `data : u16` - data half word to write
    fn program_half_word(&mut self, address : u32, data : u16);

    /// Erase a Page of FLASH
    ///
    /// This performs all operations necessary to erase a page in FLASH memory.
    /// The page should be checked to ensure that it was properly erased. A page must
    /// first be fully erased before attempting to program it.
    ///
    /// Note that the page sizes differ between devices. See the reference manual or
    /// the FLASH programming manual for details.
    ///
    /// * `page_address : u32` - page_address Full address of flash page to be erased.
    fn erase_page(&mut self, page_address : u32);

    /// Erase All FLASH
    ///
    /// This performs all operations necessary to erase all user pages in the FLASH
    /// memory. The information block is unaffected.
    fn erase_all_pages(&mut self);
}

pub trait FlashF1HighExt {
    /// Unlock the Flash Program and Erase Controller, upper Bank
    ///
    /// This enables write access to the upper bank of the Flash memory in XL devices.
    /// It is locked by default on reset.
    fn unlock_upper(&mut self);

    /// Lock the Flash Program and Erase Controller, upper Bank
    ///
    /// Used to prevent spurious writes to FLASH.
    fn lock_upper(&mut self);

    /// Clear the Programming Error Status Flag, upper Bank
    fn clear_pgerr_flag_upper(&mut self);

    /// Clear the End of Operation Status Flag, upper Bank
    fn clear_eop_flag_upper(&mut self);

    /// Clear the Write Protect Error Status Flag, upper Bank
    fn clear_wrprterr_flag_upper(&mut self);

    /// Clear the Busy Status Flag, upper Bank
    fn clear_bsy_flag_upper(&mut self);
}


impl FlashExt for FLASH {
    fn prefetch_buffer_enable(&mut self) {
        self.acr   .modify(|_,w| w
            .prftbe()  .set_bit()
        );
    }

    fn prefetch_buffer_disable(&mut self) {
        self.acr   .modify(|_,w| w
            .prftbe()  .clear_bit()
        );
    }
    
    fn set_ws(&mut self, ws : FlashLatency) {
        self.acr   .modify(|_,w|  w
            .latency()  .variant( ws )
        );
    }

    fn unlock(&mut self) {
        /* Clear the unlock state. */
        self.cr   .modify(|_,w| w
            .lock()  .set_bit()
        );

        /* Authorize the FPEC access. */
        self.keyr    .write(|w| unsafe { w
            .bits( FLASH_KEYR_KEY1 )
        });
        self.keyr    .write(|w| unsafe { w
            .bits( FLASH_KEYR_KEY2 )
        });
    }

    fn lock(&mut self) {
        self.cr   .modify(|_,w| w
            .lock()  .set_bit()
        );
    }

    fn clear_pgerr_flag(&mut self) {
        self.sr   .modify(|_,w| w
            .pgerr()  .set_bit()
        );
    }

    fn clear_eop_flag(&mut self) {
        self.sr   .modify(|_,w| w
            .eop()  .set_bit()
        );
    }

    fn clear_wrprterr_flag(&mut self) {
        self.sr   .modify(|_,w| w
            .wrprterr()  .set_bit()
        );
    }
/*
    fn clear_bsy_flag(&mut self) {
        self.sr   .modify(|_,w| w
            .bsy()  .clear_bit()
        );
    }
*/
    fn wait_for_last_operation(&mut self) {
        while self.sr.read().bsy().bit_is_set() {};
    }

    fn program_word(&mut self, address : u32, data : u32) {
        self.program_half_word(address, data as u16);
        self.program_half_word(address+2, (data>>16) as u16);
    }

    fn unlock_option_bytes(&mut self) {
        /* F1 uses same keys for flash and option */
        self.optkeyr    .write(|w| unsafe { w
            .bits( FLASH_KEYR_KEY1 )
        });
        self.optkeyr    .write(|w| unsafe { w
            .bits( FLASH_KEYR_KEY2 )
        });
    }

    fn erase_option_bytes(&mut self) {
        self.wait_for_last_operation();

        if self.cr.read().optwre().bit_is_clear() {
            self.unlock_option_bytes();
        }

        self.cr   .modify(|_,w| w
            .opter()  .set_bit()    /* Enable option byte erase. */
            .strt()   .set_bit()
        );
        self.wait_for_last_operation();
        self.cr   .modify(|_,w| w
            .opter()  .clear_bit()    /* Disable option byte erase. */
        );
    }

    fn program_option_bytes(&mut self, address : u32, data : u16) {
        self.wait_for_last_operation();

        if self.cr.read().optwre().bit_is_clear() {
            self.unlock_option_bytes();
        }

        self.cr   .modify(|_,w| w
            .optpg()  .set_bit()    /* Enable option byte programming. */
        );
        
        unsafe {ptr::write_volatile(address as *mut u32, data as u32); }
        
        self.wait_for_last_operation();
        self.cr   .modify(|_,w| w
            .optpg()  .clear_bit()    /* Disable option byte programming. */
        );
    }

}


impl FlashF1Ext for FLASH {
    fn halfcycle_enable(&mut self) {
        self.acr   .modify(|_,w| w
            .hlfcya()  .set_bit()
        );
    }

    fn halfcycle_disable(&mut self) {
        self.acr   .modify(|_,w| w
            .hlfcya()  .clear_bit()
        );
    }
}


#[cfg(not(feature="flash512more"))]
impl FlashF1LowHighExt for FLASH {
    fn clear_status_flags(&mut self) {
        self.clear_pgerr_flag();
        self.clear_eop_flag();
        self.clear_wrprterr_flag();
//        self.clear_bsy_flag();
    }

/*    fn get_status_flags(&self) -> u32 {
        uint32_t flags = (FLASH_SR & (FLASH_SR_PGERR |
                FLASH_SR_EOP |
                FLASH_SR_WRPRTERR |
                FLASH_SR_BSY));

        return flags;
    }
*/
    fn program_half_word(&mut self, address : u32, data : u16) {
        self.wait_for_last_operation();


        self.cr   .modify(|_,w| w
            .pg()  .set_bit()
        );
        
        unsafe {ptr::write_volatile(address as *mut u32, data as u32); }

        self.wait_for_last_operation();
        
        self.cr   .modify(|_,w| w
            .pg()  .clear_bit()
        );
    }

    fn erase_page(&mut self, page_address : u32) {
        self.wait_for_last_operation();

        self.cr   .modify(|_,w| w
            .per()  .set_bit()
        );
        self.ar   .write(|w|  unsafe { w
            .bits( page_address )
        });
        self.cr   .modify(|_,w| w
            .strt()  .set_bit()
        );
        
        self.wait_for_last_operation();
        
        self.cr   .modify(|_,w| w
            .per()  .clear_bit()
        );
    }

    fn erase_all_pages(&mut self) {
        self.wait_for_last_operation();

        self.cr   .modify(|_,w| w
            .mer()  .set_bit()        /* Enable mass erase. */
        );
        self.cr   .modify(|_,w| w
            .strt()  .set_bit()        /* Trigger the erase. */
        );

        self.wait_for_last_operation();
        self.cr   .modify(|_,w| w
            .mer()  .clear_bit()    /* Disable mass erase. */
        );
    }
}

#[cfg(feature="flash512more")]
impl FlashF1LowHighExt for FLASH {
    fn clear_status_flags(&mut self) {
        self.clear_pgerr_flag();
        self.clear_eop_flag();
        self.clear_wrprterr_flag();
        self.clear_bsy_flag();
        self.clear_pgerr_flag_upper();
        self.clear_eop_flag_upper();
        self.clear_wrprterr_flag_upper();
        self.clear_bsy_flag_upper();
    }

    /*fn get_status_flags(&self) -> u32 {
        uint32_t flags = (FLASH_SR & (FLASH_SR_PGERR |
                FLASH_SR_EOP |
                FLASH_SR_WRPRTERR |
                FLASH_SR_BSY));
            flags |= (FLASH_SR2 & (FLASH_SR_PGERR |
                FLASH_SR_EOP |
                FLASH_SR_WRPRTERR |
                FLASH_SR_BSY));
        }

        return flags;
    }*/

    fn program_half_word(&mut self, address : u32, data : u16) {
        self.wait_for_last_operation();

        if address >= FLASH_BASE+0x00080000 {
            self.cr2   .modify(|_,w| w
                .pg()  .set_bit()
            );
        } else {
            self.cr   .modify(|_,w| w
                .pg()  .set_bit()
            );
        }
        
        unsafe {ptr::write_volatile(*address, data as u32); }

        self.wait_for_last_operation();

        if address >= FLASH_BASE+0x00080000 {
            self.cr2   .modify(|_,w| w
                .pg()  .clear_bit()
            );
        } else {
            self.cr   .modify(|_,w| w
                .pg()  .clear_bit()
            );
        }
    }

    fn erase_page(&mut self, page_address : u32) {
        self.wait_for_last_operation();

        if page_address >= FLASH_BASE+0x00080000 {
            self.cr2   .modify(|_,w| w
                .per()  .set_bit()
            );
            self.ar2   .write(|w| w
                .bits( page_address )
            );
            self.cr2   .modify(|_,w| w
                .strt()  .set_bit()
            );
        } else  {
            self.cr   .modify(|_,w| w
                .per()  .set_bit()
            );
            self.ar   .write(|w| w
                .bits( page_address )
            );
            self.cr   .modify(|_,w| w
                .strt()  .set_bit()
            );
        }
        
        self.wait_for_last_operation();

        if page_address >= FLASH_BASE+0x00080000 {
            self.cr2   .modify(|_,w| w
                .per()  .clear_bit()
            );
        } else {
            self.cr   .modify(|_,w| w
                .per()  .clear_bit()
            );
        }
    }

    fn erase_all_pages(&mut self) {
        self.wait_for_last_operation();

        self.cr   .modify(|_,w| w
            .mer()  .set_bit()        /* Enable mass erase. */
        );
        self.cr   .modify(|_,w| w
            .strt()  .set_bit()        /* Trigger the erase. */
        );

        self.wait_for_last_operation();
        self.cr   .modify(|_,w| w
            .mer()  .clear_bit()    /* Disable mass erase. */
        );
        
        /* Repeat for bank 2 */
        self.cr2   .modify(|_,w| w
            .mer()  .set_bit()
        );
        self.cr2   .modify(|_,w| w
            .strt()  .set_bit()
        );

        self.wait_for_last_operation();
        self.cr2   .modify(|_,w| w
            .mer()  .clear_bit()
        );
    }
}

#[cfg(feature="flash512more")]
impl FlashF1HighExt for FLASH {
    fn unlock_upper(&mut self) {
        /* Clear the unlock state. */
        self.cr2   .modify(|_,w| w
            .lock()  .clear_bit()
        );

        /* Authorize the FPEC access. */
        self.keyr2    .write(|w| w
        .bits( FLASH_KEYR_KEY1 )
        );
        self.keyr2    .write(|w| w
            .bits( FLASH_KEYR_KEY2 )
        );
    }

    fn lock_upper(&mut self) {
        self.cr2   .modify(|_,w| w
            .lock()  .set_bit()
        );
    }

    fn clear_pgerr_flag_upper(&mut self) {
        self.sr2   .modify(|_,w| w
            .pgerr()  .set_bit()
        );
    }

    fn clear_eop_flag_upper(&mut self) {
        self.sr2   .modify(|_,w| w
            .eop()  .set_bit()
        );
    }

    fn clear_wrprterr_flag_upper(&mut self) {
        self.sr2   .modify(|_,w| w
            .wrprterr()  .set_bit()
        );
    }

    fn clear_bsy_flag_upper(&mut self) {
        self.sr2   .modify(|_,w| w
            .bsy()  .clear_bit()
        );
    }
}
