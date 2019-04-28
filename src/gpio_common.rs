
/// GPIO number definitions (for convenience)
/// @defgroup gpio_pin_id GPIO Pin Identifiers
/// @ingroup gpio_defines
pub mod gpio {
    pub const P0 : u16 = 1 << 0;
    pub const P1 : u16 = 1 << 1;
    pub const P2 : u16 = 1 << 2;
    pub const P3 : u16 = 1 << 3;
    pub const P4 : u16 = 1 << 4;
    pub const P5 : u16 = 1 << 5;
    pub const P6 : u16 = 1 << 6;
    pub const P7 : u16 = 1 << 7;
    pub const P8 : u16 = 1 << 8;
    pub const P9 : u16 = 1 << 9;
    pub const P10 : u16 = 1 << 10;
    pub const P11 : u16 = 1 << 11;
    pub const P12 : u16 = 1 << 12;
    pub const P13 : u16 = 1 << 13;
    pub const P14 : u16 = 1 << 14;
    pub const P15 : u16 = 1 << 15;
    pub const ALL_PORT : u16 = 0xffff;
}
    
/// GPIO_LCKR[15:0]: LCKy: Port x lock bit y (y = 0..15)
const LCKK : u32 = 1 << 16;

trait GpioCommon {
    /// Set a Group of Pins Atomic
    ///
    /// Set one or more pins of the given GPIO port to 1 in an atomic operation.
    ///
    /// `gpios : u16`. Pin identifiers
    /// If multiple pins are to be changed, use bitwise OR '|' to separate
    /// them.
    fn set(&self, gpios : u16);
   
    /// Clear a Group of Pins Atomic
    ///
    /// Clear one or more pins of the given GPIO port to 0 in an atomic operation.
    ///
    /// `gpios : u16`. Pin identifiers
    /// If multiple pins are to be changed, use bitwise OR '|' to separate
    /// them.
    fn clear(&self, gpios : u16);
    
    ///  Read a Group of Pins.
    ///
    /// `gpios : u16`. Pin identifiers
    /// If multiple pins are to be changed, use bitwise OR '|' to separate
    /// them.
    /// return Unsigned int16 value of the pin values. The bit position of the pin
    /// value returned corresponds to the pin number.
    fn get(&self, gpios : u16) -> u16;
    
    ///  Toggle a Group of Pins
    ///
    /// Toggle one or more pins of the given GPIO port. The toggling is not atomic, but
    /// the non-toggled pins are not affected.
    ///
    /// `gpios : u16`. Pin identifiers
    /// If multiple pins are to be changed, use bitwise OR '|' to separate
    /// them.
    fn toggle(&self, gpios : u16);
    
    ///  Read from a Port
    ///
    /// Read the current value of the given GPIO port. Only the lower 16 bits contain
    /// valid pin data.
    ///
    /// return u16. The value held in the specified GPIO port.
    fn port_read(&self) -> u16;
    
    /// Write to a Port
    ///
    /// Write a value to the given GPIO port.
    ///
    /// `data : u16`. The value to be written to the GPIO port.
    fn port_write(&self, data : u16);
    
    /// Lock the Configuration of a Group of Pins
    ///
    /// The configuration of one or more pins of the given GPIO port is locked. There
    /// is no mechanism to unlock these via software. Unlocking occurs at the next
    /// reset.
    ///
    /// `gpios : u16`. Pin identifiers
    /// If multiple pins are to be changed, use bitwise OR '|' to separate
    /// them.
    fn port_config_lock(&self, gpios : u16);
}

use crate::device::gpioa;

use core::ops::Deref;
pub trait Gpio: Deref<Target = gpioa::RegisterBlock> { }

impl<G> GpioCommon for G
    where G: Gpio {
    fn set(&self, gpios : u16) {
        self.bsrr  .write(|w| unsafe { w
            .bits(gpios as u32)
        });
    }

    fn clear(&self, gpios : u16) {
        self.bsrr  .write(|w| unsafe { w
            .bits((gpios as u32) << 16)
        });
    }

    fn get(&self, gpios : u16) -> u16 {
        self.port_read() & gpios
    }

    fn toggle(&self, gpios : u16) {
        let port = self.odr.read().bits();
        self.bsrr  .write(|w| unsafe { w
            .bits(((port & gpios as u32) << 16) | (!port & gpios as u32))
        });
    }

    fn port_read(&self) -> u16 {
        self.idr  .read()
            .bits() as u16
    }

    fn port_write(&self, data : u16) {
        self.odr  .write(|w| unsafe { w
            .bits(data as u32)
        });
    }

    fn port_config_lock(&self, gpios : u16) {
        // Special "Lock Key Writing Sequence", see datasheet.
        let data = LCKK | (gpios as u32);
        self.lckr  .write(|w| unsafe { w
            .bits(data)    // Set LCKK.
        });
        self.lckr  .write(|w| unsafe { w
            .bits(!LCKK & (gpios as u32))    // Clear LCKK.
        });
        self.lckr  .write(|w| unsafe { w
            .bits(data)    // Set LCKK.
        });
        let _ = self.lckr.read().bits();      // Read LCKK.
        let _ = self.lckr.read().bits();      // Read LCKK again.

    /* Tell the compiler the variable is actually used. It will get
     * optimized out anyways.
     */
    // reg32 = reg32;

    // If (reg32 & GPIO_LCKK) is true, the lock is now active.
    }
}

