
pub const LSI_FREQUENCY : u32 = 32000;
const COUNT_LENGTH : u32 = 12;
const COUNT_MASK : u32 = (1 << COUNT_LENGTH)-1;


use crate::device::IWDG;

/// Key value (write-only, reads as 0x0000)
#[derive(Clone, Copy, PartialEq)]
pub enum Key {
    Reset  = 0xaaaa,
    Unlock = 0x5555,
    Start  = 0xcccc
}

/* PR[2:0]: Prescaler divider */
#[derive(Clone, Copy, PartialEq)]
pub enum Prescaler {
    Div4 = 0x0,
    Div8 = 0x1,
    Div16 = 0x2,
    Div32 = 0x3,
    Div64 = 0x4,
    Div128 = 0x5,
    Div256 = 0x6
}

pub trait IwdgExt {
    /// IWDG Enable Watchdog Timer
    ///
    /// The watchdog timer is started. The timeout period defaults to 512 milliseconds
    /// unless it has been previously defined.
    fn start(&mut self);

    /// IWDG Set Period in Milliseconds
    ///
    /// The countdown period is converted into count and prescale values. The maximum
    /// period is 32.76 seconds; values above this are truncated. Periods less than 1ms
    /// are not supported by this library.
    ///
    /// A delay of up to 5 clock cycles of the LSI clock (about 156 microseconds)
    /// can occasionally occur if the prescale or preload registers are currently busy
    /// loading a previous value.
    ///
    /// * `period : u32` - Period in milliseconds (< 32760) from a watchdog
    /// reset until a system reset is issued.
    fn set_period_ms(&mut self, period : u32);

    /// IWDG Get Reload Register Status
    /// 
    /// Returns `bool`: true if the reload register is busy and unavailable for
    /// loading a new count value.
    fn reload_busy(&self) -> bool;

    /// IWDG Get Prescaler Register Status
    ///
    /// Returns `bool`: true if the prescaler register is busy and unavailable for
    /// loading a new period value.
    fn prescaler_busy(&self) -> bool;

    /// IWDG reset Watchdog Timer
    ///
    /// The watchdog timer is reset. The counter restarts from the value in the reload
    /// register.
    fn reset(&mut self);
}

impl IwdgExt for IWDG {
    fn start(&mut self) {
        self.kr    .write(|w| unsafe { w
            .key()   .bits(Key::Start as u16)
        });
    }

    fn set_period_ms(&mut self, period : u32) {
        /* Set the count to represent ticks of the 32kHz LSI clock */
        let count = period << 5;

        /* Strip off the first 12 bits to get the prescale value required */
        let prescale = count >> 12;
        let (exponent, reload) = if prescale > 256 {
            (Prescaler::Div256, COUNT_MASK)
        } else if prescale > 128 {
            (Prescaler::Div256, count >> 8)
        } else if prescale > 64 {
            (Prescaler::Div128, count >> 7)
        } else if prescale > 32 {
            (Prescaler::Div64, count >> 6)
        } else if prescale > 16 {
            (Prescaler::Div32, count >> 5)
        } else if prescale > 8 {
            (Prescaler::Div16, count >> 4)
        } else if prescale > 4 {
            (Prescaler::Div8, count >> 3)
        } else {
            (Prescaler::Div4, count >> 2)
        };

        /* Avoid the undefined situation of a zero count */
        /*if count == 0 {
            count = 1;
        }*/

        while self.prescaler_busy() {};
        self.kr    .write(|w| unsafe { w
            .key()   .bits(Key::Unlock as u16)
        });
        self.pr    .write(|w| w
            .pr()   .bits(exponent as u8)
        );
        while self.reload_busy() {};
        self.kr    .write(|w| unsafe { w
            .key()   .bits(Key::Unlock as u16)
        });
        self.rlr    .write(|w| w
            .rl()   .bits((reload & COUNT_MASK) as u16)
        );
    }

    fn reload_busy(&self) -> bool {
        self.sr .read().rvu().bit_is_set()
    }

    fn prescaler_busy(&self) -> bool {
        self.sr .read().pvu().bit_is_set()
    }

    fn reset(&mut self)
    {
        self.kr    .write(|w| unsafe { w
            .key() .bits(Key::Reset as u16)
        });
    }
}
