/** @defgroup gpio_file GPIO

@ingroup STM32F1xx

@brief <b>libopencm3 STM32F1xx General Purpose I/O</b>

@version 1.0.0

@author @htmlonly &copy; @endhtmlonly 2009
Uwe Hermann <uwe@hermann-uwe.de>
@author @htmlonly &copy; @endhtmlonly 2012
Ken Sarkies <ksarkies@internode.on.net>

@date 18 August 2012

Each I/O port has 16 individually configurable bits. Many I/O pins share GPIO
functionality with a number of alternate functions and must be configured to
the alternate function mode if these are to be accessed. A feature is available
to remap alternative functions to a limited set of alternative pins in the
event of a clash of requirements.

The data registers associated with each port for input and output are 32 bit
with the upper 16 bits unused. The output buffer must be written as a 32 bit
word, but individual bits may be set or reset separately in atomic operations
to avoid race conditions during interrupts. Bits may also be individually
locked to prevent accidental configuration changes. Once locked the
configuration cannot be changed until after the next reset.

Each port bit can be configured as analog or digital input, the latter can be
floating or pulled up or down. As outputs they can be configured as either
push-pull or open drain, digital I/O or alternate function, and with maximum
output speeds of 2MHz, 10MHz, or 50MHz.

On reset all ports are configured as digital floating input.

@section gpio_api_ex Basic GPIO Handling API.

Example 1: Push-pull digital output actions on ports C2 and C9

@code
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,  GPIO2 | GPIO9);
	gpio_set(GPIOC, GPIO2 | GPIO9);
	gpio_clear(GPIOC, GPIO2);
	gpio_toggle(GPIOC, GPIO2 | GPIO9);
	gpio_port_write(GPIOC, 0x204);
@endcode

Example 1: Digital input on port C12

@code
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT, GPIO12);
	reg16 = gpio_port_read(GPIOC);
@endcode
*/


use crate::device::{gpioa,AFIO};


pub use self::gpioa::crl::MODE0W as Mode;
pub use self::gpioa::crl::CNF0W as Cnf;

pub trait GpioSetModeExt {
	/// Set GPIO Pin Mode
	/// 
	/// Sets the mode (input/output) and configuration (analog/digitial and
	/// open drain/push pull), for a set of GPIO pins on a given GPIO port.
	/// 
	/// @param[in] gpioport Unsigned int32. Port identifier @ref gpio_port_id
	/// @param[in] mode Unsigned int8. Pin mode @ref gpio_mode
	/// @param[in] cnf Unsigned int8. Pin configuration @ref gpio_cnf
	/// @param[in] gpios Unsigned int16. Pin identifiers @ref gpio_pin_id
	///      If multiple pins are to be set, use bitwise OR '|' to separate
	///      them.
	fn set_mode(&self, mode : Mode, cnf : Cnf, gpios : u16);
}


use core::ops::Deref;

impl<G : Deref<Target = gpioa::RegisterBlock>> GpioSetModeExt for G {
    fn set_mode(&self, mode : Mode, cnf : Cnf, gpios : u16) {
	    /*
	     * We want to set the config only for the pins mentioned in gpios,
	     * but keeping the others, so read out the actual config first.
	     */
	    let mode = mode as u8;
	    let cnf  = cnf as u8;
	    
	    let mut crl = self.crl.read().bits();
	    let mut crh = self.crh.read().bits();

	    // Iterate over all bits, use i as the bitnumber.
	    for i in 0..16 {
		    // Only set the config if the bit is set in gpios.
		    if ((1 << i) & gpios) == 0 {
			    continue;
		    }

		    // Calculate bit offset.
		    let offset : u16 = if i < 8 { i * 4 } else { (i - 8) * 4 };

		    // Use tmp32 to either modify crl or crh.
		    let mut tmp32 = if i < 8 { crl } else { crh };

		    // Modify bits are needed.
		    tmp32 &= !(0xf << offset);	// Clear the bits first.
		    tmp32 |= ((mode as u32) << offset) | ((cnf as u32) << (offset + 2));

		    // Write tmp32 into crl or crh, leave the other unchanged.
		    if i < 8 {
			    crl = tmp32;
		    } else {
			    crh = tmp32;
		    }
	    }
        self.crl  .write(|w| unsafe { w
            .bits(crl)
        });
        self.crh  .write(|w| unsafe { w
            .bits(crh)
        });
    }
}


/* --- AFIO_EVCR values ---------------------------------------------------- */

/// PORT[2:0]: Port selection
/// EVENTOUT Port selection
#[derive(Clone, Copy, PartialEq)]
pub enum AfioEvcrPort {
    PA = 0x0,
    PB = 0x1,
    PC = 0x2,
    PD = 0x3,
    PE = 0x4
}

/// PIN[3:0]: Pin selection
/// EVENTOUT Pin selection
#[derive(Clone, Copy, PartialEq)]
pub enum AfioEvcrPin {
    Px0 = 0x0,
    Px1 = 0x1,
    Px2 = 0x2,
    Px3 = 0x3,
    Px4 = 0x4,
    Px5 = 0x5,
    Px6 = 0x6,
    Px7 = 0x7,
    Px8 = 0x8,
    Px9 = 0x9,
    Px10 = 0xA,
    Px11 = 0xB,
    Px12 = 0xC,
    Px13 = 0xD,
    Px14 = 0xE,
    Px15 = 0xF
}

pub trait AfioRemapExt {
    /// Map the EVENTOUT signal
    ///
    /// Enable the EVENTOUT signal and select the port and pin to be used.
    ///
    /// `evoutport : u8`. Port for EVENTOUT signal
    /// `evoutpin : u8`. Pin for EVENTOUT signal @ref afio_evcr_pin
    fn set_eventout(&self, evoutport : AfioEvcrPort, evoutpin : AfioEvcrPin);
    
    /// Map Alternate Function Port Bits (Main Set)
    ///
    /// A number of alternate function ports can be remapped to defined alternative
    /// port bits to avoid clashes in cases where multiple alternate functions are
    /// present.  Refer to the datasheets for the particular mapping desired. This
    /// provides the main set of remap functionality. See @ref gpio_secondary_remap for
    /// a number of lesser used remaps.
    ///
    /// The AFIO remapping feature is used only with the STM32F10x series.
    ///
    /// @note The Serial Wire JTAG disable controls allow certain GPIO ports to become
    /// available in place of some of the SWJ signals. Full SWJ capability is obtained
    /// by setting this to zero. The value of this must be specified for every call to
    /// this function as its current value cannot be ascertained from the hardware.
    ///
    /// @param[in] swjdisable Unsigned int8. Disable parts of the SWJ capability @ref
    /// afio_swj_disable.
    /// @param[in] maps Unsigned int32. Bitwise OR of map enable controls you wish to
    /// enable from @ref afio_remap, @ref afio_remap_can1, @ref afio_remap_tim3,
    /// @ref afio_remap_tim2, @ref afio_remap_tim1, @ref afio_remap_usart3. For
    /// connectivity line devices only @ref afio_remap_cld are also available.
    fn primary_remap(&self, swjdisable : u32, maps : u32);
    
    /// Map Alternate Function Port Bits (Secondary Set)
    ///
    /// A number of alternate function ports can be remapped to defined alternative
    /// port bits to avoid clashes in cases where multiple alternate functions are
    /// present.  Refer to the datasheets for the particular mapping desired. This
    /// provides the second smaller and less used set of remap functionality. See @ref
    /// gpio_primary_remap for the main set of remaps.
    ///
    /// The AFIO remapping feature is used only with the STM32F10x series.
    ///
    /// @param[in] maps Unsigned int32. Bitwise OR of map enable controls from @ref
    /// afio_remap2
    fn secondary_remap(&self, maps : u32);
}

impl AfioRemapExt for AFIO {
    fn set_eventout(&self, evoutport : AfioEvcrPort, evoutpin : AfioEvcrPin) {
	self.evcr    .modify(|_,w| unsafe { w
	    .evoe()  .set_bit()
	    .port()  .bits(evoutport as u8)
	    .pin()   .bits(evoutpin as u8)
	});
}

    fn primary_remap(&self, swjdisable : u32, maps : u32) {
	/*
	 * the SWJ_CFG bits are write only.  (read is explicitly undefined)
	 * To be sure we set only the bits we want we must clear them first.
	 * However, we are still trying to only enable the map bits desired.
	 */
	// uint32_t reg = AFIO_MAPR & ~AFIO_MAPR_SWJ_MASK;
	// AFIO_MAPR = reg | swjdisable | maps;
	
	    self.mapr     .modify(|_,w| unsafe { w
	        .swj_cfg()    .bits(swjdisable as u8)
	        .bits(maps)
	    });
    }

    fn secondary_remap(&self, maps : u32) {
	    self.mapr2    .modify(|_,w| unsafe { w
	        .bits(maps)
	    });
    }

}

//extern crate cortex_m;
//use cortex_m::peripheral::Peripheral;

use core::any::Any;

use crate::device::{GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG};

/// GPIO Ports Enum
#[derive(Clone, Copy, PartialEq)]
pub enum GPIOPort {
   /// Port A
   A,
   /// Port B
   B,
   /// Port C
   C,
   /// Port D
   D,
   /// Port E
   E,
   /// Port F
   F,
   /// Port G
   G,
}

/// GPIO Peripheral Trait
pub trait GpioPort {
    /// Associated GPIO Port
    const PORT: GPIOPort;
}

pub struct PA(u8);
pub struct PB(u8);
pub struct PC(u8);
pub struct PD(u8);
pub struct PE(u8);
pub struct PF(u8);
pub struct PG(u8);

impl GpioPort for PA {
    const PORT: GPIOPort = GPIOPort::A;
}
impl GpioPort for PB {
    const PORT: GPIOPort = GPIOPort::B;
} 
impl GpioPort for PC {
    const PORT: GPIOPort = GPIOPort::C;
}
impl GpioPort for PD {
    const PORT: GPIOPort = GPIOPort::D;
}
impl GpioPort for PE {
    const PORT: GPIOPort = GPIOPort::E;
}
impl GpioPort for PF {
    const PORT: GPIOPort = GPIOPort::F;
}
impl GpioPort for PG {
    const PORT: GPIOPort = GPIOPort::G;
}

#[derive(Clone, Copy, PartialEq)]
/// Possible modes for a GPIO pin
pub enum GPIOMode {
    /// Input mode (reset state)
    Input(InputConfig),
    /// Output mode, max speed 10 MHz.
    Output(OutputConfig),
    /// Output mode, max speed 2 MHz.
    Output2(OutputConfig),
    /// Output mode, max speed 50 MHz
    Output50(OutputConfig),
}

impl GPIOMode {
	pub fn to_mode(&self) -> u8 {
		match *self {
			GPIOMode::Input(_) => 0b00,
			GPIOMode::Output(_) => 0b01,
			GPIOMode::Output2(_) => 0b10,
			GPIOMode::Output50(_) => 0b11,
		}
    }
	pub fn to_cnf(&self) -> u8 {
		match *self {
			GPIOMode::Input(config) => config.to_cnf(),
			GPIOMode::Output(config) => config.to_cnf(),
			GPIOMode::Output2(config) => config.to_cnf(),
			GPIOMode::Output50(config) => config.to_cnf(),
		}
	}
}

/*
// Associated constants for easy access to the most common mode / config combinations
impl GPIOMode {
    /// Output mode, general purpose push-pull
    pub const OUTPUT: GPIOMode = GPIOMode::Output(OutputConfig::PushPull);
    /// Input mode, pull-up
    pub const INPUT_PULL_UP: GPIOMode = GPIOMode::Input(InputConfig::PullUp);
    /// Input mode, pull-down
    pub const INPUT_PULL_DOWN: GPIOMode = GPIOMode::Input(InputConfig::PullDown);
}*/

#[derive(Clone, Copy, PartialEq)]
/// Possible configurations for a GPIO pin in input mode
pub enum InputConfig {
    /// Analog mode
    Analog,
    /// Floating input (reset state)
    Floating,
    /// Input with pull-up
    PullUp,
    /// Input with pull-down
    PullDown
}

impl InputConfig {
	fn to_cnf(&self) -> u8 {
		match *self {
			InputConfig::Analog => 0b00,
			InputConfig::Floating => 0b01,
			InputConfig::PullUp => 0b10,
			InputConfig::PullDown => 0b10,
		}
	}
}

#[derive(Clone, Copy, PartialEq)]
/// Possible configurations for a GPIO pin in output mode
pub enum OutputConfig {
    /// General purpose output Push-pull
    PushPull,
    /// General purpose output Open-drain
    OpenDrain,
    /// Alternate function output Push-pull
    AltPushPull,
    /// Alternate function output Open-drain
    AltOpenDrain,
}

impl OutputConfig {
	fn to_cnf(&self) -> u8 {
		match *self {
			OutputConfig::PushPull => 0b00,
			OutputConfig::OpenDrain => 0b01,
			OutputConfig::AltPushPull => 0b10,
			OutputConfig::AltOpenDrain => 0b11,
		}
    }
}



pub trait GpioId {
	fn pinid(&self) -> u8;
	fn portid(&self) -> GPIOPort;
}

pub trait Pin<'a> : Any+GpioId {
	fn port(&self) -> &'a Deref<Target = gpioa::RegisterBlock> {
		match self.portid() {
			GPIOPort::A => unsafe { &*GPIOA.get() },
			GPIOPort::B => unsafe { &*GPIOB.get() },
			GPIOPort::C => unsafe { &*GPIOC.get() },
			GPIOPort::D => unsafe { &*GPIOD.get() },
			GPIOPort::E => unsafe { &*GPIOE.get() },
			GPIOPort::F => unsafe { &*GPIOF.get() },
			GPIOPort::G => unsafe { &*GPIOG.get() },
		}
	}
	fn p(&self) -> u16 {
		1 << self.pinid()
	}
	fn set_high(&self) {
		self.port().bsrr.write(|w| unsafe { w.bits(self.p() as u32) });
	}
	fn set_low(&self) {
		self.port().brr .write(|w| unsafe { w.bits(self.p() as u32) });
	}
	fn is_input_high(&self) -> bool {
		!self.is_input_low()
	}
	fn is_input_low(&self) -> bool {
		(self.port().idr.read().bits() as u16) & self.p() == 0
	}
	fn is_output_high(&self) -> bool {
		!self.is_input_low()
	}
	fn is_output_low(&self) -> bool {
		(self.port().odr.read().bits() as u16) & self.p() == 0
	}
    
    fn _set_mode(&self, mode : u8, cnf : u8) {
		let i = self.pinid() as u16;
	    let offset : u16 = if i < 8 { i * 4 } else { (i - 8) * 4 };
		if i < 8 {
			self.port().crl.modify(|r, w| unsafe { w
				.bits( r.bits()
						& !(0xf << offset)
						| ((mode as u32) << offset)
						| ((cnf as u32) << (offset + 2)) )
			});
		} else  {
			self.port().crh.modify(|r, w| unsafe { w
				.bits( r.bits()
						& !(0xf << offset)
						| ((mode as u32) << offset)
						| ((cnf as u32) << (offset + 2)))
			});
		}
	}
	
	fn set_mode(&self, mode : Mode, cnf : Cnf) {
		self._set_mode(mode as u8, cnf as u8);
	}
    
    fn set_mode2(&self, mode: GPIOMode) {
		self._set_mode(mode.to_mode(), mode.to_cnf());
		match mode {
			GPIOMode::Input(InputConfig::PullDown) => self.set_low(),
			GPIOMode::Input(InputConfig::PullUp)   => self.set_high(),
			_ => {}
		}
	}
	
	fn equ(&self, other: &Pin) -> bool {
		(self.portid() == other.portid()) && (self.pinid() == other.pinid())
	}
}

macro_rules! gpiopin {
    ($pty:ty) => (
		impl GpioId for $pty {
			fn pinid(&self) -> u8 {
				self.0
			}
			fn portid(&self) -> GPIOPort {
				Self::PORT
			}
		}
		impl<'a, P> PartialEq<P> for $pty where P : Pin<'a> {
			fn eq(&self, other: &P) -> bool {
				(self.portid() == other.portid()) && (self.pinid() == other.pinid())
			}
		}
		impl<'a> Pin<'a> for $pty {
		}
	)
}

gpiopin!(PA);
gpiopin!(PB);
gpiopin!(PC);
gpiopin!(PD);
gpiopin!(PE);
gpiopin!(PF);
gpiopin!(PG);

/*
impl<'a, P> PartialEq for P where P : Pin<'a> {
	fn eq(&self, other: &P) -> bool {
		(self.portid() == other.portid()) && (self.pinid() == other.pinid())
	}
}*/

pub mod pins {
    use super::{PA,PB,PC,PD,PE};
    pub const PA0  : PA = PA(0);
    pub const PA1  : PA = PA(1);
    pub const PA2  : PA = PA(2);
    pub const PA3  : PA = PA(3);
    pub const PA4  : PA = PA(4);
    pub const PA5  : PA = PA(5);
    pub const PA6  : PA = PA(6);
    pub const PA7  : PA = PA(7);
    pub const PA8  : PA = PA(8);
    pub const PA9  : PA = PA(9);
    pub const PA10 : PA = PA(10);
    pub const PA11 : PA = PA(11);
    pub const PA12 : PA = PA(12);
    pub const PA13 : PA = PA(13);
    pub const PA14 : PA = PA(14);
    pub const PA15 : PA = PA(15);
    pub const PB0  : PB = PB(0);
    pub const PB1  : PB = PB(1);
    pub const PB2  : PB = PB(2);
    pub const PB3  : PB = PB(3);
    pub const PB4  : PB = PB(4);
    pub const PB5  : PB = PB(5);
    pub const PB6  : PB = PB(6);
    pub const PB7  : PB = PB(7);
    pub const PB8  : PB = PB(8);
    pub const PB9  : PB = PB(9);
    pub const PB10 : PB = PB(10);
    pub const PB11 : PB = PB(11);
    pub const PB12 : PB = PB(12);
    pub const PB13 : PB = PB(13);
    pub const PB14 : PB = PB(14);
    pub const PB15 : PB = PB(15);
    pub const PC0  : PC = PC(0);
    pub const PC1  : PC = PC(1);
    pub const PC2  : PC = PC(2);
    pub const PC3  : PC = PC(3);
    pub const PC4  : PC = PC(4);
    pub const PC5  : PC = PC(5);
    pub const PC6  : PC = PC(6);
    pub const PC7  : PC = PC(7);
    pub const PC8  : PC = PC(8);
    pub const PC9  : PC = PC(9);
    pub const PC10 : PC = PC(10);
    pub const PC11 : PC = PC(11);
    pub const PC12 : PC = PC(12);
    pub const PC13 : PC = PC(13);
    pub const PC14 : PC = PC(14);
    pub const PC15 : PC = PC(15);
    pub const PD0  : PD = PD(0);
    pub const PD1  : PD = PD(1);
    pub const PD2  : PD = PD(2);
    pub const PD3  : PD = PD(3);
    pub const PD4  : PD = PD(4);
    pub const PD5  : PD = PD(5);
    pub const PD6  : PD = PD(6);
    pub const PD7  : PD = PD(7);
    pub const PD8  : PD = PD(8);
    pub const PD9  : PD = PD(9);
    pub const PD10 : PD = PD(10);
    pub const PD11 : PD = PD(11);
    pub const PD12 : PD = PD(12);
    pub const PD13 : PD = PD(13);
    pub const PD14 : PD = PD(14);
    pub const PD15 : PD = PD(15);
    pub const PE0  : PE = PE(0);
    pub const PE1  : PE = PE(1);
    pub const PE2  : PE = PE(2);
    pub const PE3  : PE = PE(3);
    pub const PE4  : PE = PE(4);
    pub const PE5  : PE = PE(5);
    pub const PE6  : PE = PE(6);
    pub const PE7  : PE = PE(7);
    pub const PE8  : PE = PE(8);
    pub const PE9  : PE = PE(9);
    pub const PE10 : PE = PE(10);
    pub const PE11 : PE = PE(11);
    pub const PE12 : PE = PE(12);
    pub const PE13 : PE = PE(13);
    pub const PE14 : PE = PE(14);
    pub const PE15 : PE = PE(15);
}

pub mod constants {
    use super::{PA,PB,PC,PD,PE};
    use super::pins::*;
    
    /* CAN1 / CAN GPIO */
    pub const CAN1_RX       : PA = PA11;
    pub const CAN1_TX       : PA = PA12;
    pub const CAN_RX        : PA = CAN1_RX;
    pub const CAN_TX        : PA = CAN1_TX;
    
    pub const CAN_PB_RX     : PB = PB8;
    pub const CAN_PB_TX     : PB = PB9;
    pub const CAN1_PB_RX    : PB = CAN_PB_RX;
    pub const CAN1_PB_TX    : PB = CAN_PB_TX;
    
    pub const CAN_PD_RX     : PD = PD0;
    pub const CAN_PD_TX     : PD = PD1;
    pub const CAN1_PD_RX    : PD = CAN_PD_RX;
    pub const CAN1_PD_TX    : PD = CAN_PD_TX;
    
    /* CAN2 GPIO */
    pub const CAN2_RX       : PB = PB12;
    pub const CAN2_TX       : PB = PB13;
    pub const CAN2_RE_RX    : PB = PB5;
    pub const CAN2_RE_TX    : PB = PB6;
    
    /* JTAG/SWD GPIO */
    pub const JTMS_SWDIO    : PA = PA13;
    pub const JTCK_SWCLK    : PA = PA14;
    pub const JTDI          : PA = PA15;
    pub const JTDO_TRACESWO : PB = PB3;
    pub const JNTRST        : PB = PB4;
    pub const TRACECK       : PE = PE2;
    pub const TRACED0       : PE = PE3;
    pub const TRACED1       : PE = PE4;
    pub const TRACED2       : PE = PE5;
    pub const TRACED3       : PE = PE6;


    /* Timer5 GPIO */
    pub const TIM5_CH1      : PA = PA0;
    pub const TIM5_CH2      : PA = PA1;
    pub const TIM5_CH3      : PA = PA2;
    pub const TIM5_CH4      : PA = PA3;

    /* Timer4 GPIO */
    pub const TIM4_CH1      : PB = PB6;
    pub const TIM4_CH2      : PB = PB7;
    pub const TIM4_CH3      : PB = PB8;
    pub const TIM4_CH4      : PB = PB9;

    pub const TIM4_RE_CH1   : PD = PD12;
    pub const TIM4_RE_CH2   : PD = PD13;
    pub const TIM4_RE_CH3   : PD = PD14;
    pub const TIM4_RE_CH4   : PD = PD15;

    /* Timer3 GPIO */
    pub const TIM3_CH1      : PA = PA6;
    pub const TIM3_CH2      : PA = PA7;
    pub const TIM3_CH3      : PB = PB0;
    pub const TIM3_CH4      : PB = PB1;

    pub const TIM3_PR_CH1   : PB = PB4;
    pub const TIM3_PR_CH2   : PB = PB5;
    pub const TIM3_PR_CH3   : PB = PB0;
    pub const TIM3_PR_CH4   : PB = PB1;

    pub const TIM3_FR_CH1   : PC = PC6;
    pub const TIM3_FR_CH2   : PC = PC7;
    pub const TIM3_FR_CH3   : PC = PC8;
    pub const TIM3_FR_CH4   : PC = PC9;

    /* Timer2 GPIO */
    pub const TIM2_CH1_ETR  : PA = PA0;
    pub const TIM2_CH2      : PA = PA1;
    pub const TIM2_CH3      : PA = PA2;
    pub const TIM2_CH4      : PA = PA3;

    pub const TIM2_PR1_CH1_ETR : PA = PA15;
    pub const TIM2_PR1_CH2  : PB = PB3;
    pub const TIM2_PR1_CH3  : PA = PA2;
    pub const TIM2_PR1_CH4  : PA = PA3;

    pub const TIM2_PR2_CH1_ETR : PA = PA0;
    pub const TIM2_PR2_CH2  : PA = PA1;
    pub const TIM2_PR2_CH3  : PB = PB10;
    pub const TIM2_PR2_CH4  : PB = PB11;

    pub const TIM2_FR_CH1_ETR : PA = PA15;
    pub const TIM2_FR_CH2   : PB = PB3;
    pub const TIM2_FR_CH3   : PB = PB10;
    pub const TIM2_FR_CH4   : PB = PB11;

    /* Timer1 GPIO */
    pub const TIM1_ETR      : PA = PA12;
    pub const TIM1_CH1      : PA = PA8;
    pub const TIM1_CH2      : PA = PA9;
    pub const TIM1_CH3      : PA = PA10;
    pub const TIM1_CH4      : PA = PA11;
    pub const TIM1_BKIN     : PB = PB12;
    pub const TIM1_CH1N     : PB = PB13;
    pub const TIM1_CH2N     : PB = PB14;
    pub const TIM1_CH3N     : PB = PB15;

    pub const TIM1_PR_ETR   : PA = PA12;
    pub const TIM1_PR_CH1   : PA = PA8;
    pub const TIM1_PR_CH2   : PA = PA9;
    pub const TIM1_PR_CH3   : PA = PA10;
    pub const TIM1_PR_CH4   : PA = PA11;
    pub const TIM1_PR_BKIN  : PA = PA6;
    pub const TIM1_PR_CH1N  : PA = PA7;
    pub const TIM1_PR_CH2N  : PB = PB0;
    pub const TIM1_PR_CH3N  : PB = PB1;

    pub const TIM1_FR_ETR   : PE = PE7;
    pub const TIM1_FR_CH1   : PE = PE9;
    pub const TIM1_FR_CH2   : PE = PE11;
    pub const TIM1_FR_CH3   : PE = PE13;
    pub const TIM1_FR_CH4   : PE = PE14;
    pub const TIM1_FR_BKIN  : PE = PE15;
    pub const TIM1_FR_CH1N  : PE = PE8;
    pub const TIM1_FR_CH2N  : PE = PE10;
    pub const TIM1_FR_CH3N  : PE = PE12;

   /* UART5 GPIO */
    pub const UART5_TX      : PC = PC12;
    pub const UART5_RX      : PD = PD2;


    /* UART4 GPIO */
    pub const UART4_TX      : PC = PC10;
    pub const UART4_RX      : PC = PC11;


    /* USART3 GPIO */
    pub const USART3_TX     : PB = PB10;
    pub const USART3_RX     : PB = PB11;
    pub const USART3_CK     : PB = PB12;
    pub const USART3_CTS    : PB = PB13;
    pub const USART3_RTS    : PB = PB14;

    pub const USART3_PR_TX  : PC = PC10;
    pub const USART3_PR_RX  : PC = PC11;
    pub const USART3_PR_CK  : PC = PC12;
    pub const USART3_PR_CTS : PB = PB13;
    pub const USART3_PR_RTS : PB = PB14;

    pub const USART3_FR_TX  : PD = PD8;
    pub const USART3_FR_RX  : PD = PD9;
    pub const USART3_FR_CK  : PD = PD10;
    pub const USART3_FR_CTS : PD = PD11;
    pub const USART3_FR_RTS : PD = PD12;


    /* USART2 GPIO */
    pub const USART2_CTS    : PA = PA0;
    pub const USART2_RTS    : PA = PA1;
    pub const USART2_TX     : PA = PA2;
    pub const USART2_RX     : PA = PA3;
    pub const USART2_CK     : PA = PA4;

    pub const USART2_RE_CTS : PD = PD3;
    pub const USART2_RE_RTS : PD = PD4;
    pub const USART2_RE_TX  : PD = PD5;
    pub const USART2_RE_RX  : PD = PD6;
    pub const USART2_RE_CK  : PD = PD7;


    /* USART1 GPIO */
    pub const USART1_TX     : PA = PA9;
    pub const USART1_RX     : PA = PA10;

    pub const USART1_RE_TX  : PB = PB6;
    pub const USART1_RE_RX  : PB = PB7;


    /* I2C1 GPIO */
    pub const I2C1_SMBAI    : PB = PB5;
    pub const I2C1_SCL      : PB = PB6;
    pub const I2C1_SDA      : PB = PB7;

    pub const I2C1_RE_SMBAI : PB = PB5;
    pub const I2C1_RE_SCL   : PB = PB8;
    pub const I2C1_RE_SDA   : PB = PB9;


    /* I2C2 GPIO */
    pub const I2C2_SCL      : PB = PB10;
    pub const I2C2_SDA      : PB = PB11;
    pub const I2C2_SMBAI    : PB = PB12;


    /* SPI1 GPIO */
    pub const SPI1_NSS      : PA = PA4;
    pub const SPI1_SCK      : PA = PA5;
    pub const SPI1_MISO     : PA = PA6;
    pub const SPI1_MOSI     : PA = PA7;

    pub const SPI1_RE_NSS   : PA = PA15;
    pub const SPI1_RE_SCK   : PB = PB3;
    pub const SPI1_RE_MISO  : PB = PB4;
    pub const SPI1_RE_MOSI  : PB = PB5;

    /* SPI2 GPIO */
    pub const SPI2_NSS      : PB = PB12;
    pub const SPI2_SCK      : PB = PB13;
    pub const SPI2_MISO     : PB = PB14;
    pub const SPI2_MOSI     : PB = PB15;

    /* SPI3 GPIO */
    pub const SPI3_NSS      : PA = PA15;
    pub const SPI3_SCK      : PB = PB3;
    pub const SPI3_MISO     : PB = PB4;
    pub const SPI3_MOSI     : PB = PB5;

    pub const SPI3_RE_NSS   : PA = PA4;
    pub const SPI3_RE_SCK   : PC = PC10;
    pub const SPI3_RE_MISO  : PC = PC11;
    pub const SPI3_RE_MOSI  : PC = PC12;


    /* ETH GPIO */
    pub const ETH_RX_DV_CRS_DV : PA = PA7;
    pub const ETH_RXD0      : PC = PC4;
    pub const ETH_RXD1      : PC = PC5;
    pub const ETH_RXD2      : PB = PB0;
    pub const ETH_RXD3      : PB = PB1;

    pub const ETH_RE_RX_DV_CRS_DV : PD = PD8;
    pub const ETH_RE_RXD0   : PD = PD9;
    pub const ETH_RE_RXD1   : PD = PD10;
    pub const ETH_RE_RXD2   : PD = PD11;
    pub const ETH_RE_RXD3   : PD = PD12;
}
