#![no_std]
#![feature(const_fn)]

#[macro_use]
mod macros;

pub use embedded_hal as hal;

#[cfg(feature="stm32f103")]
pub use stm32f103xx_hal::{
    self as device_hal,
    stm32f103xx as device
};

pub mod rcc;
//pub mod rcc_cld;
pub mod reset;

pub mod adc;
pub mod can;
pub mod crc;
pub mod dac;
pub mod dma;
pub mod flash;
/*pub mod gpio;
pub mod gpio_common;*/
pub mod i2c;
pub mod iwdg;
pub mod pwr;
//?pub mod rng;
pub mod rtc;
pub mod spi;
//?pub mod st_usbfs;
pub mod timer;
pub mod usart;



