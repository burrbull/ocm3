
use crate::device::RCC;
use crate::rcc::rst;
use crate::rcc::RccExt;


/// Peripheral Reset
///
/// The X peripheral and all its associated configuration registers are placed in
/// the reset condition. The reset is effective via the RCC peripheral reset
/// system.
pub trait Reset {
    fn reset(&self, rcc: &mut RCC);
}


macro_rules! impl_reset {
    ($PER:ty, $REG:expr) => (
        impl Reset for $PER {
            fn reset(&self, rcc: &mut RCC) {
                rcc.periph_reset_pulse($REG);// проверить
            }
        }
    )
}

use crate::device::CAN1;
use crate::device::{I2C1,I2C2};
use crate::device::{SPI1,SPI2,SPI3};
use crate::device::{TIM1,
                    TIM2,
                    TIM3,
                    TIM4,
                    TIM5,
                    TIM6,
                    TIM7,
                    TIM8,
                    TIM9,
                    TIM10,
                    TIM11,
                    TIM12,
                    TIM13,
                    TIM14};


impl_reset!(CAN1, rst::CAN1);
//impl_reset!(CAN2, rst::CAN2);
impl_reset!(I2C1, rst::I2C1);
impl_reset!(I2C2, rst::I2C2);
//impl_reset!(I2C3, rst::I2C3);
//impl_reset!(I2C4, rst::I2C4);
impl_reset!(SPI1, rst::SPI1);
impl_reset!(SPI2, rst::SPI2);
impl_reset!(SPI3, rst::SPI3);
//impl_reset!(SPI4, rst::SPI4);
//impl_reset!(SPI5, rst::SPI5);
//impl_reset!(SPI6, rst::SPI6);
impl_reset!(TIM1, rst::TIM1);
impl_reset!(TIM2, rst::TIM2);
impl_reset!(TIM3, rst::TIM3);
impl_reset!(TIM4, rst::TIM4);
impl_reset!(TIM5, rst::TIM5);
impl_reset!(TIM6, rst::TIM6);
impl_reset!(TIM7, rst::TIM7);
impl_reset!(TIM8, rst::TIM8);
impl_reset!(TIM9, rst::TIM9);
impl_reset!(TIM10, rst::TIM10);
impl_reset!(TIM11, rst::TIM11);
impl_reset!(TIM12, rst::TIM12);
impl_reset!(TIM13, rst::TIM13);
impl_reset!(TIM14, rst::TIM14);
