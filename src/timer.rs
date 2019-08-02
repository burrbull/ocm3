//! General Purpose and Advanced Control Timers for
//! the STM32 series of ARM Cortex Microcontrollers by ST Microelectronics.
//! 
//! The STM32 series have four general purpose timers (2-5), while some have
//! an additional two advanced timers (1,8), and some have two basic timers (6,7).
//! Some of the larger devices have additional general purpose timers (9-14).
//! 
//! @todo Add timer DMA burst settings
//! 
//! # Basic TIMER handling API
//! 
//! Enable the timer clock first. The timer mode sets the clock division ratio, the
//! count alignment (edge or centred) and count direction. Finally enable the
//! timer.
//! 
//! The timer output compare block produces a signal that can be configured for
//! output to a pin or passed to other peripherals for use as a trigger. In all
//! cases the output compare mode must be set to define how the output responds to
//! a compare match, and the output must be enabled. If output to a pin is
//! required, enable the appropriate GPIO clock and set the pin to alternate output
//! mode.
//! 
//! Example: Timer 2 with 2x clock divide, edge aligned and up counting.
//! ```
//! rcc.periph_clock_enable(RCC_TIM2);
//! tim2.reset();
//! tim2.set_mode(ClockDivision::DIV2,
//!            Alignment::EDGE, Direction::UP);
//! ...
//! tim2.set_period(1000);
//! tim2.enable_counter();
//! ```
//! 
//! Example: Timer 1 with PWM output, no clock divide and centre alignment. Set the
//! Output Compare mode to PWM and enable the output of channel 1. Note that for
//! the advanced timers the break functionality must be enabled before the signal
//! will appear at the output, even though break is not being used. This is in
//! addition to the normal output enable. Enable the alternate function clock (APB2
//! only) and port A clock. Set ports A8 and A9 (timer 1 channel 1 compare outputs)
//! to alternate function push-pull outputs where the PWM output will appear.
//! 
//! ```
//! rcc.periph_clock_enable(rcc::en::GPIOA);
//! rcc.periph_clock_enable(rcc::en::TIM1);
//! gpio.set_output_options(GPIOA, GPIO_OTYPE_PP,
//!             GPIO_OSPEED_50MHZ, GPIO8 | GPIO9);
//! rcc.periph_clock_enable(RCC_TIM1);
//! tim1.reset();
//! tim1.set_mode(ClockDivision::NODIV, Alignment::CENTER_1,
//!            Direction::UP);
//! tim1.set_oc_mode(OcId::OC1, OcMode::PWM2);
//! tim1.enable_oc_output(OcId::OC1);
//! tim1.enable_break_main_output();
//! tim1.set_oc_value(OcId::OC1, 200);
//! tim1.set_period(1000);
//! tim1.enable_counter();
//! ```
//! 
//! Example: Timer 3 as a Quadrature encoder counting input from a motor or control
//! knob.
//! ```
//! rcc.periph_clock_enable(RCC_TIM3);
//! tim3.set_period(1024);
//! tim3.slave_set_mode(SlaveMode::ENCODERTI1TI2); // encoder
//! tim3.ic_set_input(IcId::IC1, IcInput::IN_TI1);
//! tim3.ic_set_input(IcId::IC2, IcInput::IN_TI2);
//! tim3.enable_counter();
//! ...
//! int motor_pos = tim3.get_count();
//! ```
//! 
//! @todo input capture example
//! 


/*
 * Basic TIMER handling API.
 *
 * Examples:
 *  timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT_MUL_2,
 *                 TIM_CR1_CMS_CENTRE_3, TIM_CR1_DIR_UP);
 */

use crate::device::{TIM1,
                    TIM2};

use crate::device::{tim1, tim2};

/// Output Compare mode designators
pub use self::tim1::ccmr1_output::OC1MW as OcMode;

/// Input Capture input filter. The frequency used to sample the
/// input and the number of events needed to validate an output transition.
///
/// TIM_IC_CK_INT_N_x No division from the Deadtime and Sampling Clock frequency
/// (DTF), filter length x
/// TIM_IC_DTF_DIV_y_N_x Division by y from the DTF, filter length x
pub use self::tim1::ccmr1_input::IC1FR as IcFilter;

pub trait GeneralEnums {
    /// Slave mode selection
    type SlaveMode;
    /// Clock division
    type ClockDivision;
    /// Direction
    type Direction;
    /// Trigger selection
    type TriggerSelection;
}

impl GeneralEnums for TIM1 {
    type SlaveMode = self::tim1::smcr::SMSW;
    type ClockDivision = self::tim1::cr1::CKDW;
    type Direction = self::tim1::cr1::DIRW;
    type TriggerSelection = self::tim1::smcr::TSW;
}

impl GeneralEnums for TIM2 {
    type SlaveMode = self::tim2::smcr::SMSW;
    type ClockDivision = self::tim2::cr1::CKDW;
    type Direction = self::tim2::cr1::DIRW;
    type TriggerSelection = self::tim2::smcr::TSW;
}


/* --- TIMx convenience defines -------------------------------------------- */

/// Output Compare channel designators
#[derive(Clone, Copy, PartialEq)]
pub enum OcId {
    OC1,
    OC1N,
    OC2,
    OC2N,
    OC3,
    OC3N,
    OC4,
}

/// Input Capture channel designators
#[derive(Clone, Copy, PartialEq)]
pub enum IcId {
    IC1 = 0,
    IC2 = 1,
    IC3 = 2,
    IC4 = 3,
}

/// Input Capture input prescaler.
///
/// Input capture is done every x events
#[derive(Clone, Copy, PartialEq)]
pub enum IcPsc {
    Off   = 0b00,
    Psc2 = 0b01,
    Psc4 = 0b10,
    Psc8 = 0b11,
}

/// Input Capture input source.
///
/// The direction of the channel (input/output) as well as the input used.
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum IcInput {
    OUT = 0,
    IN_TI1 = 1,
    IN_TI2 = 2,
    IN_TRC = 3,
    IN_TI3 = 5,
    IN_TI4 = 6,
}

/// Slave external trigger polarity
#[derive(Clone, Copy, PartialEq)]
pub enum EtPol {
    RISING,
    FALLING,
}

/// Input Capture input polarity
#[derive(Clone, Copy, PartialEq)]
pub enum IcPol {
    RISING,
    FALLING,
}


#[derive(Clone, Copy, PartialEq)]
pub enum EventGenerator {
    /// Break generation
    BG   = 1 << 7,
    /// Trigger generation
    TG   = 1 << 6,
    /// Capture/compare control update generation
    COMG = 1 << 5,
    /// Capture/compare 4 generation
    CC4G = 1 << 4,
    /// Capture/compare 3 generation
    CC3G = 1 << 3,
    /// Capture/compare 2 generation
    CC2G = 1 << 2,
    /// Capture/compare 1 generation
    CC1G = 1 << 1,
    /// Update generation
    UG   = 1 << 0
}

#[derive(Clone, Copy, PartialEq)]
pub enum StatusFlag {
    /// Capture/compare 4 overcapture flag
    CC4OF = (1 << 12),
    /// Capture/compare 3 overcapture flag
    CC3OF = (1 << 11),
    /// Capture/compare 2 overcapture flag
    CC2OF = (1 << 10),
    /// Capture/compare 1 overcapture flag
    CC1OF = (1 << 9),
    /// Break interrupt flag
    BIF = (1 << 7),
    /// Trigger interrupt flag
    TIF = (1 << 6),
    /// COM interrupt flag
    COMIF = (1 << 5),
    /// Capture/compare 4 interrupt flag
    CC4IF = (1 << 4),
    /// Capture/compare 3 interrupt flag
    CC3IF = (1 << 3),
    /// Capture/compare 2 interrupt flag
    CC2IF = (1 << 2),
    /// Capture/compare 1 interrupt flag
    CC1IF = (1 << 1),
    /// Update interrupt flag
    UIF = (1 << 0)
}

bitortype!(Irq, u32);

/// Timer DMA and Interrupt Enable Values
pub mod irq {
    use super::Irq;
    /// Trigger DMA request enable
    pub const TDE : Irq =  Irq(1 << 14);
    /// COM DMA request enable
    pub const COMDE : Irq =  Irq(1 << 13);
    /// Capture/Compare 4 DMA request enable
    pub const CC4DE : Irq =  Irq(1 << 12);
    /// Capture/Compare 3 DMA request enable
    pub const CC3DE : Irq =  Irq(1 << 11);
    /// Capture/Compare 2 DMA request enable
    pub const CC2DE : Irq =  Irq(1 << 10);
    /// Capture/Compare 1 DMA request enable
    pub const CC1DE : Irq =  Irq(1 << 9);
    /// Update DMA request enable
    pub const UDE : Irq =  Irq(1 << 8);
    /// Break interrupt enable
    pub const BIE : Irq =  Irq(1 << 7);
    /// Trigger interrupt enable
    pub const TIE : Irq =  Irq(1 << 6);
    /// COM interrupt enable
    pub const COMIE : Irq =  Irq(1 << 5);
    /// Capture/compare 4 interrupt enable
    pub const CC4IE : Irq =  Irq(1 << 4);
    /// Capture/compare 3 interrupt enable
    pub const CC3IE : Irq =  Irq(1 << 3);
    /// Capture/compare 2 interrupt enable
    pub const CC2IE : Irq =  Irq(1 << 2);
    /// Capture/compare 1 interrupt enable
    pub const CC1IE : Irq =  Irq(1 << 1);
    /// Update interrupt enable
    pub const UIE : Irq =  Irq(1 << 0);
}


bitortype!(IdleState, u32);

/// Force Output Idle State Control Values
pub mod idle_state {
    use super::IdleState;
    /// Output idle state 4 (OC4 output)
    pub const OIS4: IdleState = IdleState(1 << 14);
    /// Output idle state 3 (OC3N output)
    pub const OIS3N: IdleState = IdleState(1 << 13);
    /// Output idle state 3 (OC3 output)
    pub const OIS3: IdleState = IdleState(1 << 12);
    /// Output idle state 2 (OC2N output)
    pub const OIS2N: IdleState = IdleState(1 << 11);
    /// Output idle state 2 (OC2 output)
    pub const OIS2: IdleState = IdleState(1 << 10);
    /// Output idle state 1 (OC1N output)
    pub const OIS1N: IdleState = IdleState(1 << 9);
    /// Output idle state 1 (OC1 output)
    pub const OIS1: IdleState = IdleState(1 << 8);
    pub const OIS_MASK: u32 = (0x7f << 8);
}


#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum MasterMode {
    RESET = 0,
    ENABLE = 1,
    UPDATE = 2,
    COMPARE_PULSE = 3,
    COMPARE_OC1REF = 4,
    COMPARE_OC2REF = 5,
    COMPARE_OC3REF = 6,
    COMPARE_OC4REF = 7
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum Alignment {
    EDGE = 0,
    CENTER_1 = 1,
    CENTER_2 = 2,
    CENTER_3 = 3
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum BreakLock {
    OFF     = 0,
    LEVEL_1 = 1,
    LEVEL_2 = 2,
    LEVEL_3 = 3
}

pub trait TimGeneralExt: GeneralEnums {
    /// Enable Interrupts for a Timer
    ///
    /// * `irq: u32` - Logical OR of all interrupt enable bits to be set
    fn enable_irq(&self, irq: Irq);

    /// Disable Interrupts for a Timer.
    ///
    /// * `irq: u32` - Logical OR of all interrupt enable bits to be cleared
    fn disable_irq(&self, irq: Irq);

    /// Read a Status Flag.
    ///
    /// * `flag: StatusFlag` - Status register flag  @ref tim_sr_values.
    ///
    /// Returns `bool` flag set.
    fn get_flag(&self, flag: StatusFlag) -> bool;

    /// Clear a Status Flag.
    ///
    /// * `flag: StatusFlag` - Status register flag.
    fn clear_flag(&self, flag: StatusFlag);

    /// Set the Timer Mode.
    ///
    /// The modes are:
    /// * Clock divider ratio (to form the sampling clock for the input filters,
    /// and the dead-time clock in the advanced timers 1 and 8)
    /// * Edge/centre alignment
    /// * Count direction
    ///
    /// The alignment and count direction are effective only for timers 1 to 5 and 8
    /// while the clock divider ratio is effective for all timers except 6,7
    /// The remaining timers are limited hardware timers which do not support these mode
    /// settings.
    ///
    /// **Note**: When center alignment mode is selected, count direction is controlled by
    /// hardware and cannot be written. The count direction setting has no effect
    /// in this case.
    ///
    /// * `clock_div: ClockDivision` - Clock Divider Ratio in bits 8,9.
    /// * `alignment: Alignment` - Alignment bits in 5,6
    /// * `direction: Direction` - Count direction in bit 4
    fn set_mode(&self, clock_div: Self::ClockDivision,
            alignment: Alignment, direction: Self::Direction);

    /// Set Input Filter and Dead-time Clock Divider Ratio.
    ///
    /// This forms the sampling clock for the input filters and the dead-time clock
    /// in the advanced timers 1 and 8, by division from the timer clock.
    ///
    /// * `clock_div: ClockDivision` - Clock Divider Ratio in bits 8,9
    fn set_clock_division(&self, clock_div : Self::ClockDivision);

    /// Enable Auto-Reload Buffering.
    ///
    /// During counter operation this causes the counter to be loaded from its
    /// auto-reload register only at the next update event.
    fn enable_preload(&self);

    /// Disable Auto-Reload Buffering.
    ///
    /// This causes the counter to be loaded immediately with a new count value when the
    /// auto-reload register is written, so that the new value becomes effective for the
    /// current count cycle rather than for the cycle following an update event.
    fn disable_preload(&self);

    /// Specify the counter alignment mode.
    ///
    /// The mode can be edge aligned or centered.
    ///
    /// * `alignment: Alignment` - Alignment bits in 5,6
    fn set_alignment(&self, alignment: Alignment);

    /// Set the Timer to Count Up.
    ///
    /// This has no effect if the timer is set to center aligned.
    fn direction_up(&self);

    /// Set the Timer to Count Down.
    ///
    /// This has no effect if the timer is set to center aligned.
    fn direction_down(&self);

    /// Enable the Timer for One Cycle and Stop.
    fn one_shot_mode(&self);

    /// Enable the Timer to Run Continuously.
    fn continuous_mode(&self);

    /// Set the Timer to Generate Update IRQ or DMA on any Event.
    ///
    /// The events which will generate an interrupt or DMA request can be:
    /// * a counter underflow/overflow,
    /// * a forced update,
    /// * an event from the slave mode controller.
    fn update_on_any(&self);

    /// Set the Timer to Generate Update IRQ or DMA only from Under/Overflow Events.
    fn update_on_overflow(&self);

    /// Enable Timer Update Events.
    fn enable_update_event(&self);

    /// Disable Timer Update Events.
    ///
    /// Update events are not generated and the shadow registers keep their values.
    fn disable_update_event(&self);

    /// Enable the timer to start counting.
    ///
    /// This should be called after the timer initial configuration has been completed.
    fn enable_counter(&self);

    /// Stop the timer from counting.
    fn disable_counter(&self);

    /// Set the Master Mode
    ///
    /// This sets the Trigger Output TRGO for synchronizing with slave timers or
    /// passing as an internal trigger to the ADC or DAC.
    ///
    /// * `mode: MasterMode` - Master Mode
    fn set_master_mode(&self, mode: MasterMode);

    /// Set Timer DMA Requests on Capture/Compare Events.
    ///
    /// Capture/compare events will cause DMA requests to be generated.
    fn set_dma_on_compare_event(&self);

    /// Set Timer DMA Requests on Update Events.
    ///
    /// Update events will cause DMA requests to be generated.
    fn set_dma_on_update_event(&self);

    /// Set the Value for the Timer Prescaler.
    ///
    /// The timer clock is prescaled by the 16 bit scale value plus 1.
    ///
    /// * `value: u16` Prescaler values 0...0xFFFF.
    fn set_prescaler(&self, value: u16);

    /// Timer Set Period
    ///
    /// Specify the timer period in the auto-reload register.
    ///
    /// * `period: u16` - Period in counter clock ticks.
    fn set_period(&self, period: u16);

    /// Timer Enable the Output Compare Clear Function
    ///
    /// When this is enabled, the output compare signal is cleared when a high is
    /// detected on the external trigger input. This works in the output compare and
    /// PWM modes only (not forced mode).
    ///
    /// The output compare signal remains off until the next update event.
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action taken)
    fn enable_oc_clear(&self, oc_id: OcId);

    /// Timer Disable the Output Compare Clear Function
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action taken)
    fn disable_oc_clear(&self, oc_id: OcId);

    /// Timer Enable the Output Compare Fast Mode
    ///
    /// When this is enabled, the output compare signal is forced to the compare state
    /// by a trigger input, independently of the compare match. This speeds up the
    /// setting of the output compare to 3 clock cycles as opposed to at least 5 in the
    /// slow mode. This works in the PWM1 and PWM2 modes only.
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action taken)
    fn set_oc_fast_mode(&self, oc_id: OcId);

    /// Timer Enable the Output Compare Slow Mode
    ///
    /// This disables the fast compare mode and the output compare depends on the
    /// counter and compare register values.
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action taken)
    fn set_oc_slow_mode(&self, oc_id: OcId);

    /// Timer Set Output Compare Mode
    ///
    /// Specifies how the comparator output will respond to a compare match. The mode
    /// can be:
    /// * Frozen - the output does not respond to a match.
    /// * Active - the output assumes the active state on the first match.
    /// * Inactive - the output assumes the inactive state on the first match.
    /// * Toggle - The output switches between active and inactive states on each
    /// match.
    /// * Force inactive. The output is forced low regardless of the compare state.
    /// * Force active. The output is forced high regardless of the compare state.
    /// * PWM1 - The output is active when the counter is less than the compare
    /// register contents and inactive otherwise.
    /// * PWM2 - The output is inactive when the counter is less than the compare
    /// register contents and active otherwise.
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action taken)
    /// * `oc_mode : OcMode` - OC mode designators.
    fn set_oc_mode(&self, oc_id: OcId,
               oc_mode : OcMode);

    /// Timer Enable the Output Compare Preload Register
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action taken)
    fn enable_oc_preload(&self, oc_id : OcId);

    /// Timer Disable the Output Compare Preload Register
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action)
    fn disable_oc_preload(&self, oc_id: OcId);

    /// Timer Set Output Compare Value
    ///
    /// This is a convenience function to set the OC preload register value for loading
    /// to the compare register.
    ///
    /// * `oc_id: OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (no action taken)
    /// * `value : u16` - Compare value.
    fn set_oc_value(&self, oc_id: OcId, value: u16);

    /// Force generate a timer event.
    ///
    /// The event specification consists of 8 possible events that can be forced on the
    /// timer. The forced events are automatically cleared by hardware. The UG event is
    /// useful to cause shadow registers to be preloaded before the timer is started to
    /// avoid uncertainties in the first cycle in case an update event may never be
    /// generated.
    ///
    /// * `event: EventGenerator` - Event specification
    fn generate_event(&self, event: EventGenerator);

    /// Read Counter
    ///
    /// Read back the value of a timer's counter register contents
    ///
    /// Returns `u16` Counter value.
    fn timer_get_counter(&self) -> u16;

    /// Set Counter
    ///
    /// Set the value of a timer's counter register contents.
    ///
    /// * `count: u16` - Counter value.
    fn set_counter(&self, count: u16);

    /// Set Input Capture Filter Parameters
    ///
    /// Set the input filter parameters for an input channel, specifying:
    /// * the frequency of sampling from the Deadtime and Sampling clock
    /// (@see @ref timer_set_clock_division)
    /// * the number of events that must occur before a transition is considered
    /// valid.
    ///
    /// * `ic: IcId` - Input Capture channel designator.
    /// * `tim_ic_filter: IcFilter` - Input Capture Filter identifier.
    fn ic_set_filter(&self, ic: IcId, tim_ic_filter: IcFilter);

    /// Set Input Capture Prescaler
    ///
    /// Set the number of events between each capture.
    ///
    /// * `ic: IcId` - Input Capture channel designator.
    /// * `psc: IcPsc` - Input Capture sample clock prescaler.
    fn ic_set_prescaler(&self, ic: IcId, psc: IcPsc);

    /// Set Capture/Compare Channel Direction/Input
    ///
    /// The Capture/Compare channel is defined as output (compare) or input with the
    /// input mapping specified:
    /// * channel is configured as output
    /// * channel is configured as input and mapped on corresponding input
    /// * channel is configured as input and mapped on alternate input
    /// (TI2 for channel 1, TI1 for channel 2, TI4 for channel 3, TI3 for channel 4)
    /// * channel is configured as input and is mapped on TRC (requires an
    /// internal trigger input selected through TS bit
    ///
    /// @note not all combinations of the input and channel are valid, see datasheets.
    /// @note these parameters are writable only when the channel is off.
    ///
    /// * `ic: IcId` - Input Capture channel designator.
    /// * `ic_in : IcInput` - Input Capture channel direction and source input.
    fn ic_set_input(&self, ic: IcId, ic_in: IcInput);

    /// Enable Timer Input Capture
    ///
    /// * `ic : IcId` - Input Capture channel designator.
    fn ic_enable(&self, ic: IcId);

    /// Disable Timer Input Capture
    ///
    /// * `ic : IcId` - Input Capture channel designator.
    fn ic_disable(&self, ic: IcId);

    /// Set External Trigger Filter Parameters for Slave
    ///
    /// Set the input filter parameters for the external trigger, specifying:
    /// * the frequency of sampling from the Deadtime and Sampling clock
    /// * the number of events that must occur before a transition is considered
    /// valid.
    ///
    /// * `flt: IcFilter` - Input Capture Filter identifier.
    fn slave_set_filter(&self, flt: IcFilter);

    /// Set External Trigger Prescaler for Slave
    ///
    /// Set the external trigger frequency division ratio.
    ///
    /// * `psc: IcPsc` - Input Capture sample clock prescaler.
    fn slave_set_prescaler(&self, psc: IcPsc);

    /// Set External Trigger Polarity for Slave
    ///
    /// * `pol: EtPol` - Slave External Trigger polarity.
    fn slave_set_polarity(&self, pol: EtPol);

    /// Set Slave Mode
    ///
    /// * `mode: SlaveMode` - Slave mode
    fn slave_set_mode(&self, mode: Self::SlaveMode);

    /// Set Slave Trigger Source
    ///
    /// * `trigger: TriggerSelection` - Slave trigger source
    fn slave_set_trigger(&self, trigger: Self::TriggerSelection);

}

/// These settings are only valid for the advanced timers.
pub trait TimAdvancedExt {

    /// Timer set Output Compare Idle State High
    ///
    /// @sa Similar function suitable for multiple OC idle state settings
    /// @ref timer_set_output_idle_state
    ///
    /// * `oc_id : OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (only for advanced
    ///     timers 1 and 8)
    fn set_oc_idle_state_set(&self, oc_id : OcId);

    /// Set the Value for the Timer Repetition Counter.
    ///
    /// A timer update event is generated only after the specified number of repeat
    /// count cycles have been completed.
    ///
    /// * `value : u8` - Repetition values 0...0xFF.
    fn set_repetition_counter(&self, value : u8);

    /// Timer Set Output Compare Idle State Low
    ///
    /// @sa Similar function suitable for multiple OC idle state settings
    /// @ref timer_reset_output_idle_state
    ///
    /// * `oc_id : OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (only for advanced
    ///     timers 1 and 8)
    fn set_oc_idle_state_unset(&self, oc_id : OcId);

    /// Set Timer Output Idle States High.
    ///
    /// This determines the value of the timer output compare when it enters idle state.
    ///
    /// @sa @ref timer_set_oc_idle_state_set
    ///
    /// * `outputs : IdleState` - Timer Output Idle State Controls @ref
    /// tim_x_cr2_ois. If several settings are to be made, use the logical OR of the
    /// output control values.
    fn set_output_idle_state(&self, outputs : IdleState);

    /// Set Timer Output Idle States Low.
    ///
    /// This determines the value of the timer output compare when it enters idle state.
    ///
    /// @sa @ref timer_set_oc_idle_state_unset
    ///
    /// * `outputs : IdleState` - Timer Output Idle State Controls @ref
    /// tim_x_cr2_ois
    fn reset_output_idle_state(&self, outputs : IdleState);

    /// Enable Output in Break
    ///
    /// Enables the output in the Break feature of an advanced timer. This does not
    /// enable the break functionality itself but only sets the Master Output Enable in
    /// the Break and Deadtime Register.
    ///
    /// **Note**: It is necessary to call this function to enable the output on an advanced
    /// timer <b>even if break or deadtime features are not being used</b>.
    fn enable_break_main_output(&self);

    /// Disable Output in Break
    ///
    /// Disables the output in the Break feature of an advanced timer. This clears
    /// the Master Output Enable in the Break and Deadtime Register.
    fn disable_break_main_output(&self);

    /// Enable Automatic Output in Break
    ///
    /// Enables the automatic output feature of the Break function of an advanced
    /// timer so that the output is re-enabled at the next update event following a
    /// break event.
    fn enable_break_automatic_output(&self) ;

    /// Disable Automatic Output in Break
    ///
    /// Disables the automatic output feature of the Break function of an advanced
    /// timer so that the output is re-enabled at the next update event following a
    /// break event.
    fn disable_break_automatic_output(&self);

    /// Activate Break when Input High
    ///
    /// Sets the break function to activate when the break input becomes high.
    fn set_break_polarity_high(&self);

    /// Activate Break when Input Low
    ///
    /// Sets the break function to activate when the break input becomes low.
    fn set_break_polarity_low(&self);
    
    /// Enable Break
    ///
    /// Enables the break function of an advanced timer.
    fn enable_break(&self);
    
    /// Disable Break
    ///
    /// Disables the break function of an advanced timer.
    fn disable_break(&self);

    /// Enable Off-State in Idle Mode
    ///
    /// Enables the off-state in idle mode for the break function of an advanced
    /// timer. When the master output is disabled the output is set to its
    /// inactive level as defined by the output polarity.
    fn set_enabled_off_state_in_idle_mode(&self);
    
    /// Disable Off-State in Idle Mode
    ///
    /// Disables the off-state in idle mode for the break function of an advanced
    /// timer. When the master output is disabled the output is also disabled.
    fn set_disabled_off_state_in_idle_mode(&self);

    /// Set Lock Bits
    ///
    /// Set the lock bits for an advanced timer. Three levels of lock providing
    /// protection against software errors. Once written they cannot be changed until a
    /// timer reset has occurred.
    ///
    /// * `lock : BreakLock` - Lock specification
    fn set_break_lock(&self, lock : BreakLock);

    /// Set Deadtime
    ///
    /// The deadtime and sampling clock (DTSC) is set in the clock division ratio part
    /// of the timer mode settings. The deadtime count is an 8 bit value defined in
    /// terms of the number of DTSC cycles:
    ///
    /// @li Bit 7 = 0, deadtime = bits(6:0)
    /// @li Bits 7:6 = 10, deadtime = 2x(64+bits(5:0))
    /// @li Bits 7:5 = 110, deadtime = 8x(32+bits(5:0))
    /// @li Bits 7:5 = 111, deadtime = 16x(32+bits(5:0))
    ///
    /// * `deadtime : u8` - Deadtime count specification as defined above.
    fn set_deadtime(&self, deadtime : u8);
}

/// This setting is only valid for the advanced timer channels with complementary outputs.
pub trait TimComplementaryExt {
    /// Enable Timer Capture/Compare Control Update with Trigger.
    ///
    /// If the capture/compare control bits CCxE, CCxNE and OCxM are set to be
    /// preloaded, they are updated by software generating the COMG event (@ref
    /// timer_generate_event) or when a rising edge occurs on the trigger input TRGI.
    fn enable_compare_control_update_on_trigger(&self);

    /// Disable Timer Capture/Compare Control Update with Trigger.
    ///
    /// If the capture/compare control bits CCxE, CCxNE and OCxM are set to be
    /// preloaded, they are updated by software generating the COMG event (@ref
    /// timer_generate_event).
    fn disable_compare_control_update_on_trigger(&self);

    /// Enable Timer Capture/Compare Control Preload.
    ///
    /// The capture/compare control bits CCxE, CCxNE and OCxM are set to be preloaded
    /// when a COM event occurs.
    fn enable_preload_complementry_enable_bits(&self);

    /// Disable Timer Capture/Compare Control Preload.
    ///
    /// The capture/compare control bits CCxE, CCxNE and OCxM preload is disabled.
    fn disable_preload_complementry_enable_bits(&self);
    
    /// Enable Off-State in Run Mode
    ///
    /// Enables the off-state in run mode for the break function of an advanced
    /// timer in which the complementary outputs have been configured. It has no effect
    /// if no complementary output is present. When the capture-compare output is
    /// disabled while the complementary output is enabled, the output is set to its
    /// inactive level as defined by the output polarity.
    fn set_enabled_off_state_in_run_mode(&self);
    
    /// Disable Off-State in Run Mode
    ///
    /// Disables the off-state in run mode for the break function of an advanced
    /// timer in which the complementary outputs have been configured. It has no effect
    /// if no complementary output is present. When the capture-compare output is
    /// disabled, the output is also disabled.
    fn set_disabled_off_state_in_run_mode(&self);

}

pub trait TimOutputExt {
    /*
    /// Return Interrupt Source.
    ///
    /// Returns true if the specified interrupt flag (UIF, TIF or CCxIF, with BIF or
    /// COMIF for advanced timers) was set and the interrupt was enabled. If the
    /// specified flag is not an interrupt flag, the function returns false.
    ///
    /// @todo Timers 6-7, 9-14 have fewer interrupts, but invalid flags are not caught
    /// here.
    ///
    /// * `flag : Irq` - Status register flag  @ref tim_sr_values.
    ///
    /// Returns `bool` flag set.
    fn interrupt_source(&self, flag : Irq) -> bool;
*/
    /// Timer Set the Output Polarity High
    ///
    /// The polarity of the channel output is set active high.
    ///
    /// * `oc_id : OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3  (only for advanced
    ///     timers 1 and 8)
    fn set_oc_polarity_high(&self, oc_id : OcId);
    
    /// Timer Set the Output Polarity Low
    ///
    /// The polarity of the channel output is set active low.
    ///
    /// * `oc_id : OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (only for advanced
    ///     timers 1 and 8)
    fn set_oc_polarity_low(&self, oc_id : OcId);

    /// Timer Enable the Output Compare
    ///
    /// The channel output compare functionality is enabled.
    ///
    /// * `oc_id : OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (only for advanced
    ///     timers 1 and 8)
    fn enable_oc_output(&self, oc_id : OcId);

    /// Timer Disable the Output Compare
    /// 
    /// The channel output compare functionality is disabled.
    /// 
    /// * `oc_id : OcId` - OC channel designators
    ///     TIM_OCx where x=1..4, TIM_OCxN where x=1..3 (only for advanced
    ///     timers 1 and 8)
    fn disable_oc_output(&self, oc_id : OcId);

}


/// Implements TimGeneralExt
macro_rules! impl_timgeneral {
    ($target:ty) => (
    
    impl TimGeneralExt for $target {

        fn enable_irq(&self, irq : Irq) {
            self.dier     .modify(|r,w| unsafe { w
                .bits( r.bits() | irq.value() )
            });
        }

        fn disable_irq(&self, irq : Irq) {
            self.dier     .modify(|r,w| unsafe { w
                .bits( r.bits() & !irq.value() )
            });
        }

        fn get_flag(&self, flag : StatusFlag) -> bool {
            ( self.sr.read().bits() & (flag as u32) ) != 0
        }

        fn clear_flag(&self, flag : StatusFlag) {
            /* All defined bits are rc_w0 */
            self.sr       .write(|w| unsafe { w
                .bits( !(flag as u32) )
            });
        //TIM_SR(timer_peripheral) = ~flag;
        }

        fn set_mode(&self, clock_div : Self::ClockDivision,
                alignment : Alignment, direction : Self::Direction) {
            self.cr1       .modify(|_,w| w
                .cms()    .bits( alignment as u8 )
                .ckd()    .variant( clock_div )
                .dir()    .variant( direction )
            );
        }

        fn set_clock_division(&self, clock_div : Self::ClockDivision) {
            self.cr1       .modify(|_,w| w
                .ckd()    .variant( clock_div )
            );
        }

        fn enable_preload(&self) {
            self.cr1       .modify(|_,w| w
                .arpe()    .set_bit()
            );
        }

        fn disable_preload(&self) {
            self.cr1       .modify(|_,w| w
                .arpe()    .clear_bit()
            );
        }

        fn set_alignment(&self, alignment : Alignment) {
            self.cr1       .modify(|_,w| w
                .cms()    .bits( alignment as u8 )
            );
        }

        fn direction_up(&self) {
            self.cr1       .modify(|_,w| w
                .dir()     .up()
            );
        }

        fn direction_down(&self) {
            self.cr1       .modify(|_,w| w
                .dir()     .down()
            );
        }

        fn one_shot_mode(&self) {
            self.cr1       .modify(|_,w| w
                .opm()     .enabled()
            );
        }

        fn continuous_mode(&self) {
            self.cr1       .modify(|_,w| w
                .opm()     .disabled()
            );
        }

        fn update_on_any(&self) {
            self.cr1       .modify(|_,w| w
                .urs()     .clear_bit()
            );
        }

        fn update_on_overflow(&self) {
            self.cr1       .modify(|_,w| w
                .urs()     .set_bit()
            );
        }

        fn enable_update_event(&self) {
            self.cr1       .modify(|_,w| w
                .udis()     .clear_bit()
            );
        }

        fn disable_update_event(&self) {
            self.cr1       .modify(|_,w| w
                .udis()    .set_bit()
            );
        }

        fn enable_counter(&self) {
            self.cr1       .modify(|_,w| w
                .cen()     .enabled()
            );
        }

        fn disable_counter(&self) {
            self.cr1       .modify(|_,w| w
                .cen()     .disabled()
            );
        }

        fn set_master_mode(&self, mode : MasterMode) {
            self.cr2        .modify(|_,w| w
                .mms()     .bits(mode as u8)
            );
        }

        fn set_dma_on_compare_event(&self) {
            self.cr2       .modify(|_,w| w
                .ccds()    .clear_bit()
            );
        }

        fn set_dma_on_update_event(&self) {
            self.cr2       .modify(|_,w| w
                .ccds()    .set_bit()
            );
        }

        fn set_prescaler(&self, value : u16) {
            self.psc       .write(|w| w
                .psc() .bits ( value)
            );
        }

        fn set_period(&self, period : u16) {
            self.arr       .write(|w| w
                .arr() .bits ( period )
            );
        }

        fn enable_oc_clear(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc1ce()    .set_bit()
                    );
                },
                OcId::OC2 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc2ce()    .set_bit()
                    );
                },
                OcId::OC3 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc3ce()    .set_bit()
                    );
                },
                OcId::OC4 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc4ce()    .set_bit() // TYPO in Ref.Manual
                    );
                }
            /* Ignoring as oc clear enable only applies to the whole
             * channel.
             */
                _ => unreachable!()
            }
        }

        fn disable_oc_clear(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc1ce()    .clear_bit()
                    );
                },
                OcId::OC2 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc2ce()    .clear_bit()
                    );
                },
                OcId::OC3 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc3ce()    .clear_bit()
                    );
                },
                OcId::OC4 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc4ce()    .clear_bit() // TYPO in Ref.Manual
                    );
                },
            /* Ignoring as oc clear enable only applies to the whole
             * channel.
             */
                _ => unreachable!()
            }
        }

        fn set_oc_fast_mode(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc1fe()    .set_bit()
                    );
                },
                OcId::OC2 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc2fe()    .set_bit()
                    );
                },
                OcId::OC3 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc3fe()    .set_bit()
                    );
                },
                OcId::OC4 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc4fe()    .set_bit()
                    );
                }
            /* Ignoring as fast enable only applies to the whole channel. */
                _ => unreachable!()
            }
        }

        fn set_oc_slow_mode(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc1fe()    .clear_bit()
                    );
                },
                OcId::OC2 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc2fe()    .clear_bit()
                    );
                },
                OcId::OC3 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc3fe()    .clear_bit()
                    );
                },
                OcId::OC4 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc4fe()    .clear_bit()
                    );
                },
            /* Ignoring as this option applies to the whole channel. */
                _ => unreachable!()
            }
        }

        fn set_oc_mode(&self, oc_id : OcId,
                   oc_mode : OcMode) {
            match oc_id {
                OcId::OC1 => { unsafe {
                    self.ccmr1_output()    .modify(|_,w| w
                        .cc1s()    .bits(0b00)
                        //.oc1m()    .variant(oc_mode)
                        .oc1m()    .bits(oc_mode as u8)
                    );
                } },
                OcId::OC2 => { unsafe {
                    self.ccmr1_output()    .modify(|_,w| w
                        .cc2s()    .bits(0b00)
                        //.oc2m()    .variant(oc_mode)
                        .oc2m()    .bits(oc_mode as u8)
                    );
                } },
                OcId::OC3 => { unsafe {
                    self.ccmr2_output()    .modify(|_,w| w
                        .cc3s()    .bits(0b00)
                        //.oc3m()    .variant(oc_mode)
                        .oc3m()    .bits(oc_mode as u8)
                    );
                } },
                OcId::OC4 => { unsafe {
                    self.ccmr2_output()    .modify(|_,w| w
                        .cc4s()    .bits(0b00)
                        //.oc4m()    .variant(oc_mode)
                        .oc4m()    .bits(oc_mode as u8)
                    );
                } },
                /* Ignoring as this option applies to the whole channel. */
                _ => unreachable!()
            }
        }

        fn enable_oc_preload(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc1pe()    .set_bit()
                    );
                },
                OcId::OC2 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc2pe()    .set_bit()
                    );
                },
                OcId::OC3 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc3pe()    .set_bit()
                    );
                },
                OcId::OC4 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc4pe()    .set_bit()
                    );
                },
            /* Ignoring as this option applies to the whole channel. */
                _ => unreachable!()
            }
        }

        fn disable_oc_preload(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc1pe()    .clear_bit()
                    );
                },
                OcId::OC2 => {
                    self.ccmr1_output()    .modify(|_,w| w
                        .oc2pe()    .clear_bit()
                    );
                },
                OcId::OC3 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc3pe()     .clear_bit()
                    );
                },
                OcId::OC4 => {
                    self.ccmr2_output()    .modify(|_,w| w
                        .oc4pe()    .clear_bit()
                    );
                },
            /* Ignoring as this option applies to the whole channel. */
                _ => unreachable!()
            }
        }

        fn set_oc_value(&self, oc_id : OcId, value : u16) {
            match oc_id {
                OcId::OC1 => {
                    self.ccr1    .write(|w| w
                        .ccr()  .bits( value )
                    );
                },
                OcId::OC2 => {
                    self.ccr2    .write(|w| w
                        .ccr()  .bits( value )
                    );
                },
                OcId::OC3 => {
                    self.ccr3    .write(|w| w
                        .ccr()  .bits( value )
                    );
                },
                OcId::OC4 => {
                    self.ccr4    .write(|w| w
                        .ccr()  .bits( value )
                    );
                },
                /* Ignoring as this option applies to the whole channel. */
                _ => {}
            }
        }

        /*fn generate_event(&self, event : EventGenerator) {
            self.egr            .modify(|r,w| unsafe { w
                .bits( r.bits() | ( event as u32 ))
            });
        //TIM_EGR(timer_peripheral) |= event;
        }*/
        
        fn generate_event(&self, event : EventGenerator) {
            self.egr        .write(|w| unsafe { w
                .bits( event as u32 )
            });
        }

        fn timer_get_counter(&self) -> u16 {
            self.cnt  .read()  .bits() as u16
        }

        fn set_counter(&self, count : u16) {
            self.cnt       .write(|w| unsafe { w
                .bits(count as u32)
            });
        }

        fn ic_set_filter(&self, ic : IcId, tim_ic_filter : IcFilter) {
            match ic {
                IcId::IC1 => {
                    self.ccmr1_input()    .modify(|_,w| w
                        .ic1f()         .bits( tim_ic_filter as u8 )
                    );
                },
                IcId::IC2 => {
                    self.ccmr1_input()    .modify(|_,w| w
                        .ic2f()         .bits( tim_ic_filter as u8 )
                    );
                },
                IcId::IC3 => {
                    self.ccmr2_input()    .modify(|_,w| w
                        .ic3f()         .bits( tim_ic_filter as u8 )
                    );
                },
                IcId::IC4 => {
                    self.ccmr2_input()    .modify(|_,w| w
                        .ic4f()         .bits( tim_ic_filter as u8 )
                    );
                }
            }
        }

        fn ic_set_prescaler(&self, ic : IcId, psc : IcPsc) {
            match ic {
                IcId::IC1 => { unsafe {
                    self.ccmr1_input()    .modify(|_,w| w
                        .ic1psc()         .bits( psc as u8 )
                    );
                } },
                IcId::IC2 => { unsafe {
                    self.ccmr1_input()    .modify(|_,w| w
                        .ic2psc()         .bits( psc as u8 )
                    );
                } },
                IcId::IC3 => {
                    self.ccmr2_input()    .modify(|_,w| w
                        .ic3psc()         .bits( psc as u8 )
                    );
                },
                IcId::IC4 => {
                    self.ccmr2_input()    .modify(|_,w| w
                        .ic4psc()         .bits( psc as u8 )
                    );
                },
            }
        }

        fn ic_set_input(&self, ic : IcId, ic_in : IcInput) {
            let mut ic_in = (ic_in as u8) & 3;

            match ic {
                /* Input select bits are flipped for these combinations */
                IcId::IC2 | IcId::IC4 => {
                    if (ic_in == IcInput::IN_TI1 as u8 ) ||
                        (ic_in == IcInput::IN_TI2 as u8) {
                        ic_in ^= 3;
                    }
                },
                _ => {}
            }

            match ic {
                IcId::IC1 => { unsafe {
                    self.ccmr1_input()    .modify(|_,w| w
                        .cc1s()         .bits( ic_in )
                    );
                } },
                IcId::IC2 => { unsafe {
                    self.ccmr1_input()    .modify(|_,w| w
                        .cc2s()         .bits( ic_in )
                    );
                } },
                IcId::IC3 => { unsafe {
                    self.ccmr2_input()    .modify(|_,w| w
                        .cc3s()         .bits( ic_in )
                    );
                } },
                IcId::IC4 => { unsafe {
                    self.ccmr2_input()    .modify(|_,w| w
                        .cc4s()         .bits( ic_in )
                    );
                } }
            }
        }

        fn ic_enable(&self, ic: IcId) {
            self.ccer       .modify(|r,w| unsafe { w
                .bits( r.bits() | (1 << ((ic as u32) * 4)) )
            });
        }

        fn ic_disable(&self, ic: IcId) {
            self.ccer       .modify(|r,w| unsafe { w
                .bits( r.bits() & !(1 << ((ic as u32) * 4)) )
            });
        }

        fn slave_set_filter(&self, flt: IcFilter) {
            self.smcr       .modify(|_,w| w
                .etps()     .bits(flt as u8)
            );
        }

        fn slave_set_prescaler(&self, psc: IcPsc) {
            self.smcr       .modify(|_,w| w
                .etps()     .bits(psc as u8)
            );
        }

        fn slave_set_polarity(&self, pol: EtPol) {
            let bit = match pol {
                EtPol::RISING => false,
                EtPol::FALLING => true
            };
            self.smcr       .modify(|_,w| w
                .etp()      .bit(bit)
            );
        }

        fn slave_set_mode(&self, mode: Self::SlaveMode) {
            self.smcr       .modify(|_,w| w
                .sms()      .variant( mode )
            );
        }

        fn slave_set_trigger(&self, trigger: Self::TriggerSelection) {
            self.smcr       .modify(|_,w| w
                .ts()       .variant(trigger)
            );
        }
    }
    )
}


/// Implements TimOutputExt for Not Advanced Timers
macro_rules! impl_timoutputnotadvanced {
    ($target:ty) => (
    impl TimOutputExt for $target {
/*
        fn interrupt_source(&self, flag : Irq) -> bool {
            // flag not set or interrupt disabled or not an interrupt source
            !(((self.sr.read().bits() &
                self.dier.read().bits() & flag.value()) == 0) ||
                (flag.value() > (StatusFlag::BIF as u32)))
        }
*/
        fn set_oc_polarity_high(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1p()    .clear_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2p()    .clear_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3p()    .clear_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4p()    .clear_bit()
                    );
                }
            /* Ignoring as this option applies to TIM1 and TIM8 only. */
                _ => unreachable!()
            }
        }
        
        fn set_oc_polarity_low(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1p()    .set_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2p()    .set_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3p()    .set_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4p()    .set_bit()
                    );
                }
            /* Ignoring as this option applies to TIM1 and TIM8 only. */
                _ => unreachable!()
            }
        }

        fn enable_oc_output(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1e()    .set_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2e()    .set_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3e()    .set_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4e()    .set_bit()
                    );
                }
            /* Ignoring as this option applies to TIM1 and TIM8 only. */
                _ => unreachable!()
            }
        }

        fn disable_oc_output(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1e()    .clear_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2e()    .clear_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3e()    .clear_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4e()    .clear_bit()
                    );
                }
            /* Ignoring as this option applies to TIM1 and TIM8 only. */
                _ => unreachable!()
            }
        }
    }
    )
}

/// Implements TimOutputExt for Advanced Timers
macro_rules! impl_timoutputadvanced {
    ($target:ty) => (
    impl TimOutputExt for $target {
        /*
        fn interrupt_source(&self, flag : u32) -> bool {
    // Only an interrupt source for advanced timers
    #if ADVANCED_TIMERS
        if ((flag == TIM_SR_BIF) || (flag == TIM_SR_COMIF)) {
            return TIMER_IS_ADVANCED(timer_peripheral);
        }
    #endif
    }
    */
        fn set_oc_polarity_low(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1p()    .set_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2p()    .set_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3p()     .set_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4p()    .set_bit()
                    );
                }
                OcId::OC1N => {
                    self.ccer    .modify(|_,w| w
                        .cc1np()   .set_bit()
                    );
                }
                OcId::OC2N => {
                    self.ccer    .modify(|_,w| w
                        .cc2np()   .set_bit()
                    );
                }
                OcId::OC3N => {
                    self.ccer    .modify(|_,w| w
                        .cc3np()    .set_bit()
                    );
                }
            }
        }
        
        fn enable_oc_output(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1e()    .set_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2e()    .set_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3e()     .set_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4e()    .set_bit()
                    );
                }
                OcId::OC1N => {
                    self.ccer    .modify(|_,w| w
                        .cc1ne()    .set_bit()
                    );
                }
                OcId::OC2N => {
                    self.ccer    .modify(|_,w| w
                        .cc2ne()    .set_bit()
                    );
                }
                OcId::OC3N => {
                    self.ccer    .modify(|_,w| w
                        .cc3ne()    .set_bit()
                    );
                }
            }
        }

        fn disable_oc_output(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1e()    .clear_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2e()    .clear_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3e()     .clear_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4e()    .clear_bit()
                    );
                }
                OcId::OC1N => {
                    self.ccer    .modify(|_,w| w
                        .cc1ne()    .clear_bit()
                    );
                }
                OcId::OC2N => {
                    self.ccer    .modify(|_,w| w
                        .cc2ne()    .clear_bit()
                    );
                }
                OcId::OC3N => {
                    self.ccer    .modify(|_,w| w
                        .cc3ne()    .clear_bit()
                    );
                }
            }
        }
        
        fn set_oc_polarity_high(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.ccer    .modify(|_,w| w
                        .cc1p()    .clear_bit()
                    );
                }
                OcId::OC2 => {
                    self.ccer    .modify(|_,w| w
                        .cc2p()    .clear_bit()
                    );
                }
                OcId::OC3 => {
                    self.ccer    .modify(|_,w| w
                        .cc3p()     .clear_bit()
                    );
                }
                OcId::OC4 => {
                    self.ccer    .modify(|_,w| w
                        .cc4p()    .clear_bit()
                    );
                }
                OcId::OC1N => {
                    self.ccer    .modify(|_,w| w
                        .cc1np()    .clear_bit()
                    );
                }
                OcId::OC2N => {
                    self.ccer    .modify(|_,w| w
                        .cc2np()    .clear_bit()
                    );
                }
                OcId::OC3N => {
                    self.ccer    .modify(|_,w| w
                        .cc3np()    .clear_bit()
                    );
                }
            }
        }
    }
    )
}


/// Implements TimAdvancedExt
macro_rules! impl_timadvanced {
    ($target:ty) => (
    impl TimAdvancedExt for $target { // for Advanced
        fn set_oc_idle_state_set(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.cr2    .modify(|_,w| w
                        .ois1()    .set_bit()
                    );
                }
                OcId::OC2 => {
                    self.cr2    .modify(|_,w| w
                        .ois2()    .set_bit()
                    );
                }
                OcId::OC3 => {
                    self.cr2    .modify(|_,w| w
                        .ois3()     .set_bit()
                    );
                }
                OcId::OC4 => {
                    self.cr2    .modify(|_,w| w
                        .ois4()    .set_bit()
                    );
                }
                OcId::OC1N => {
                    self.cr2    .modify(|_,w| w
                        .ois1n()    .set_bit()
                    );
                }
                OcId::OC2N => {
                    self.cr2    .modify(|_,w| w
                        .ois2n()    .set_bit()
                    );
                }
                OcId::OC3N => {
                    self.cr2    .modify(|_,w| w
                        .ois3n()    .set_bit()
                    );
                }
            }
        }
        
        fn set_repetition_counter(&self, value : u8) {
            self.rcr      .write(|w| unsafe { w
                .bits ( value as u32 )
            });
        }
        
        fn set_oc_idle_state_unset(&self, oc_id : OcId) {
            match oc_id {
                OcId::OC1 => {
                    self.cr2    .modify(|_,w| w
                        .ois1()    .clear_bit()
                    );
                }
                OcId::OC2 => {
                    self.cr2    .modify(|_,w| w
                        .ois2()    .clear_bit()
                    );
                }
                OcId::OC3 => {
                    self.cr2    .modify(|_,w| w
                        .ois3()     .clear_bit()
                    );
                }
                OcId::OC4 => {
                    self.cr2    .modify(|_,w| w
                        .ois4()    .clear_bit()
                    );
                }
                OcId::OC1N => {
                    self.cr2    .modify(|_,w| w
                        .ois1n()    .clear_bit()
                    );
                }
                OcId::OC2N => {
                    self.cr2    .modify(|_,w| w
                        .ois2n()    .clear_bit()
                    );
                }
                OcId::OC3N => {
                    self.cr2    .modify(|_,w| w
                        .ois3n()    .clear_bit()
                    );
                }
            }
        }
        
        fn set_output_idle_state(&self, outputs : IdleState) {
            self.cr2   .modify(|r,w| unsafe { w
                .bits( r.bits() | (outputs.value() & idle_state::OIS_MASK) )
            });
        }

        fn reset_output_idle_state(&self, outputs : IdleState) {
            self.cr2   .modify(|r,w| unsafe { w
                .bits( r.bits() & !(outputs.value() & idle_state::OIS_MASK) )
            });
        }

        fn enable_break_main_output(&self) {
            self.bdtr       .modify(|_,w| w
                .moe()      .set_bit()
            );
        }

        fn disable_break_main_output(&self) {
            self.bdtr       .modify(|_,w| w
                .moe()      .clear_bit()
            );
        }

        fn enable_break_automatic_output(&self) {
            self.bdtr       .modify(|_,w| w
                .aoe()      .set_bit()
            );
        }

        fn disable_break_automatic_output(&self) {
            self.bdtr       .modify(|_,w| w
                .aoe()      .clear_bit()
            );
        }

        fn set_break_polarity_high(&self) {
            self.bdtr       .modify(|_,w| w
                .bkp()      .set_bit()
            );
        }

        fn set_break_polarity_low(&self) {
            self.bdtr       .modify(|_,w| w
                .bkp()      .clear_bit()
            );
        }
        
        fn enable_break(&self) {
            self.bdtr       .modify(|_,w| w
                .bke()      .set_bit()
            );
        }
        
        fn disable_break(&self) {
            self.bdtr       .modify(|_,w| w
                .bke()      .clear_bit()
            );
        }

        fn set_enabled_off_state_in_idle_mode(&self) {
            self.bdtr       .modify(|_,w| w
                .ossi()      .set_bit()
            );
        }
        
        fn set_disabled_off_state_in_idle_mode(&self) {
            self.bdtr       .modify(|_,w| w
                .ossi()      .clear_bit()
            );
        }

        fn set_break_lock(&self, lock : BreakLock) {
            self.bdtr       .modify(|_,w| unsafe { w
                .lock()     .bits( lock as u8 )
            });
        }

        fn set_deadtime(&self, deadtime : u8) {
            self.bdtr       .modify(|_,w| unsafe { w
                .dtg()     .bits( deadtime )
            });
        }
    }
    )
}


/// Implements TimComplementaryExt for Advanced Timers
macro_rules! impl_timcomplementary {
    ($target:ty) => (
    
    impl TimComplementaryExt for $target {
        fn enable_compare_control_update_on_trigger(&self) {
            self.cr2       .modify(|_,w| w
                .ccus()    .set_bit()
            );
        }
        
        fn disable_compare_control_update_on_trigger(&self) {
            self.cr2       .modify(|_,w| w
                .ccus()    .clear_bit()
            );
        }
        
        fn enable_preload_complementry_enable_bits(&self) {
            self.cr2       .modify(|_,w| w
                .ccpc()    .set_bit()
            );
        }
        
        fn disable_preload_complementry_enable_bits(&self) {
            self.cr2       .modify(|_,w| w
                .ccpc()    .clear_bit()
            );
        }

        fn set_enabled_off_state_in_run_mode(&self) {
            self.bdtr       .modify(|_,w| w
                .ossr()      .set_bit()
            );
        }
        
        fn set_disabled_off_state_in_run_mode(&self) {
            self.bdtr       .modify(|_,w| w
                .ossr()     .clear_bit()
            );
        }
    }
    )
}



pub trait Timerf1 {
    
    /// Set Input Polarity
    ///
    /// * `ic : IcId` - Input Capture channel designator.
    /// * `pol : IcPol` - Input Capture polarity.
    fn ic_set_polarity(&self, ic : IcId, pol : IcPol);
}

macro_rules! impl_timf1 {
    ($target:ty) => (
    impl Timerf1 for $target {
        fn ic_set_polarity(&self, ic : IcId, pol : IcPol) {
            match pol {
                IcPol::FALLING => {
                    self.ccer       .modify(|r,w| unsafe { w
                        .bits( r.bits() | (2 << ((ic as u32) * 4)) )
                    });
                },
                IcPol::RISING => {
                    self.ccer       .modify(|r,w| unsafe { w
                        .bits( r.bits() & !(2 << ((ic as u32) * 4)) )
                    });
                }
            }
        }
    }
    )
}

pub trait Ocm3Tim1 {

    /// Set Timer 1 Input to XOR of Three Channels.
    ///
    /// The first timer capture input is formed from the XOR of the first three timer
    /// input channels 1, 2, 3.
    fn set_ti1_ch123_xor(&self);

    /// Set Timer 1 Input to Channel 1.
    ///
    /// The first timer capture input is taken from the timer input channel 1 only.
    fn set_ti1_ch1(&self);

}

impl Ocm3Tim1 for TIM1 {
    fn set_ti1_ch123_xor(&self) {
        self.cr2       .modify(|_,w| w
            .ti1s()      .set_bit()
        );
    }
    fn set_ti1_ch1(&self) {
        self.cr2       .modify(|_,w| w
            .ti1s()     .clear_bit()
        );
    }
}




/* TODO Timer DMA burst */
impl_timgeneral!(TIM1);
impl_timoutputadvanced!(TIM1);
impl_timadvanced!(TIM1);
impl_timcomplementary!(TIM1);
impl_timf1!(TIM1);

impl_timgeneral!(TIM2);
impl_timoutputnotadvanced!(TIM2);
impl_timf1!(TIM2);
