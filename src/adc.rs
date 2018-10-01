//! Analog to Digital Conversion System in
//! the STM32 series of ARM Cortex Microcontrollers by ST Microelectronics.
//!
//! The style of ADC Peripheral supported by this code is found in the F1, F2,
//! F37x, F38x, F4, and L1 series devices (at the time of writing) but is quite
//! different to the style found on the F0 and F30x and F31x.
//! Devices can have up to three A/D converters each with their own set of
//! registers.
//! However all the A/D converters share a common clock.  On most devices, this is
//! prescaled from the APB2 clock by default by a minimum factor of 2 to a maximum
//! of 8, though on the L1 this is always a divider from the HSI. (And therefore HSI
//! _must_ be enabled before attempting to enable the ADC)
//!
//! Each A/D converter has up to ADC_MAX_CHANNELS channels:
//! * On ADC1 the analog channels 16 and 17 are internally connected to the
//! temperature sensor and V<sub>REFINT</sub>, respectively.
//! * On ADC2 (if available) the analog channels 16 and 17 are internally
//! connected to V<sub>SS</sub>.
//! * On ADC3 (if available) the analog channels 9, 14, 15, 16 and 17 are
//! internally connected to V<sub>SS</sub>.
//!
//! The conversions can occur as a one-off conversion whereby the process stops once
//! conversion is complete. The conversions can also be continuous wherein a new
//! conversion starts immediately the previous conversion has ended.
//!
//! Conversion can occur as a single channel conversion or a scan of a group of
//! channels in either continuous or one-off mode. If more than one channel is
//! converted in a scan group, DMA must be used to transfer the data as there is
//! only one result register available. An interrupt can be set to occur at the end
//! of conversion, which occurs after all channels have been scanned.
//!
//! A discontinuous mode allows a subgroup of group of a channels to be converted in
//! bursts of a given length.
//!
//! Injected conversions allow a second group of channels to be converted separately
//! from the regular group. An interrupt can be set to occur at the end of
//! conversion, which occurs after all channels have been scanned.
//!
//! # Basic ADC Handling API
//!
//! Example 1: Simple single channel conversion polled. Enable the peripheral clock
//! and ADC, reset ADC and set the prescaler divider. Set dual mode to independent
//! (default). Enable triggering for a software trigger.
//!
//! ```
//! rcc.periph_clock_enable(rcc::en::ADC1);
//! adc.power_off();
//! rcc.periph_reset_pulse(rcc::rst::ADC1);
//! rcc.set_adcpre(rcc:AdcPre::DIV2);
//! adc.set_dual_mode(adc::DualMode::Independent);
//! adc.disable_scan_mode();
//! adc.set_single_conversion_mode();
//! adc.set_sample_time(ADC_CHANNEL0, adc::SampleTime::Cycles1_5);
//! adc.enable_trigger(ExternalTrigger::SWSTART);
//! adc.power_on();
//! adc.reset_calibration();
//! adc.calibration();
//! adc.start_conversion_regular();
//! while !adc.eoc() {};
//! reg16 = adc.read_regular();
//! ```

#[derive(Clone, Copy, PartialEq)]
pub enum Channel {
    C0 = 0,
    C1 = 1,
    C2 = 2,
    C3 = 3,
    C4 = 4,
    C5 = 5,
    C6 = 6,
    C7 = 7,
    C8 = 8,
    C9 = 9,
    C10 = 10,
    C11 = 11,
    C12 = 12,
    C13 = 13,
    C14 = 14,
    C15 = 15,
    C16 = 16,
    C17 = 17,
    C18 = 18,
}

#[derive(Clone, Copy, PartialEq)]
pub enum InjectedChannel {
  C1,
  C2,
  C3,
  C4,
}

pub trait AdcExt {
    /// ADC Off
    ///
    /// Turn off the ADC to reduce power consumption to a few microamps.
    fn power_off (&mut self);
    
    /// ADC Enable Analog Watchdog for Regular Conversions
    ///
    /// The analog watchdog allows the monitoring of an analog signal between two
    /// threshold levels. The thresholds must be preset. Comparison is done before data
    /// alignment takes place, so the thresholds are left-aligned.
    fn enable_analog_watchdog_regular (&mut self);
    
    /// ADC Disable Analog Watchdog for Regular Conversions
    fn disable_analog_watchdog_regular (&mut self);
    
    /// ADC Enable Analog Watchdog for Injected Conversions
    ///
    /// The analog watchdog allows the monitoring of an analog signal between two
    /// threshold levels. The thresholds must be preset. Comparison is done before data
    /// alignment takes place, so the thresholds are left-aligned.
    fn enable_analog_watchdog_injected (&mut self);
    
    /// ADC Disable Analog Watchdog for Injected Conversions
    fn disable_analog_watchdog_injected (&mut self);
    
    /// ADC Enable Discontinuous Mode for Regular Conversions
    ///
    /// In this mode the ADC converts, on each trigger, a subgroup of up to 8 of the
    /// defined regular channel group. The subgroup is defined by the number of
    /// consecutive channels to be converted. After a subgroup has been converted
    /// the next trigger will start conversion of the immediately following subgroup
    /// of the same length or until the whole group has all been converted. When the
    /// the whole group has been converted, the next trigger will restart conversion
    /// of the subgroup at the beginning of the whole group.
    /// 
    /// * `length: u8` - Number of channels in the group
    fn enable_discontinuous_mode_regular (&mut self, length: u8);
    
    /// ADC Disable Discontinuous Mode for Regular Conversions
    fn disable_discontinuous_mode_regular (&mut self);
    
    /// ADC Enable Discontinuous Mode for Injected Conversions
    ///
    /// In this mode the ADC converts sequentially one channel of the defined group of
    /// injected channels, cycling back to the first channel in the group once the
    /// entire group has been converted.
    fn enable_discontinuous_mode_injected (&mut self);
    
    /// ADC Disable Discontinuous Mode for Injected Conversions
    fn disable_discontinuous_mode_injected (&mut self);
    
    /// ADC Enable Automatic Injected Conversions
    ///
    /// The ADC converts a defined injected group of channels immediately after the
    /// regular channels have been converted. The external trigger on the injected
    /// channels is disabled as required.
    fn enable_automatic_injected_group_conversion (&mut self);
    
    /// ADC Disable Automatic Injected Conversions
    fn disable_automatic_injected_group_conversion (&mut self);
    
    /// ADC Enable Analog Watchdog for All Regular and/or Injected Channels
    ///
    /// The analog watchdog allows the monitoring of an analog signal between two
    /// threshold levels. The thresholds must be preset. Comparison is done before data
    /// alignment takes place, so the thresholds are left-aligned.
    ///
    /// The analog watchdog must be enabled for either or both of the regular or
    /// injected channels. If neither are enabled, the analog watchdog feature will be
    /// disabled.
    fn enable_analog_watchdog_on_all_channels (&mut self);
    
    /// ADC Enable Analog Watchdog for a Selected Channel
    ///
    /// The analog watchdog allows the monitoring of an analog signal between two
    /// threshold levels. The thresholds must be preset. Comparison is done before data
    /// alignment takes place, so the thresholds are left-aligned.
    ///
    /// The analog watchdog must be enabled for either or both of the regular or
    /// injected channels. If neither are enabled, the analog watchdog feature will be
    /// disabled. If both are enabled, the same channel number is monitored.
    ///
    /// * `channel: Channel` - ADC channel number
    fn enable_analog_watchdog_on_selected_channel (&mut self, channel: Channel);
    
    /// ADC Set Scan Mode
    ///
    /// In this mode a conversion consists of a scan of the predefined set of channels,
    /// regular and injected, each channel conversion immediately following the
    /// previous one. It can use single, continuous or discontinuous mode.
    fn enable_scan_mode (&mut self);
    
    /// ADC Disable Scan Mode
    fn disable_scan_mode (&mut self);
    
    /// ADC Enable Injected End-Of-Conversion Interrupt
    fn enable_eoc_interrupt_injected (&mut self);
    
    /// ADC Disable Injected End-Of-Conversion Interrupt
    fn disable_eoc_interrupt_injected (&mut self);
    
    /// ADC Enable Analog Watchdog Interrupt
    fn enable_awd_interrupt (&mut self);
    
    /// ADC Disable Analog Watchdog Interrupt
    fn disable_awd_interrupt (&mut self);
    
    /// ADC Enable Regular End-Of-Conversion Interrupt
    fn enable_eoc_interrupt (&mut self);
    
    /// ADC Disable Regular End-Of-Conversion Interrupt
    fn disable_eoc_interrupt (&mut self);
    
    /// ADC Set the Data as Left Aligned
    fn set_left_aligned (&mut self);
    
    /// ADC Set the Data as Right Aligned
    fn set_right_aligned (&mut self);
    
    /// ADC Read the End-of-Conversion Flag
    ///
    /// This flag is set after all channels of a regular or injected group have been
    /// converted.
    ///
    /// Returns `bool`. End of conversion flag.
    fn eoc (&self) -> bool;
    
    /// ADC Read the End-of-Conversion Flag for Injected Conversion
    ///
    /// This flag is set after all channels of an injected group have been converted.
    ///
    /// Returns `bool`. End of conversion flag.
    fn eoc_injected (&self) -> bool;
    
    /// ADC Read from the Regular Conversion Result Register
    ///
    /// The result read back is 12 bits, right or left aligned within the first 16 bits.
    /// For ADC1 only, the higher 16 bits will hold the result from ADC2 if
    /// an appropriate dual mode has been set.
    /// 
    /// Returns `u32` conversion result.
    fn read_regular (&self) -> u32;
    
    /// ADC Read from an Injected Conversion Result Register
    ///
    /// The result read back from the selected injected result register (one of four)
    /// is 12 bits, right or left aligned within the first 16 bits. The result can have
    /// a negative value if the injected channel offset has been set.
    ///
    /// * `reg: InjectedChannel` - Register number (1 ... 4).
    /// Returns `u32` conversion result.
    fn read_injected (&self, reg: InjectedChannel) -> u32;
    
    /// ADC Enable Continuous Conversion Mode
    ///
    /// In this mode the ADC starts a new conversion of a single channel or a channel
    /// group immediately following completion of the previous channel group conversion.
    fn set_continuous_conversion_mode (&mut self);
    
    /// ADC Enable Single Conversion Mode
    ///
    /// In this mode the ADC performs a conversion of one channel or a channel group
    /// and stops.
    fn set_single_conversion_mode (&mut self);
    
    /// ADC Set Analog Watchdog Upper Threshold
    ///
    /// * `threshold: u16` - Upper threshold value
    fn set_watchdog_high_threshold (&mut self, threshold: u16);
    
    /// ADC Set Analog Watchdog Lower Threshold
    ///
    /// * `threshold: u16` - Lower threshold value
    fn set_watchdog_low_threshold (&mut self, threshold: u16);
    
    /// ADC Set a Regular Channel Conversion Sequence
    ///
    /// Define a sequence of channels to be converted as a regular group with a length
    /// from 1 to ADC_REGULAR_SEQUENCE_MAX channels. If this is called during
    /// conversion, the current conversion is reset and conversion begins again with
    /// the newly defined group.
    /// 
    /// * `channel: &[Channel]` - Set of channels in sequence, integers 0..31.
    fn set_regular_sequence (&mut self, channels: &[Channel]);
    
    /// ADC Set an Injected Channel Conversion Sequence
    ///
    /// Defines a sequence of channels to be converted as an injected group with a
    /// length from 1 to 4 channels. If this is called during conversion, the current
    /// conversion is reset and conversion begins again with the newly defined group.
    ///
    /// * `channels: &[Channel]` - Set of channels in sequence, integers 0..18
    fn set_injected_sequence (&mut self, channels: &[Channel]);
    
    /// ADC Set the Injected Channel Data Offset
    ///
    /// This value is subtracted from the injected channel results after conversion is
    /// complete, and can result in negative results. A separate value can be specified
    /// for each injected data register.
    ///
    /// * `reg: InjectedChannel` - Register number (1 ... 4).
    /// * `offset: u32`.
    fn set_injected_offset (&mut self, reg: InjectedChannel, offset: u32);
    
    /// ADC Software Triggered Conversion on Regular Channels
    ///
    /// This starts conversion on a set of defined regular channels if the ADC trigger
    /// is set to be a software trigger. It is cleared by hardware once conversion
    /// starts.
    ///
    /// Special F1 Note this is a software trigger and requires triggering to be
    /// enabled and the trigger source to be set appropriately otherwise conversion
    /// will not start. This is not the same as the ADC start conversion operation.
    fn start_conversion_regular (&mut self);
    
    /// ADC Software Triggered Conversion on Injected Channels
    ///
    /// This starts conversion on a set of defined injected channels if the ADC trigger
    /// is set to be a software trigger. It is cleared by hardware once conversion
    /// starts.
    ///
    /// Special F1 Note this is a software trigger and requires triggering to be
    /// enabled and the trigger source to be set appropriately otherwise conversion
    /// will not start. This is not the same as the ADC start conversion operation.
    fn start_conversion_injected (&mut self);
    
    /// ADC Enable DMA Transfers
    fn enable_dma (&mut self);
    
    /// ADC Disable DMA Transfers
    fn disable_dma (&mut self);

}

pub trait AdcDualExt {
    /// ADC Set Dual A/D Mode
    ///
    /// The dual mode uses ADC1 as master and ADC2 in a slave arrangement. This setting
    /// is applied to ADC1 only. Start of conversion when triggered can cause
    /// simultaneous conversion with ADC2, or alternate conversion. Regular and
    /// injected conversions can be configured, each one being separately simultaneous
    /// or alternate.
    ///
    /// Fast interleaved mode starts ADC1 immediately on trigger, and ADC2 seven clock
    /// cycles later.
    ///
    /// Slow interleaved mode starts ADC1 immediately on trigger, and ADC2 fourteen
    /// clock cycles later, followed by ADC1 fourteen cycles later again. This can only
    /// be used on a single channel.
    ///
    /// Alternate trigger mode must occur on an injected channel group, and alternates
    /// between the ADCs on each trigger.
    ///
    /// Note that sampling must not overlap between ADCs on the same channel.
    ///
    /// * `mode: DualMode` - Dual mode selection from
    fn set_dual_mode (&mut self, mode: DualMode);
}

pub trait AdcTemperatureExt {
    /// ADC Enable The Temperature Sensor
    ///
    /// This enables both the sensor and the reference voltage measurements on channels
    /// 16 and 17.
    fn enable_temperature_sensor (&mut self);

    /// ADC Disable The Temperature Sensor
    ///
    /// Disabling this will reduce power consumption from the sensor and the reference
    /// voltage measurements.
    fn disable_temperature_sensor (&mut self);
}

pub trait AdcSpecialExt {
    /// ADC Power On
    ///
    /// If the ADC is in power-down mode then it is powered up. The application needs
    /// to wait a time of about 3 microseconds for stabilization before using the ADC.
    /// If the ADC is already on this function call has no effect.
    ///
    ///  **Note**: Common with F37x
    fn power_on (&mut self);

    /// ADC Start a Conversion Without Trigger
    ///
    /// This initiates a conversion by software without a trigger. The ADC needs to be
    /// powered on before this is called, otherwise this function has no effect.
    ///
    /// **Note**: This is not available in other STM32F families. To ensure code
    /// compatibility, enable triggering and use a software trigger source @see
    /// adc_start_conversion_regular.
    fn start_conversion_direct (&mut self);

    /// ADC Initialize Calibration Registers
    ///
    /// This resets the calibration registers. It is not clear if this is required to be
    /// done before every calibration operation.
    fn reset_calibration (&mut self);

    /// Start the ADC calibration and immediately return
    fn calibrate_async (&mut self);

    /// Is the ADC Calibrating?
    /// return true if the adc is currently calibrating
    fn is_calibrating (&self) -> bool;

    /// Start ADC calibration and wait for it to finish.
    /// The ADC must have been powered down for at least 2 ADC clock cycles, then
    /// powered on before calibration starts
    fn calibrate (&mut self) {
        self.calibrate_async();
        while self.is_calibrating() {};
    }

    /// ADC Set the Sample Time for a Single Channel
    ///
    /// The sampling time can be selected in ADC clock cycles from 1.5 to 239.5.
    ///
    /// * `channel: Channel` - ADC Channel integer 0..18 or from @ref
    /// adc_channel.
    /// * `time: SampleTime` - Sampling time selection from @ref adc_sample_rg.
    ///
    /// **Note**: Common with f2 and f37x and f4
    fn set_sample_time (&mut self, channel: Channel, time: SampleTime);

    /// ADC Set the Sample Time for All Channels
    ///
    /// The sampling time can be selected in ADC clock cycles from 1.5 to 239.5, same
    /// for all channels.
    ///
    /// * `time: SampleTime`- Sampling time selection from @ref adc_sample_rg.
    ///
    /// **Note**: Common with f2 and f37x and f4
    fn set_sample_time_on_all_channels (&mut self, time: SampleTime);


    /// ADC Disable an External Trigger for Regular Channels
    fn disable_external_trigger_regular (&mut self);

    /// ADC Disable an External Trigger for Injected Channels
    fn disable_external_trigger_injected (&mut self);
}

pub trait AdcTrigger {
    /// ADC Enable an External Trigger for Regular Channels
    ///
    /// This enables an external trigger for set of defined regular channels.
    ///
    /// * `trigger: ExternalTrigger` - Trigger identifier for ADC1 and ADC2.
    fn enable_external_trigger_regular (&mut self, trigger: ExternalTrigger);

    /// ADC Enable an External Trigger for Injected Channels
    ///
    /// This enables an external trigger for set of defined injected channels.
    ///
    /// * `trigger: ExternalTriggerInjected` - Trigger identifier
    fn enable_external_trigger_injected (&mut self, trigger: ExternalTriggerInjected);
}

pub trait AdcTrigger3 {
    /// ADC Enable an External Trigger for Regular Channels
    ///
    /// This enables an external trigger for set of defined regular channels.
    ///
    /// * `trigger: ExternalTrigger3`. Trigger identifier for ADC3.
    fn enable_external_trigger_regular (&mut self, trigger: ExternalTrigger3);

    /// ADC Enable an External Trigger for Injected Channels
    ///
    /// This enables an external trigger for set of defined injected channels.
    ///
    /// * `trigger: ExternalTriggerInjected3`. Trigger identifier
    fn enable_external_trigger_injected (&mut self, trigger: ExternalTriggerInjected3);
}

use crate::device::{/*adc1,*/ADC1,ADC2/*,ADC3*/};

const AWDCH_MAX : u8 = 17;
const SQR_MAX_CHANNELS_REGULAR : usize = 16;

/* --- ADC_JOFRx, ADC_HTR, ADC_LTR values ---------------------------------- */

const HT_MSK : u32 = 0xfff;
const LT_MSK : u32 = 0xfff;

const JSQR_JL_LSB : u8 = 20;
const SQR1_L_LSB  : u8 = 20;

const fn jsqr_jsq_val (n: usize, val: u8) -> u32 {
    (val as u32) << ((n - 1) * 5)
}

const fn jsqr_jl_val (val: usize) -> u32 {
    ((val - 1) as u32) << JSQR_JL_LSB
}

macro_rules! impl_adc {
    ($ADCx:ty) => (

    impl AdcExt for $ADCx {
        
        #[inline]
        fn power_off (&mut self) {
            self.cr2      .modify(|_, w| w
                .adon()   .clear_bit()
            );
        }
        
        #[inline]
        fn enable_analog_watchdog_regular (&mut self) {
            self.cr1      .modify(|_, w| w
                .awden()  .set_bit()
            );
        }
        
        #[inline]
        fn disable_analog_watchdog_regular (&mut self) {
            self.cr1      .modify(|_, w| w
                .awden()  .clear_bit()
            );
        }
        
        #[inline]
        fn enable_analog_watchdog_injected (&mut self) {
            self.cr1      .modify(|_, w| w
                .jawden() .set_bit()
            );
        }
        
        #[inline]
        fn disable_analog_watchdog_injected (&mut self) {
            self.cr1      .modify(|_, w| w
                .jawden() .clear_bit()
            );
        }
        
        #[inline]
        fn enable_discontinuous_mode_regular (&mut self, length: u8) {
            assert!((length-1) <= 7);
            self.cr1      .modify(|_, w| w
                .discen() .set_bit()
            );
            self.cr1      .modify(|_, w| unsafe { w
                .discnum().bits(length-1)
            });
        }
        
        #[inline]
        fn disable_discontinuous_mode_regular (&mut self) {
            self.cr1      .modify(|_, w| w
                .discen() .clear_bit()
            );
        }
        
        #[inline]
        fn enable_discontinuous_mode_injected (&mut self) {
            self.cr1      .modify(|_, w| w
                .jdiscen().set_bit()
            );
        }
        
        #[inline]
        fn disable_discontinuous_mode_injected (&mut self) {
            self.cr1      .modify(|_, w| w
                .jdiscen().clear_bit()
            );
        }
        
        #[inline]
        fn enable_automatic_injected_group_conversion (&mut self) {
            self.disable_external_trigger_injected();
            self.cr1      .modify(|_, w| w
                .jauto()  .set_bit()
            );
        }
        
        #[inline]
        fn disable_automatic_injected_group_conversion (&mut self) {
            self.cr1      .modify(|_, w| w
                .jauto()  .clear_bit()
            );
        }
        
        #[inline]
        fn enable_analog_watchdog_on_all_channels (&mut self) {
            self.cr1      .modify(|_, w| w
                .awdsgl() .clear_bit()
            );
        }
        
        #[inline]
        fn enable_analog_watchdog_on_selected_channel (&mut self, channel: Channel) {
            let ch = channel as u8;
            assert!(ch <= AWDCH_MAX);
            self.cr1      .modify(|_, w| unsafe { w
                .awdch()  .bits(ch)
                .awdsgl() .set_bit()
            });
        }
        
        #[inline]
        fn enable_scan_mode (&mut self) {
            self.cr1      .modify(|_, w| w
                .scan()   .set_bit()
            );
        }
        
        #[inline]
        fn disable_scan_mode (&mut self) {
            self.cr1      .modify(|_, w| w
                .scan()   .clear_bit()
            );
        }
        
        #[inline]
        fn enable_eoc_interrupt_injected (&mut self) {
            self.cr1      .modify(|_, w| w
                .jeocie() .set_bit()
            );
        }
        
        #[inline]
        fn disable_eoc_interrupt_injected (&mut self) {
            self.cr1      .modify(|_, w| w
                .jeocie() .clear_bit()
            );
        }
        
        #[inline]
        fn enable_awd_interrupt (&mut self) {
            self.cr1      .modify(|_, w| w
                .awdie()  .set_bit()
            );
        }
        
        #[inline]
        fn disable_awd_interrupt (&mut self) {
            self.cr1      .modify(|_, w| w
                .awdie()  .clear_bit()
            );
        }
        
        #[inline]
        fn enable_eoc_interrupt (&mut self) {
            self.cr1      .modify(|_, w| w
                .eocie()  .set_bit()
            );
        }
        
        #[inline]
        fn disable_eoc_interrupt (&mut self) {
            self.cr1      .modify(|_, w| w
                .eocie()  .clear_bit()
            );
        }
        
        #[inline]
        fn set_left_aligned (&mut self) {
            self.cr2      .modify(|_, w| w
                .align()  .set_bit()
            );
        }
        
        #[inline]
        fn set_right_aligned (&mut self) {
            self.cr2      .modify(|_, w| w
                .align()  .clear_bit()
            );
        }
        
        #[inline]
        fn eoc (&self) -> bool {
            self.sr       .read()
                .eoc()    .bit_is_set()
        }
        
        #[inline]
        fn eoc_injected (&self) -> bool {
            self.sr       .read()
                .jeoc()   .bit_is_set()
        }
        
        #[inline]
        fn read_regular (&self) -> u32 {
            self.dr   .read().bits()
        }
        
        #[inline]
        fn read_injected (&self, reg: InjectedChannel) -> u32 {
            match reg {
                InjectedChannel::C1 => self.jdr1.read().bits(),
                InjectedChannel::C2 => self.jdr2.read().bits(),
                InjectedChannel::C3 => self.jdr3.read().bits(),
                InjectedChannel::C4 => self.jdr4.read().bits(),
            }
        }
        
        #[inline]
        fn set_continuous_conversion_mode (&mut self) {
            self.cr2      .modify(|_, w| w
                .cont()   .set_bit()
            );
        }
        
        #[inline]
        fn set_single_conversion_mode (&mut self) {
            self.cr2      .modify(|_, w| w
                .cont()   .clear_bit()
            );
        }
        
        #[inline]
        fn set_watchdog_high_threshold (&mut self, threshold: u16) {
            self.htr      .write(|w| unsafe {w
                .bits( (threshold as u32) & HT_MSK )
            });
        }
        
        #[inline]
        fn set_watchdog_low_threshold (&mut self, threshold: u16) {
            self.ltr      .write(|w| unsafe {w
                .bits( (threshold as u32) & LT_MSK )
            });
        }
        
        #[inline]
        fn set_regular_sequence (&mut self, channels: &[Channel]) {
            let length = channels.len();
            assert!(length <= SQR_MAX_CHANNELS_REGULAR);
           
            /*let fst : [u32] = [0,6,12].iter().map(|x| {
                (1+x..7+x).fold(0u32, |c, i| c | (channels[i - 1] as u8 as u32 << ((i - x - 1) * 5)) );
            }).collect();
            let (first6, second6, third6) = (fst[0], fst[1], fst[2]);*/
            let first6 = (1..7).fold(0u32, |c, i|
                c | ((channels[i - 1] as u8 as u32) << ((i - 1) * 5))
            );
            let second6  = (7..13).fold(0u32, |c, i|
                c | ((channels[i - 1] as u8 as u32) << ((i - 6 - 1) * 5))
            );
            let third6  = (13..19).fold(0u32, |c, i|
                c | ((channels[i - 1] as u8 as u32) << ((i - 12 - 1) * 5))
            );
            
            //if cfg!(self.sqr5) {
            /*if AWDCH_MAX == 28 {
                let fourth6 = (19..25).fold(0u32, |c, i| c | (channels[i - 1] as u8 as u32 << ((i - 18 - 1) * 5)) );
                let fifth6  = (25..29).fold(0u32, |c, i| c | (channels[i - 1] as u8 as u32 << ((i - 24 - 1) * 5)) );
            
                self.sqr1    .modify(|_, w| w
                    .bits( fifth6 | ((length - 1) as u32 << SQR1_L_LSB) )
                );
                self.sqr2    .modify(|_, w| w
                    .bits( fourth6 )
                );
                self.sqr3    .modify(|_, w| w
                    .bits( third6 )
                );
                self.sqr4    .modify(|_, w| w
                    .bits( second6 )
                );
                self.sqr5    .modify(|_, w| w
                    .bits( first6 )
                );
            } else {*/
                self.sqr1    .write(|w| unsafe { w
                    .bits( third6 | (((length - 1) as u32) << SQR1_L_LSB))
                });
                self.sqr2    .write(|w| unsafe { w
                    .bits( second6 )
                });
                self.sqr3    .write(|w| unsafe { w
                    .bits( first6 )
                });
            //}
        }
        
        #[inline]
        fn set_injected_sequence (&mut self, channels: &[Channel]) {
            let mut reg32 = 0u32;
            
            let length = channels.len();
            /* Maximum sequence length is 4 channels. Minimum sequence is 1.*/
            assert!((length - 1) <= 3);
        
            for i in 0..length {
                reg32 |= jsqr_jsq_val(4 - i, channels[length - i - 1] as u8);
            }
        
            reg32 |= jsqr_jl_val(length);
        
            self.jsqr    .write(|w| unsafe { w
                .bits( reg32 )
            });
        }
        
        #[inline]
        fn set_injected_offset (&mut self, reg: InjectedChannel, offset: u32) {
            let jofr = match reg {
                InjectedChannel::C1 => &self.jofr1,
                InjectedChannel::C2 => &self.jofr2,
                InjectedChannel::C3 => &self.jofr3,
                InjectedChannel::C4 => &self.jofr4,
            };
            jofr.write(|w| unsafe { w
                .bits( offset )
            });
        }
        
        #[inline]
        fn start_conversion_regular (&mut self) {
            // Start conversion on regular channels.
            self.cr2      .modify(|_, w| w
                .swstart().set_bit()
            );
            // Wait until the ADC starts the conversion.
            while self.cr2.read().swstart().bit_is_set() {}
        }
        
        #[inline]
        fn start_conversion_injected (&mut self) {
            // Start conversion on injected channels.
            self.cr2      .modify(|_, w| w
                .jswstart().set_bit()
            );
            // Wait until the ADC starts the conversion.
            while self.cr2.read().jswstart().bit_is_set() {}
        }
        
        
        #[inline]
        fn enable_dma (&mut self) {
            self.cr2      .modify(|_, w| w
                .dma()    .set_bit()
            );
        }
        
        #[inline]
        fn disable_dma (&mut self) {
            self.cr2      .modify(|_, w| w
                .dma()    .clear_bit()
            );
        }

    }
    
    //#[cfg(feature="stm32f1")]
    impl AdcSpecialExt for $ADCx {
        
        #[inline]
        fn power_on (&mut self) {
            if self.cr2.read().adon().bit_is_clear() {
                self.cr2    .modify(|_, w| w
                    .adon()      .set_bit()
                );
            }
        }
        
        #[inline]
        fn start_conversion_direct (&mut self) {
            if self.cr2.read().adon().bit_is_set() {
                self.cr2 .modify(|_, w| w
                    .adon().set_bit()
                );
            }
        }
        
        #[inline]
        fn disable_external_trigger_regular (&mut self) {
            self.cr2     .modify(|_, w| w
                .exttrig() .clear_bit()
            );
        }
        
        #[inline]
        fn disable_external_trigger_injected (&mut self) {
            self.cr2     .modify(|_, w| w
                .jexttrig().clear_bit()
            );
        }
        
        #[inline]
        fn reset_calibration (&mut self) {
            self.cr2     .modify(|_, w| w
                .rstcal()  .set_bit()
            );
            while self.cr2.read().rstcal().bit_is_set() {};
        }
        
        #[inline]
        fn calibrate_async (&mut self) {
            self.cr2     .modify(|_, w| w
                .cal()     .set_bit()
            );
        }
        
        #[inline]
        fn is_calibrating (&self) -> bool {
            self.cr2.read().cal().bit_is_set()
        }
        
        #[inline]
        fn set_sample_time (&mut self, channel: Channel, time: SampleTime) {
            let chan = channel as u32;
            let (ch, reg32) = if chan < 10 {
                    (chan, self.smpr2.read().bits())
                } else {
                    (chan-10, self.smpr1.read().bits())
                };
            let mut reg32 = reg32 & !(0x7 << (ch * 3));
            reg32 |= u32::from(time) << (ch * 3);
            if chan < 10 {
                self.smpr2.write(|w| unsafe { w.bits(reg32) });
            } else { 
                self.smpr1.write(|w| unsafe { w.bits(reg32) });
            }
        }
        
        #[inline]
        fn set_sample_time_on_all_channels (&mut self, time: SampleTime) {
            let reg32 = (0..10).fold(0u32, |c, i|
                c | ((time as u32) << (i * 3) )
            );
            self.smpr2.write(|w| unsafe { w.bits(reg32) });
            let reg32 = (10..18).fold(0u32, |c, i|
                c | ( u32::from(time) << ((i-10) * 3) )
            );
            self.smpr1.write(|w| unsafe { w.bits(reg32) });
        }
        
    }
)}

macro_rules! impl_adc_trigger12 {
    ($ADCx:ty) => (

    impl AdcTrigger for $ADCx {
        
        #[inline]
        fn enable_external_trigger_regular (&mut self, trigger: ExternalTrigger) {
            self.cr2     .modify(|_, w| unsafe { w
                .extsel()  .bits( trigger.into() )
                .exttrig() .set_bit()
            });
        }
        
        #[inline]
        fn enable_external_trigger_injected (&mut self, trigger: ExternalTriggerInjected) {
            self.cr2     .modify(|_, w| unsafe { w
                .jextsel() .bits( trigger.into() )
                .jexttrig().set_bit()
            });
        }
        
    }
)}


impl_adc!(ADC1);
impl_adc!(ADC2);
impl_adc_trigger12!(ADC1);
impl_adc_trigger12!(ADC2);
/*impl_adc!(ADC3);*/

impl AdcDualExt for ADC1 {
    #[inline]
    fn set_dual_mode (&mut self, mode: DualMode) {
        self.cr1     .modify(|_, w| unsafe {w
            .dualmod().bits( mode.into() )
        });
    }
}

impl AdcTemperatureExt for ADC1 {
    #[inline]
    fn enable_temperature_sensor (&mut self) {
        self.cr2       .modify(|_, w| w
            .tsvrefe() .set_bit()
        );
    }
    
    #[inline]
    fn disable_temperature_sensor (&mut self) {
        self.cr2     .modify(|_, w| w
            .tsvrefe() .clear_bit()
        );
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum DualMode {
    /// Independent (non-dual) mode
    Independent,
    /// Combined regular simultaneous + injected simultaneous mode
    RegInjecSimult,
    /// Combined regular simultaneous + alternate trigger mode
    RegSimultAlterTrig,
    /// Combined injected simultaneous + fast interleaved mode
    InjecSimultFastInterl,
    /// Combined injected simultaneous + slow interleaved mode
    InjecSimultSlowInterl,
    /// Injected simultaneous mode only
    InjecSimult,
    /// Regular simultaneous mode only
    RegSimult,
    /// Fast interleaved mode only
    FastInterl,
    /// Slow interleaved mode only
    SlowInterl,
    /// Alternate trigger mode only
    AlterTrig
}

impl From<DualMode> for u8 {
    fn from (mode: DualMode) -> u8 {
        match mode {
            DualMode::Independent            => 0,
            DualMode::RegInjecSimult         => 1,
            DualMode::RegSimultAlterTrig     => 2,
            DualMode::InjecSimultFastInterl  => 3,
            DualMode::InjecSimultSlowInterl  => 4,
            DualMode::InjecSimult            => 5,
            DualMode::RegSimult              => 6,
            DualMode::FastInterl             => 7,
            DualMode::SlowInterl             => 8,
            DualMode::AlterTrig              => 9
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum ExternalTrigger {
    /// Timer 1 Compare Output 1
    TIM1_CC1,
    /// Timer 1 Compare Output 2
    TIM1_CC2,
    /// Timer 1 Compare Output 3
    TIM1_CC3,
    /// Timer 2 Compare Output 2
    TIM2_CC2,
    /// Timer 3 Trigger Output
    TIM3_TRGO,
    /// Timer 4 Compare Output 4
    TIM4_CC4,
    /// External Interrupt 11
    EXTI11,
    /// Software Trigger
    SWSTART
}

impl From<ExternalTrigger> for u8 {
    fn from (trigger: ExternalTrigger) -> u8 {
        match trigger {
            ExternalTrigger::TIM1_CC1  => 0,
            ExternalTrigger::TIM1_CC2  => 1,
            ExternalTrigger::TIM1_CC3  => 2,
            ExternalTrigger::TIM2_CC2  => 3,
            ExternalTrigger::TIM3_TRGO => 4,
            ExternalTrigger::TIM4_CC4  => 5,
            ExternalTrigger::EXTI11    => 6,
            ExternalTrigger::SWSTART   => 7
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum ExternalTriggerInjected {
    /// Timer 1 Trigger Output
    TIM1_TRGO,
    /// Timer 1 Compare Output 4
    TIM1_CC4,
    /// Timer 2 Trigger Output
    TIM2_TRGO,
    /// Timer 2 Compare Output 1
    TIM2_CC1,
    /// Timer 3 Compare Output 4
    TIM3_CC4,
    /// Timer 4 Trigger Output
    TIM4_TRGO,
    /// External Interrupt 15
    EXTI15,
    /// Injected Software Trigger
    JSWSTART /* Software start. */
}

impl From<ExternalTriggerInjected> for u8 {
    fn from (trigger: ExternalTriggerInjected) -> u8 {
        match trigger {
            ExternalTriggerInjected::TIM1_TRGO => 0,
            ExternalTriggerInjected::TIM1_CC4  => 1,
            ExternalTriggerInjected::TIM2_TRGO => 2,
            ExternalTriggerInjected::TIM2_CC1  => 3,
            ExternalTriggerInjected::TIM3_CC4  => 4,
            ExternalTriggerInjected::TIM4_TRGO => 5,
            ExternalTriggerInjected::EXTI15    => 6,
            ExternalTriggerInjected::JSWSTART  => 7
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum ExternalTrigger3 {
    /// Timer 2 Compare Output 1
    TIM3_CC1 = 0,
    /// Timer 2 Compare Output 3
    TIM2_CC3 = 1,
    /// Timer 1 Compare Output 3
    TIM1_CC3 = 2,
    /// Timer 8 Compare Output 1
    TIM8_CC1 = 3,
    /// Timer 8 Trigger Output
    TIM8_TRGO = 4,
    /// Timer 5 Compare Output 1
    TIM5_CC1 = 5,
    /// Timer 5 Compare Output 3
    TIM5_CC3 = 6
}
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, PartialEq)]
pub enum ExternalTriggerInjected3 {
    /// Timer 1 Trigger Output
    TIM1_TRGO = 0,
    /// Timer 1 Compare Output 4
    TIM1_CC4 = 1,
    /// Timer 4 Compare Output 3
    TIM4_CC3 = 2,
    /// Timer 8 Compare Output 2
    TIM8_CC2 = 3,
    /// Timer 8 Compare Output 4
    TIM8_CC4 = 4,
    /// Timer 5 Trigger Output
    TIM5_TRGO = 5,
    /// Timer 5 Compare Output 4
    TIM5_CC4 = 6,
    /// Injected Software Trigger
    JSWSTART = 7 /* Software start. */
}

#[derive(Clone, Copy, PartialEq)]
pub enum SampleTime {
    Cycles1_5,
    Cycles7_5,
    Cycles13_5,
    Cycles28_5,
    Cycles41_5,
    Cycles55_5,
    Cycles71_5,
    Cycles239_5
}

impl From<SampleTime> for u32 {
    fn from (time: SampleTime) -> u32 {
        match time {
            SampleTime::Cycles1_5   => 0,
            SampleTime::Cycles7_5   => 1,
            SampleTime::Cycles13_5  => 2,
            SampleTime::Cycles28_5  => 3,
            SampleTime::Cycles41_5  => 4,
            SampleTime::Cycles55_5  => 5,
            SampleTime::Cycles71_5  => 6,
            SampleTime::Cycles239_5 => 7
        }
    }
}
