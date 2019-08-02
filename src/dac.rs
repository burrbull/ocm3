//! Digital to Analog Conversion System in the
//! STM32F series of ARM Cortex Microcontrollers by ST Microelectronics.
//! 
//! The DAC is present only in a limited set of devices, notably some
//! of the connection line, high density and XL devices.
//! 
//! Two DAC channels are available, however unlike the ADC channels these
//! are separate DAC devices controlled by the same register block.
//! 
//! The DAC is on APB1. Its clock must be enabled in RCC and depending on
//! specific family, the GPIO
//! ports set to alternate function output before it can be used.
//! On most families, the GPIO pins should be configured to Analog IN to
//! avoid parasitic consumption.
//! The digital output driver is disabled so the output driver mode
//! (push-pull/open drain) is arbitrary.
//! 
//! The DAC has a holding (buffer) register and an output register from
//! which the analog output is derived. The holding register must be
//! loaded first. If triggering is enabled the output register is loaded
//! from the holding register after a trigger occurs. If triggering is
//! not enabled the holding register contents are transferred directly
//! to the output register.
//! 
//! **Note**: To avoid nonlinearities, do not allow outputs to range close
//! to zero or V_analog.
//! 
//! # Dual Channel Conversion
//! 
//! There are dual modes in which both DACs are used to output data
//! simultaneously or independently on both channels. The data must be
//! presented according to the formats described in the datasheets. A
//! convenience function @ref dac_load_data_buffer_dual is provided
//! for software controlled use.
//! 
//! A variety of modes are available depending on whether independent
//! or simultaneous output is desired, and whether waveforms are to be
//! superimposed. Refer to the datasheets.
//! 
//! If DMA is used, only enable it for one of the channels. The DMA
//! requests will then serve data in dual format to the data register
//! dedicated to dual mode. The data will then be split and loaded to the
//! appropriate DAC following the next trigger. There are three registers
//! available, one for each of the formats: 12 bit right-aligned, 12 bit
//! left-aligned and 8 bit right-aligned. The desired format is determined
//! by specifying the appropriate register to the DMA controller.
//! 
//! # Basic DAC handling API
//! 
//! Set the DAC's GPIO port to Analog IN. Enable the
//! DAC clock. Enable the DAC, set a trigger source and load the buffer
//! with the first value. After the DAC is triggered, load the buffer with
//! the next value. This example uses software triggering and added noise.
//! The trigger and further buffer load calls are made when data is to be
//! sent out.
//! 
//! ```
//! gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
//!           GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4);
//! rcc.peripheral_enable_clock(rcc::Enr::Apb1(rcc::enr::DACEN));
//! dac1.disable();
//! dac1.set_waveform_characteristics(dac::Mamp::M8);
//! dac1.set_waveform_generation(dac::Wave::NOISE);
//! dac1.enable();
//! dac1.set_trigger_source(DAC_CR_TSEL1_SW);
//! dac1.load_data_buffer_single(0, DataAlign::Right12);
//! ....
//! dac1.software_trigger();
//! dac1.load_data_buffer_single(value, DataAlign::Right12);
//! ```
//! 
//! # Simultaneous Dual DAC with DMA
//! 
//! This example in part sets up the DAC channel 1 DMA (DMA2 channel 3) to read
//! 16 bit data from memory into the right-aligned 8 bit dual register DAC_DHR8RD.
//! Both DAC channels are enabled, and both triggers are set to the same timer
//! 2 input as required for simultaneous operation. DMA is enabled for DAC channel
//! 1 only to ensure that only one DMA request is generated.
//!
//! ```
//! dma2ch3.set_memory_size(dma::DataSize::BIT16);
//! dma2ch3.set_peripheral_size(dma::DataSize::BIT16);
//! dma2ch3.set_read_from_memory();
//! dma2ch3.set_peripheral_address(&{ *DAC::ptr() }.dhr8rd as *const _ as u32);
//! dma2ch3.enable_channel();
//! ...
//! dacd.trigger_enable();
//! dacd.set_trigger_source(dac::trigger1::T2, dac::trigger2::T2);
//! dac1.dma_enable();
//! dacd.enable();
//! ```

use crate::device::{dac,DAC};


/** DAC data size (8/12 bits), alignment (right/left) */
#[derive(Clone, Copy, PartialEq)]
pub enum DataAlign {
    /// right-aligned 8 bit data in bits 0-7
    Right8,
    /// right-aligned 12 bit data in bits 0-11
    Right12,
    /// left aligned 12 bit data in bits 4-15
    Left12
}

/// DAC Channel 1 Trigger Source Selection
///
/// **Note**: Refer to the timer documentation for details of the TRGO event.
///
/// **Note**: T3 replaced by T8 and T5 replaced by T15 in some devices.
///
/// **Note**: this is <b>not</b> valid for the STM32L1 family.
///
/// **Note**: only used if bit TEN2 is set (DAC channel 1 trigger enabled).
#[allow(non_camel_case_types)]
pub enum Trigger1Sel {
    /// Timer 6 TRGO event
    T6 = 0,
    /// Timer 3 TRGO event
    ///
    /// Timer 8 TRGO event
    T3_8 = 1,
    /// Timer 7 TRGO event
    T7 = 12,
    /// Timer 5 TRGO event
    ///
    /// Timer 15 TRGO event
    T5_15 = 13,
    /// Timer 2 TRGO event
    T2 = 14,
    /// Timer 4 TRGO event
    T4 = 15,
    /// External line 9
    E9 = 16,
    /// Software trigger
    SW = 17,
}

/// DAC Channel 2 Trigger Source Selection
///
/// **Note**: Refer to the timer documentation for details of the TRGO event.
///
/// **Note**: T3 replaced by T8 and T5 replaced by T15 in some devices.
///
/// **Note**: this is <b>not</b> valid for the STM32L1 family.
///
/// **Note**: only used if bit TEN2 is set (DAC channel 2 trigger enabled)
#[allow(non_camel_case_types)]
pub enum Trigger2Sel {
    /// Timer 6 TRGO event
    T6 = 10,
    /// Timer 3 TRGO event
    ///
    /// Timer 8 TRGO event
    T3_8 = 11,
    /// Timer 7 TRGO event
    T7 = 12,
    /// Timer 5 TRGO event
    ///
    /// Timer 15 TRGO event
    T5_15 = 13,
    /// Timer 2 TRGO event
    T2 = 14,
    /// Timer 4 TRGO event
    T4 = 15,
    /// External line9
    E9 = 16,
    /// Software trigger
    SW = 17,
}


/// DAC channel1 noise/triangle wave generation enable
///
/// **Note**: only used if bit TEN1 = 1 (DAC channel1 trigger enabled)
#[derive(Clone, Copy, PartialEq)]
pub enum Wave {
    /// wave generation disabled
    DIS = 0,
    /// Noise wave generation enabled
    NOISE = 1,
    /// Triangle wave generation enabled
    TRI = 2
}

/// DAC Channel 1 LFSR Mask and Triangle Wave Amplitude
/// Unmask bits [(n-1)..0] of LFSR/Triangle Amplitude equal to (2**(n+1)-1
#[derive(Clone, Copy, PartialEq)]
pub enum Mamp {
    M1 = 0x0,
    M2 = 0x1,
    M3 = 0x2,
    M4 = 0x3,
    M5 = 0x4,
    M6 = 0x5,
    M7 = 0x6,
    M8 = 0x7,
    M9 = 0x8,
    M10 = 0x9,
    M11 = 0xA,
    M12 = 0xB,
}

pub struct Channel1;
pub struct Channel2;
pub struct ChannelDual;

pub trait DacChannelExt {
    /// DAC Channel Enable
    ///
    /// Enable a digital to analog converter channel. After setting this enable, the
    /// DAC requires a t<sub>wakeup</sub> time typically around 10 microseconds before
    /// it actually wakes up.
    fn enable(&mut self);
    
    /// DAC Channel Disable.
    ///
    /// Disable a digital to analog converter channel.
    fn disable(&mut self);
    
    /// DAC Channel Output Buffer Enable
    ///
    /// Enable a digital to analog converter channel output drive buffer. This is an
    /// optional amplifying buffer that provides additional drive for the output
    /// signal. The buffer is enabled by default after a reset and needs to be
    /// explicitly disabled if required.
    fn buffer_enable(&mut self);
    
    /// DAC Channel Output Buffer Disable
    ///
    /// Disable a digital to analog converter channel output drive buffer. Disabling
    /// this will reduce power consumption slightly and will increase the output
    /// impedance of the DAC.  The buffers are enabled by default after a reset.
    fn buffer_disable(&mut self);
    
    /// DAC Channel DMA Enable
    ///
    /// Enable a digital to analog converter channel DMA mode (connected to DMA2 channel
    /// 3 for DAC channel 1 and DMA2 channel 4 for DAC channel 2). A DMA request is
    /// generated following an external trigger.
    fn dma_enable(&mut self);
    
    /// DAC Channel DMA Disable
    ///
    /// Disable a digital to analog converter channel DMA mode.
    fn dma_disable(&mut self);
    
    /// DAC Channel Trigger Enable
    ///
    /// Enable a digital to analog converter channel external trigger mode. This allows
    /// an external trigger to initiate register transfers from the buffer register to
    /// the DAC output register, followed by a DMA transfer to the buffer register if
    /// DMA is enabled.  The trigger source must also be selected.
    fn trigger_enable(&mut self);
    
    /// DAC Channel Trigger Disable
    ///
    /// Disable a digital to analog converter channel external trigger.
    fn trigger_disable(&mut self);
    
    /// Disable DAC Channel Waveform Generation
    ///
    /// Disable a digital to analog converter channel superimposed waveform generation.
    fn disable_waveform_generation(&mut self);
    
    /// Trigger the DAC by a Software Trigger
    ///
    /// If the trigger source is set to be a software trigger, cause a trigger to occur.
    /// The trigger is cleared by hardware after conversion.
    fn software_trigger(&self);
}

pub trait DacChannelSingleExt {

    /// Load DAC Data Register.
    ///
    /// Loads the appropriate digital to analog converter data register with 12 or 8 bit
    /// data to be converted on a channel.
    ///
    /// * `dac_data : u16` - with appropriate alignment.
    /// * `dac_data_format : DataAlign` - Alignment and size.
    fn load_data_buffer(&self, dac_data : u16, dac_data_format : DataAlign);
    
    /// Enable and Set DAC Channel Waveform Generation.
    ///
    /// Enable the digital to analog converter waveform generation as either
    /// pseudo-random noise or triangular wave. These signals are superimposed on
    /// existing output values in the DAC output registers.
    ///
    /// **Note**: The DAC trigger must be enabled for this to work.
    ///
    /// * `dac_wave_ens : Wave`
    fn set_waveform_generation(&mut self, dac_wave_ens : Wave);
    
    /// Set DAC Channel LFSR Mask or Triangle Wave Amplitude.
    ///
    /// Sets the digital to analog converter superimposed waveform generation
    /// characteristics.  @li If the noise generation mode is set, this sets the length
    /// of the PRBS sequence and hence the amplitude of the output noise signal.
    /// Default setting is length 1.  @li If the triangle wave generation mode is set,
    /// this sets the amplitude of the output signal as 2^(n)-1 where n is the
    /// parameter value. Default setting is 1.
    ///
    /// **Note**: High amplitude levels of these waveforms can overload the DAC and distort
    /// the signal output.
    ///
    /// **Note**: This must be called before enabling the DAC as the settings will then
    /// become read-only.
    ///
    /// **Note**: The DAC trigger must be enabled for this to work.
    ///
    /// * `dac_mamp : Mamp`
    fn set_waveform_characteristics(&mut self, dac_mamp : Mamp);
}

impl Channel1 {
    pub fn cr (&mut self) -> &dac::CR {
        &unsafe { &*DAC::ptr() } .cr
    }
    
    /// Set DAC Channel Trigger Source.
    ///
    /// Sets the digital to analog converter trigger source, which can be taken from
    /// various timers, an external trigger or a software trigger.
    ///
    /// * `dac_trig_src : Trigger1Sel`
    pub fn set_trigger_source(&mut self, dac_trig_src : Trigger1Sel) {
        self.cr()    .modify(|_,w| unsafe { w
            .tsel1() .bits( dac_trig_src as u8 )
        });
    }
}

impl Channel2 {
    pub fn cr (&mut self) -> &dac::CR {
        &unsafe { &*DAC::ptr() } .cr
    }
    
    /// Set DAC Channel Trigger Source.
    ///
    /// Sets the digital to analog converter trigger source, which can be taken from
    /// various timers, an external trigger or a software trigger.
    ///
    /// * `dac_trig_src : Trigger2Sel`
    pub fn set_trigger_source(&mut self, dac_trig_src : Trigger2Sel) {
        self.cr()    .modify(|_,w| w
            .tsel2() .bits( dac_trig_src as u8 )
        );
    }
    
}

impl ChannelDual {
    pub fn cr (&mut self) -> &dac::CR {
        &unsafe { &*DAC::ptr() } .cr
    }
    
    /// Load DAC Dual Data Register.
    ///
    /// Loads the appropriate digital to analog converter dual data register with 12 or
    /// 8 bit data to be converted for both channels. This allows high bandwidth
    /// simultaneous or independent analog output. The data in both channels are aligned
    /// identically.
    ///
    /// * `dac_data1 : u16` for channel 1 with appropriate alignment.
    /// * `dac_data2 : u16` for channel 2 with appropriate alignment.
    /// * `dac_data_format : DataAlign` - Right or left aligned, and 8 or
    /// 12 bit.
    pub fn load_data_buffer(&self, dac_data1 : u16, dac_data2 : u16,
        dac_data_format : DataAlign) {
        let dac = unsafe { &*DAC::ptr() };
        match dac_data_format {
            DataAlign::Right8 => {
                dac.dhr8rd      .write(|w| w
                    .dacc1dhr()   .bits ( dac_data1 as u8 )
                    .dacc2dhr()   .bits ( dac_data2 as u8 )
                );
            },
            DataAlign::Right12 => {
                dac.dhr12rd      .write(|w| unsafe { w
                    .dacc1dhr()   .bits ( dac_data1 )
                    .dacc2dhr()   .bits ( dac_data2 )
                });
            },
            DataAlign::Left12 => {
                dac.dhr12ld      .write(|w| unsafe { w
                    .bits ( (dac_data1 as u32) | ((dac_data2 as u32) << 16) )
                });
            }
        }
    }

    /// Set DAC Channel Trigger Source.
    ///
    /// Sets the digital to analog converter trigger source, which can be taken from
    /// various timers, an external trigger or a software trigger.
    ///
    /// * `dac_trig_src1 : Trigger1Sel`
    /// * `dac_trig_src2 : Trigger2Sel`
    pub fn set_trigger_source(&mut self, dac_trig_src1 : Trigger1Sel, dac_trig_src2 : Trigger2Sel) {
        self.cr()   .modify(|_,w| unsafe { w
            .tsel1() .bits( dac_trig_src1 as u8 )
            .tsel2() .bits( dac_trig_src2 as u8 )
        });
    }
    
    /// Enable and Set DAC Channel Waveform Generation.
    ///
    /// Enable the digital to analog converter waveform generation as either
    /// pseudo-random noise or triangular wave. These signals are superimposed on
    /// existing output values in the DAC output registers.
    ///
    /// **Note**: The DAC trigger must be enabled for this to work.
    ///
    /// * `dac_wave_ens1 : Wave`
    /// * `dac_wave_ens2 : Wave`
    pub fn set_waveform_generation(&mut self, dac_wave_ens1 : Wave, dac_wave_ens2 : Wave) {
        self.cr()        .modify(|_,w| unsafe { w
            .wave1() .bits( dac_wave_ens1 as u8 )
            .wave2() .bits( dac_wave_ens2 as u8 )
        });
    }
    
    /// Set DAC Channel LFSR Mask or Triangle Wave Amplitude.
    ///
    /// Sets the digital to analog converter superimposed waveform generation
    /// characteristics.  @li If the noise generation mode is set, this sets the length
    /// of the PRBS sequence and hence the amplitude of the output noise signal.
    /// Default setting is length 1.  @li If the triangle wave generation mode is set,
    /// this sets the amplitude of the output signal as 2^(n)-1 where n is the
    /// parameter value. Default setting is 1.
    ///
    /// **Note**: High amplitude levels of these waveforms can overload the DAC and distort
    /// the signal output.
    ///
    /// **Note**: This must be called before enabling the DAC as the settings will then
    /// become read-only.
    ///
    /// **Note**: The DAC trigger must be enabled for this to work.
    ///
    /// * `dac_mamp1 : Mamp`
    /// * `dac_mamp2 : Mamp`
    pub fn set_waveform_characteristics(&mut self, dac_mamp1 : Mamp, dac_mamp2 : Mamp) {
        self.cr()        .modify(|_,w| w
            .mamp1() .bits( dac_mamp1 as u8 )
            .mamp2() .bits( dac_mamp2 as u8 )
        );
    }
}

impl DacChannelExt for Channel1 {
    
    fn enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .en1().set_bit()
        );
    }
    
    fn disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .en1().clear_bit()
        );
    }
    
    fn buffer_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .boff1().clear_bit()
        );
    }
    
    fn buffer_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .boff1().set_bit()
        );
    }
    
    fn dma_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .dmaen1().set_bit()
        );
    }
    
    fn dma_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .dmaen1().clear_bit()
        );
    }
    
    fn trigger_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .ten1().set_bit()
        );
    }
    
    fn trigger_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .ten1().clear_bit()
        );
    }
    
    fn disable_waveform_generation(&mut self) {
        self.cr()    .modify(|_,w| unsafe { w
            .wave1().bits(0b00)
        });
    }
    
    fn software_trigger(&self) {
        unsafe { &*DAC::ptr() } .swtrigr    .write(|w| w
            .swtrig1().set_bit()
        );
    }
     
}

impl DacChannelExt for Channel2 {
    
    fn enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .en2().set_bit()
        );
    }
    
    fn disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .en2().clear_bit()
        );
    }
    
    fn buffer_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .boff2().clear_bit()
        );
    }
    
    fn buffer_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .boff2().set_bit()
        );
    }
    
    fn dma_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .dmaen2().set_bit()
        );
    }
    
    fn dma_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .dmaen2().clear_bit()
        );
    }
    
    fn trigger_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .ten2().set_bit()
        );
    }
    
    fn trigger_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .ten2().clear_bit()
        );
    }
    
    fn disable_waveform_generation(&mut self) {
        self.cr()    .modify(|_,w| unsafe { w
            .wave2().bits(0b00)
        });
    }
    
    fn software_trigger(&self) {
        unsafe { &*DAC::ptr() } .swtrigr    .write(|w| w
            .swtrig2().set_bit()
        );
    }
    
}

impl DacChannelExt for ChannelDual {
    
    fn enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .en1().set_bit()
            .en2().set_bit()
        );
    }
    
    fn disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .en1().clear_bit()
            .en2().clear_bit()
        );
    }
    
    fn buffer_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .boff1().clear_bit()
            .boff2().clear_bit()
        );
    }
    
    fn buffer_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .boff1().set_bit()
            .boff2().set_bit()
        );
    }
    
    fn dma_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .dmaen1().set_bit()
            .dmaen2().set_bit()
        );
    }
    
    fn dma_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .dmaen1().clear_bit()
            .dmaen2().clear_bit()
        );
    }
    
    fn trigger_enable(&mut self) {
        self.cr()    .modify(|_,w| w
            .ten1().set_bit()
            .ten2().set_bit()
        );
    }
    
    fn trigger_disable(&mut self) {
        self.cr()    .modify(|_,w| w
            .ten1().clear_bit()
            .ten2().clear_bit()
        );
    }
    
    fn disable_waveform_generation(&mut self) {
        self.cr()    .modify(|_,w| unsafe { w
            .wave1().bits(0b00)
            .wave2().bits(0b00)
        });
    }
    
    fn software_trigger(&self) {
        unsafe { &*DAC::ptr() } .swtrigr    .write(|w| w
            .swtrig1().set_bit()
            .swtrig2().set_bit()
        );
    }
    
}


impl DacChannelSingleExt for Channel1 {
    fn load_data_buffer(&self, dac_data : u16, dac_data_format : DataAlign) {
        let dac = unsafe { &*DAC::ptr() };
        match dac_data_format {
            DataAlign::Right8 => {
                dac.dhr8r1      .write(|w| unsafe { w
                    .bits ( dac_data as u32 )
                });
            },
            DataAlign::Right12 => {
                dac.dhr12r1     .write(|w| unsafe { w
                    .bits ( dac_data as u32 )
                });
            },
            DataAlign::Left12 => {
                dac.dhr12l1     .write(|w| unsafe { w
                    .bits ( dac_data as u32 )
                });
            }
        }
    }
    
    fn set_waveform_generation(&mut self, dac_wave_ens : Wave) {
        self.cr()        .modify(|_,w| unsafe { w
            .wave1() .bits( dac_wave_ens as u8 )
        });
    }
    
    fn set_waveform_characteristics(&mut self, dac_mamp : Mamp) {
        self.cr()        .modify(|_,w| w
            .mamp1() .bits( dac_mamp as u8 )
        );
    }
}

impl DacChannelSingleExt for Channel2 {
    fn load_data_buffer(&self, dac_data : u16, dac_data_format : DataAlign) {
        let dac = unsafe { &*DAC::ptr() };
        match dac_data_format {
            DataAlign::Right8 => {
                dac.dhr8r2      .write(|w| unsafe { w
                    .bits ( dac_data as u32 )
                });
            },
            DataAlign::Right12 => {
                dac.dhr12r2     .write(|w| unsafe { w
                    .bits ( dac_data as u32 )
                });
            },
            DataAlign::Left12 => {
                dac.dhr12l2     .write(|w| unsafe { w
                    .bits ( dac_data as u32 )
                });
            }
        }
    }
    
    fn set_waveform_generation(&mut self, dac_wave_ens : Wave) {
        self.cr()        .modify(|_,w| unsafe { w
            .wave2() .bits( dac_wave_ens as u8 )
        });
    }
    
    fn set_waveform_characteristics(&mut self, dac_mamp : Mamp) {
        self.cr()        .modify(|_,w| w
            .mamp2() .bits( dac_mamp as u8 )
        );
    }
}

