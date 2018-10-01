pub trait CrcExt {
    /// Reset the CRC calculator to initial values.
    fn reset (&mut self);
    
    /// Add a word to the CRC calculator and return the result.
    ///
    /// * `datap: u32` - new word to add to the CRC calculator
    ///
    /// Returns final CRC calculator value
    fn calculate (&mut self, datap: u32) -> u32;
    
    /// Add a block of data to the CRC calculator and return the final result
    ///
    /// * `datap: &[u32]` - block of 32bit data words
    ///
    /// Returns final CRC calculator value
    fn calculate_block (&mut self, datap: &[u32]) -> u32;
}

use crate::device::CRC;

impl CrcExt for CRC {
    /// CRC Reset.
    ///
    /// Reset the CRC unit and forces the data register to all 1s.
    #[inline]
    fn reset (&mut self) {
        self.cr     .write(|w| w
            .reset()  .set_bit()
        );
    }

    /// CRC Calculate.
    ///
    /// Writes a data word to the register, the write operation stalling until the
    /// computation is complete.
    ///
    /// * `data: u32`.
    ///
    /// Returns `u32` Computed CRC result
    #[inline]
    fn calculate (&mut self, data: u32) -> u32 {
        self.dr     .write(|w| unsafe { w
            .bits(data)
        });
        self.dr.read().bits()
    }

    /// CRC Calculate of a Block of Data.
    ///
    /// Writes data words consecutively to the register, the write operation stalling
    /// until the computation of each word is complete.
    ///
    /// * `datap: &[u32]` - array of 32 bit data words.
    ///
    /// Returns `u32` Final computed CRC result
    #[inline]
    fn calculate_block (&mut self, datap: &[u32]) -> u32 {
        for x in datap {
            self.dr .write(|w| unsafe { w
                .bits(*x)
            });
        }
        self.dr.read().bits()
    }

}
