macro_rules! bitortype {
    ($TYPE:ident, $VAL:ty) => {
        #[derive(Clone, Copy, PartialEq, Eq)]
        pub struct $TYPE($VAL);
        impl $TYPE {
            #[inline]
            pub fn value(&self) -> $VAL {
                self.0
            }
        }
        impl core::ops::BitOr for $TYPE {
            #[inline]
            type Output = Self;
            fn bitor(self, rhs: Self) -> Self {
                $TYPE(self.0 | rhs.0)
            }
        }
    };
}
