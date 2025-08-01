//! Multiplexed I/O (MIO) module.
//!
//! This module provides a [singleton][Pins] for the resource management of all MIO pins. This
//! also allows associating the pins, their modes and their IDs to the peripherals they are able to
//! serve.
use arbitrary_int::{u2, u3};
use zynq7000::gpio::MmioGpio;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct MuxConfig {
    l3: u3,
    l2: u2,
    l1: bool,
    l0: bool,
}

impl From<zynq7000::slcr::mio::Config> for MuxConfig {
    fn from(value: zynq7000::slcr::mio::Config) -> Self {
        Self::new(
            value.l0_sel(),
            value.l1_sel(),
            value.l2_sel(),
            value.l3_sel(),
        )
    }
}

impl MuxConfig {
    #[inline]
    pub const fn new(l0: bool, l1: bool, l2: u2, l3: u3) -> Self {
        Self { l3, l2, l1, l0 }
    }

    #[inline]
    pub const fn new_with_l0() -> Self {
        Self::new(true, false, u2::new(0b00), u3::new(0b000))
    }

    #[inline]
    pub const fn new_with_l1() -> Self {
        Self::new(false, true, u2::new(0b00), u3::new(0b000))
    }

    #[inline]
    pub const fn new_with_l2(l2: u2) -> Self {
        Self::new(false, false, l2, u3::new(0b000))
    }

    #[inline]
    pub const fn new_with_l3(l3: u3) -> Self {
        Self::new(false, false, u2::new(0b00), l3)
    }

    #[inline]
    pub const fn new_for_gpio() -> Self {
        Self::new(false, false, u2::new(0), u3::new(0))
    }

    #[inline]
    pub const fn l0_sel(&self) -> bool {
        self.l0
    }

    #[inline]
    pub const fn l1_sel(&self) -> bool {
        self.l1
    }

    #[inline]
    pub const fn l2_sel(&self) -> u2 {
        self.l2
    }

    #[inline]
    pub const fn l3_sel(&self) -> u3 {
        self.l3
    }
}

pub trait PinId {
    const OFFSET: usize;
}

macro_rules! pin_id {
    ($Id:ident, $num:literal) => {
        // Need paste macro to use ident in doc attribute
        paste::paste! {
            #[doc = "Pin ID representing pin " $Id]
            #[derive(Debug)]
            pub enum $Id {}
            impl $crate::sealed::Sealed for $Id {}
            impl PinId for $Id {
                const OFFSET: usize = $num;
            }
        }
    };
}

pin_id!(Mio0, 0);
pin_id!(Mio1, 1);
pin_id!(Mio2, 2);
pin_id!(Mio3, 3);
pin_id!(Mio4, 4);
pin_id!(Mio5, 5);
pin_id!(Mio6, 6);
pin_id!(Mio7, 7);
pin_id!(Mio8, 8);
pin_id!(Mio9, 9);
pin_id!(Mio10, 10);
pin_id!(Mio11, 11);
pin_id!(Mio12, 12);
pin_id!(Mio13, 13);
pin_id!(Mio14, 14);
pin_id!(Mio15, 15);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio16, 16);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio17, 17);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio18, 18);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio19, 19);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio20, 20);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio21, 21);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio22, 22);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio23, 23);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio24, 24);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio25, 25);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio26, 26);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio27, 27);
pin_id!(Mio28, 28);
pin_id!(Mio29, 29);
pin_id!(Mio30, 30);
pin_id!(Mio31, 31);

pin_id!(Mio32, 32);
pin_id!(Mio33, 33);
pin_id!(Mio34, 34);
pin_id!(Mio35, 35);
pin_id!(Mio36, 36);
pin_id!(Mio37, 37);
pin_id!(Mio38, 38);
pin_id!(Mio39, 39);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio40, 40);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio41, 41);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio42, 42);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio43, 43);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio44, 44);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio45, 45);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio46, 46);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio47, 47);
pin_id!(Mio48, 48);
pin_id!(Mio49, 49);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio50, 50);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
pin_id!(Mio51, 51);
pin_id!(Mio52, 52);
pin_id!(Mio53, 53);

pub trait MioPin: crate::sealed::Sealed {
    fn offset(&self) -> usize;
}

pub struct Pin<I: PinId> {
    phantom: core::marker::PhantomData<I>,
}

impl<I: PinId> Pin<I> {
    #[inline]
    const unsafe fn new() -> Self {
        Self {
            //pin: LowLevelPin::new(I::OFFSET),
            phantom: core::marker::PhantomData,
        }
    }

    /// Steal a typed MIO pin.
    ///
    /// Usually, you can just use the MIO pin members of the [Pins] structure.
    /// However, if you pass the pins into a consuming peripheral driver which performs
    /// immediate type erasure, and you require the pins for/after a re-configuration
    /// of the system, you can unsafely steal the pin. This function will NOT perform any
    /// re-configuration.
    ///
    /// # Safety
    ///
    /// This allows to create multiple instances of the same pin, which can lead to
    /// data races on concurrent access.
    #[inline]
    pub const unsafe fn steal() -> Self {
        unsafe { Self::new() }
    }
}

pub struct Pins {
    pub mio0: Pin<Mio0>,
    pub mio1: Pin<Mio1>,
    pub mio2: Pin<Mio2>,
    pub mio3: Pin<Mio3>,
    pub mio4: Pin<Mio4>,
    pub mio5: Pin<Mio5>,
    pub mio6: Pin<Mio6>,
    pub mio7: Pin<Mio7>,
    pub mio8: Pin<Mio8>,
    pub mio9: Pin<Mio9>,
    pub mio10: Pin<Mio10>,
    pub mio11: Pin<Mio11>,
    pub mio12: Pin<Mio12>,
    pub mio13: Pin<Mio13>,
    pub mio14: Pin<Mio14>,
    pub mio15: Pin<Mio15>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio16: Pin<Mio16>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio17: Pin<Mio17>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio18: Pin<Mio18>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio19: Pin<Mio19>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio20: Pin<Mio20>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio21: Pin<Mio21>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio22: Pin<Mio22>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio23: Pin<Mio23>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio24: Pin<Mio24>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio25: Pin<Mio25>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio26: Pin<Mio26>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio27: Pin<Mio27>,
    pub mio28: Pin<Mio28>,
    pub mio29: Pin<Mio29>,
    pub mio30: Pin<Mio30>,
    pub mio31: Pin<Mio31>,

    pub mio32: Pin<Mio32>,
    pub mio33: Pin<Mio33>,
    pub mio34: Pin<Mio34>,
    pub mio35: Pin<Mio35>,
    pub mio36: Pin<Mio36>,
    pub mio37: Pin<Mio37>,
    pub mio38: Pin<Mio38>,
    pub mio39: Pin<Mio39>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio40: Pin<Mio40>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio41: Pin<Mio41>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio42: Pin<Mio42>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio43: Pin<Mio43>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio44: Pin<Mio44>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio45: Pin<Mio45>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio46: Pin<Mio46>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio47: Pin<Mio47>,
    pub mio48: Pin<Mio48>,
    pub mio49: Pin<Mio49>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio50: Pin<Mio50>,
    #[cfg(not(feature = "7z010-7z007s-clg225"))]
    pub mio51: Pin<Mio51>,
    pub mio52: Pin<Mio52>,
    pub mio53: Pin<Mio53>,
}

impl Pins {
    pub const fn new(_mmio: MmioGpio) -> Self {
        Self {
            mio0: unsafe { Pin::new() },
            mio1: unsafe { Pin::new() },
            mio2: unsafe { Pin::new() },
            mio3: unsafe { Pin::new() },
            mio4: unsafe { Pin::new() },
            mio5: unsafe { Pin::new() },
            mio6: unsafe { Pin::new() },
            mio7: unsafe { Pin::new() },
            mio8: unsafe { Pin::new() },
            mio9: unsafe { Pin::new() },
            mio10: unsafe { Pin::new() },
            mio11: unsafe { Pin::new() },
            mio12: unsafe { Pin::new() },
            mio13: unsafe { Pin::new() },
            mio14: unsafe { Pin::new() },
            mio15: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio16: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio17: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio18: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio19: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio20: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio21: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio22: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio23: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio24: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio25: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio26: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio27: unsafe { Pin::new() },
            mio28: unsafe { Pin::new() },
            mio29: unsafe { Pin::new() },
            mio30: unsafe { Pin::new() },
            mio31: unsafe { Pin::new() },

            mio32: unsafe { Pin::new() },
            mio33: unsafe { Pin::new() },
            mio34: unsafe { Pin::new() },
            mio35: unsafe { Pin::new() },
            mio36: unsafe { Pin::new() },
            mio37: unsafe { Pin::new() },
            mio38: unsafe { Pin::new() },
            mio39: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio40: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio41: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio42: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio43: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio44: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio45: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio46: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio47: unsafe { Pin::new() },
            mio48: unsafe { Pin::new() },
            mio49: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio50: unsafe { Pin::new() },
            #[cfg(not(feature = "7z010-7z007s-clg225"))]
            mio51: unsafe { Pin::new() },
            mio52: unsafe { Pin::new() },
            mio53: unsafe { Pin::new() },
        }
    }
}

impl<I: PinId> MioPin for Pin<I> {
    fn offset(&self) -> usize {
        I::OFFSET
    }
}

impl<I: PinId> crate::sealed::Sealed for Pin<I> {}
