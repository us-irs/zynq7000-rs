use crate::gpio::mio::{
    Mio10, Mio11, Mio12, Mio13, Mio14, Mio15, Mio28, Mio29, Mio30, Mio31, Mio32, Mio33, Mio34,
    Mio35, Mio36, Mio37, Mio38, Mio39, Mio48, Mio49, MioPin, Pin,
};
#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27, Mio40,
    Mio41, Mio42, Mio43, Mio44, Mio45, Mio46, Mio47, Mio50, Mio51,
};

pub trait Sdio0ClockPin: MioPin {}
pub trait Sdio0CommandPin: MioPin {}
pub trait Sdio0Data0Pin: MioPin {}
pub trait Sdio0Data1Pin: MioPin {}
pub trait Sdio0Data2Pin: MioPin {}
pub trait Sdio0Data3Pin: MioPin {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0ClockPin for Pin<Mio16> {}
impl Sdio0ClockPin for Pin<Mio28> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0ClockPin for Pin<Mio40> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0CommandPin for Pin<Mio17> {}
impl Sdio0CommandPin for Pin<Mio29> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0CommandPin for Pin<Mio41> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data0Pin for Pin<Mio18> {}
impl Sdio0Data0Pin for Pin<Mio30> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data0Pin for Pin<Mio42> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data1Pin for Pin<Mio19> {}
impl Sdio0Data1Pin for Pin<Mio31> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data1Pin for Pin<Mio43> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data2Pin for Pin<Mio20> {}
impl Sdio0Data2Pin for Pin<Mio32> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data2Pin for Pin<Mio44> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data3Pin for Pin<Mio21> {}
impl Sdio0Data3Pin for Pin<Mio33> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data3Pin for Pin<Mio45> {}

pub trait Sdio1ClockPin: MioPin {}
pub trait Sdio1CommandPin: MioPin {}
pub trait Sdio1Data0Pin: MioPin {}
pub trait Sdio1Data1Pin: MioPin {}
pub trait Sdio1Data2Pin: MioPin {}
pub trait Sdio1Data3Pin: MioPin {}

impl Sdio1ClockPin for Pin<Mio12> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1ClockPin for Pin<Mio24> {}
impl Sdio1ClockPin for Pin<Mio36> {}
impl Sdio1ClockPin for Pin<Mio48> {}

impl Sdio1CommandPin for Pin<Mio11> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1CommandPin for Pin<Mio23> {}
impl Sdio1CommandPin for Pin<Mio35> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1CommandPin for Pin<Mio47> {}

impl Sdio1Data0Pin for Pin<Mio10> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data0Pin for Pin<Mio22> {}
impl Sdio1Data0Pin for Pin<Mio34> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data0Pin for Pin<Mio46> {}

impl Sdio1Data1Pin for Pin<Mio13> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data1Pin for Pin<Mio25> {}
impl Sdio1Data1Pin for Pin<Mio37> {}
impl Sdio1Data1Pin for Pin<Mio49> {}

impl Sdio1Data2Pin for Pin<Mio14> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data2Pin for Pin<Mio26> {}
impl Sdio1Data2Pin for Pin<Mio38> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data2Pin for Pin<Mio50> {}

impl Sdio1Data2Pin for Pin<Mio15> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data3Pin for Pin<Mio27> {}
impl Sdio1Data3Pin for Pin<Mio39> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data3Pin for Pin<Mio51> {}
