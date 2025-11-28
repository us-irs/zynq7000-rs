//! # Time units
#![deny(missing_docs)]

// Frequency based

/// Hertz
pub type Hertz = fugit::HertzU32;
/// Type alias for Hertz.
pub type Hz = Hertz;

/// KiloHertz
pub type KiloHertz = fugit::KilohertzU32;
/// Type alias for Kilo Hertz.
pub type KHz = KiloHertz;

/// MegaHertz
pub type MegaHertz = fugit::MegahertzU32;
/// Type alias for Mega Hertz.
pub type MHz = MegaHertz;

// Period based

/// Seconds
pub type Seconds = fugit::SecsDurationU32;

/// Milliseconds
pub type Milliseconds = fugit::MillisDurationU32;

/// Microseconds
pub type Microseconds = fugit::MicrosDurationU32;

/// Nanoseconds
pub type Nanoseconds = fugit::NanosDurationU32;
