//! # Time units

// Frequency based

/// Hertz
pub type Hertz = fugit::HertzU32;
pub type Hz = Hertz;

/// KiloHertz
pub type KiloHertz = fugit::KilohertzU32;
pub type KHz = KiloHertz;

/// MegaHertz
pub type MegaHertz = fugit::MegahertzU32;
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
