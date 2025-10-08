// The constants here are also checked/tested at compile time against table 12-3 in the TRM
// p.368.
use arbitrary_int::u3;
use zynq7000::qspi::{InstructionCode, LinearQspiConfig};

pub const RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(false)
    .with_separate_memory_bus(false)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x0))
    .with_instruction_code(InstructionCode::Read)
    .build();
const RD_ONE_DEV_RAW: u32 = RD_ONE.raw_value();
static_assertions::const_assert_eq!(RD_ONE_DEV_RAW, 0x8000_0003);

pub const RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(true)
    .with_separate_memory_bus(true)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x0))
    .with_instruction_code(InstructionCode::Read)
    .build();
const RD_TWO_RAW: u32 = RD_TWO.raw_value();
static_assertions::const_assert_eq!(RD_TWO_RAW, 0xE000_0003);

pub const FAST_RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(false)
    .with_separate_memory_bus(false)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x1))
    .with_instruction_code(InstructionCode::FastRead)
    .build();
const FAST_RD_ONE_RAW: u32 = FAST_RD_ONE.raw_value();
static_assertions::const_assert_eq!(FAST_RD_ONE_RAW, 0x8000_010B);

pub const FAST_RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(true)
    .with_separate_memory_bus(true)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x1))
    .with_instruction_code(InstructionCode::FastRead)
    .build();
const FAST_RD_TWO_RAW: u32 = FAST_RD_TWO.raw_value();
static_assertions::const_assert_eq!(FAST_RD_TWO_RAW, 0xE000_010B);

pub const DUAL_OUT_FAST_RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(false)
    .with_separate_memory_bus(false)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x1))
    .with_instruction_code(InstructionCode::FastReadDualOutput)
    .build();
const DUAL_OUT_FAST_RD_ONE_RAW: u32 = DUAL_OUT_FAST_RD_ONE.raw_value();
static_assertions::const_assert_eq!(DUAL_OUT_FAST_RD_ONE_RAW, 0x8000_013B);

pub const DUAL_OUT_FAST_RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(true)
    .with_separate_memory_bus(true)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x1))
    .with_instruction_code(InstructionCode::FastReadDualOutput)
    .build();
const DUAL_OUT_FAST_RD_TWO_RAW: u32 = DUAL_OUT_FAST_RD_TWO.raw_value();
static_assertions::const_assert_eq!(DUAL_OUT_FAST_RD_TWO_RAW, 0xE000_013B);

pub const QUAD_OUT_FAST_RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(false)
    .with_separate_memory_bus(false)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x1))
    .with_instruction_code(InstructionCode::FastReadQuadOutput)
    .build();
const QUAD_OUT_FAST_RD_ONE_RAW: u32 = QUAD_OUT_FAST_RD_ONE.raw_value();
static_assertions::const_assert_eq!(QUAD_OUT_FAST_RD_ONE_RAW, 0x8000_016B);

pub const QUAD_OUT_FAST_RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
    .with_enable_linear_mode(true)
    .with_both_memories(true)
    .with_separate_memory_bus(true)
    .with_upper_memory_page(false)
    .with_mode_enable(false)
    .with_mode_on(false)
    .with_mode_bits(0x0)
    .with_num_dummy_bytes(u3::new(0x1))
    .with_instruction_code(InstructionCode::FastReadQuadOutput)
    .build();
const QUAD_OUT_FAST_RD_TWO_RAW: u32 = QUAD_OUT_FAST_RD_TWO.raw_value();
static_assertions::const_assert_eq!(QUAD_OUT_FAST_RD_TWO_RAW, 0xE000_016B);

pub(crate) mod winbond_spansion {
    use super::*;
    pub const DUAL_IO_FAST_RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(false)
        .with_separate_memory_bus(false)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x0))
        .with_instruction_code(InstructionCode::FastReadDualIo)
        .build();
    const DUAL_IO_FAST_RD_ONE_RAW: u32 = DUAL_IO_FAST_RD_ONE.raw_value();
    static_assertions::const_assert_eq!(DUAL_IO_FAST_RD_ONE_RAW, 0x82FF_00BB);

    pub const DUAL_IO_FAST_RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(true)
        .with_separate_memory_bus(true)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x0))
        .with_instruction_code(InstructionCode::FastReadDualIo)
        .build();
    const DUAL_IO_FAST_RD_TWO_RAW: u32 = DUAL_IO_FAST_RD_TWO.raw_value();
    static_assertions::const_assert_eq!(DUAL_IO_FAST_RD_TWO_RAW, 0xE2FF_00BB);

    pub const QUAD_IO_FAST_RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(false)
        .with_separate_memory_bus(false)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x2))
        .with_instruction_code(InstructionCode::FastReadQuadIo)
        .build();
    const QUAD_IO_FAST_RD_ONE_RAW: u32 = QUAD_IO_FAST_RD_ONE.raw_value();
    static_assertions::const_assert_eq!(QUAD_IO_FAST_RD_ONE_RAW, 0x82FF_02EB);

    pub const QUAD_IO_FAST_RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(true)
        .with_separate_memory_bus(true)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x2))
        .with_instruction_code(InstructionCode::FastReadQuadIo)
        .build();
    const QUAD_IO_FAST_RD_TWO_RAW: u32 = QUAD_IO_FAST_RD_TWO.raw_value();
    static_assertions::const_assert_eq!(QUAD_IO_FAST_RD_TWO_RAW, 0xE2FF_02EB);
}

pub(crate) mod micron {
    use super::*;
    pub const DUAL_IO_FAST_RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(false)
        .with_separate_memory_bus(false)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x1))
        .with_instruction_code(InstructionCode::FastReadDualIo)
        .build();
    const DUAL_IO_FAST_RD_ONE_RAW: u32 = DUAL_IO_FAST_RD_ONE.raw_value();
    static_assertions::const_assert_eq!(DUAL_IO_FAST_RD_ONE_RAW, 0x82FF_01BB);

    pub const DUAL_IO_FAST_RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(true)
        .with_separate_memory_bus(true)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x1))
        .with_instruction_code(InstructionCode::FastReadDualIo)
        .build();
    const DUAL_IO_FAST_RD_TWO_RAW: u32 = DUAL_IO_FAST_RD_TWO.raw_value();
    static_assertions::const_assert_eq!(DUAL_IO_FAST_RD_TWO_RAW, 0xE2FF_01BB);

    pub const QUAD_IO_FAST_RD_ONE: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(false)
        .with_separate_memory_bus(false)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x4))
        .with_instruction_code(InstructionCode::FastReadQuadIo)
        .build();
    const QUAD_IO_FAST_RD_ONE_RAW: u32 = QUAD_IO_FAST_RD_ONE.raw_value();
    static_assertions::const_assert_eq!(QUAD_IO_FAST_RD_ONE_RAW, 0x82FF_04EB);

    pub const QUAD_IO_FAST_RD_TWO: LinearQspiConfig = LinearQspiConfig::builder()
        .with_enable_linear_mode(true)
        .with_both_memories(true)
        .with_separate_memory_bus(true)
        .with_upper_memory_page(false)
        .with_mode_enable(true)
        .with_mode_on(false)
        .with_mode_bits(0xff)
        .with_num_dummy_bytes(u3::new(0x4))
        .with_instruction_code(InstructionCode::FastReadQuadIo)
        .build();
    const QUAD_IO_FAST_RD_TWO_RAW: u32 = QUAD_IO_FAST_RD_TWO.raw_value();
    static_assertions::const_assert_eq!(QUAD_IO_FAST_RD_TWO_RAW, 0xE2FF_04EB);
}
