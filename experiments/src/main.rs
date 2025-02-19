use zynq7000::slcr::{ClockControl, Slcr};


fn main() {
    let size = core::mem::size_of::<ClockControl>();
    println!("Size of ClockControl: {}", size);
    let size = core::mem::size_of::<Slcr>();
    println!("Size of SLCR: {}", size);

    println!("Hello, world!");
}
