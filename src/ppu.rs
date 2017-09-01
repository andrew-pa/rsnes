
pub struct PictureProcessor {
}

impl PictureProcessor {
    pub fn read(&self, addr: u16) -> u8 { println!("ppu read @ ${:x}", addr); 0 }
    pub fn write(&self, addr: u16, val: u8) { println!("ppu write ${:x} = {:x}", addr, val); }
}
