use cpu::{Memory};
use ppu::PictureProcessor;
use cartridge::*;

pub struct NesMemory<'c, 'p> {
    //Hardware stuff
    cart: &'c mut Cartridge,
    ram: [u8; 0x800],
    ppu: &'p mut PictureProcessor
}

impl<'c, 'p> NesMemory<'c, 'p> {
    pub fn new(c : &'c mut Cartridge, p: &'p mut PictureProcessor) -> NesMemory<'c, 'p> {
        NesMemory {
            cart : c, ppu: p,
            ram: [0u8; 0x800]
        }
    }
}

impl<'c, 'p> Memory for NesMemory<'c, 'p> {
    fn read8(&self, adr:u16) -> u8 {
        //println!("read @ 0x{:x}", adr);
        if adr <= 0x1fff {
            self.ram[(adr%0x800) as usize]
        } else if adr >= 0x2000 && adr < 0x3fff {
            self.ppu.read((adr-0x2000) % 8)
        } else if adr >= 0x6000 && adr < 0x8000 {
            self.cart.sram[(adr-0x6000) as usize]
        } else if adr >= 0x8000 {
            self.cart.read(adr)
        } else { println!("read {}", adr); 0 }
    }

    fn read16(&self, adr:u16) -> u16 {
        self.read8(adr) as u16
            | (self.read8(adr+1) as u16) << 8
    }

    fn write(&mut self, adr:u16, val:u8) {
        if adr <= 0x1fff {
            self.ram[(adr%0x800) as usize] = val;
        } else if adr >= 0x2000 && adr < 0x3fff {
            self.ppu.write((adr-0x2000) % 8, val)
        } else if adr == 0x4014 {
            println!("PPU DMA @ {}", val);
        } else if adr >= 0x6000 && adr < 0x8000 {
            self.cart.sram[(adr-0x6000) as usize] = val;
        } else if adr >= 0x8000 {
            self.cart.write(adr, val)
        } else { println!("read {}", adr); }
    }
}
