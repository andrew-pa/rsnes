use cpu::{Memory};
use ppu::PictureProcessor;
use cartridge::*;

use std::rc::Rc;
use std::cell::RefCell;

pub struct NesMemory<'c> {
    //Hardware stuff
    cart: &'c mut Cartridge,
    ram: [u8; 0x800],
    ppu: Rc<RefCell<PictureProcessor>>
}

impl<'c> NesMemory<'c> {
    pub fn new(c : &'c mut Cartridge, p: Rc<RefCell<PictureProcessor>>) -> NesMemory<'c> {
        NesMemory {
            cart : c, ppu: p,
            ram: [0u8; 0x800]
        }
    }
}

impl<'c> Memory for NesMemory<'c> {
    fn read8(&mut self, adr:u16) -> u8 {
        //println!("read @ 0x{:x}", adr);
        if adr <= 0x1fff {
            self.ram[(adr%0x800) as usize]
        } else if adr >= 0x2000 && adr < 0x3fff {
            self.ppu.borrow_mut().read((adr-0x2000) % 8)
        } else if adr >= 0x6000 && adr < 0x8000 {
            self.cart.sram[(adr-0x6000) as usize]
        } else if adr >= 0x8000 {
            self.cart.read(adr)
        } else { println!("read {:x}", adr); 0 }
    }

    fn read16(&mut self, adr:u16) -> u16 {
        self.read8(adr) as u16
            | (self.read8(adr+1) as u16) << 8
    }

    fn write(&mut self, adr:u16, val:u8) {
        if adr <= 0x1fff {
            self.ram[(adr%0x800) as usize] = val;
        } else if adr >= 0x2000 && adr < 0x3fff {
            println!("ppu {:x}", adr);
            self.ppu.borrow_mut().write((adr-0x2000) % 8, val)
        } else if adr == 0x4014 {
            println!("PPU DMA @ {}", val);
        } else if adr >= 0x6000 && adr < 0x8000 {
            self.cart.sram[(adr-0x6000) as usize] = val;
        } else if adr >= 0x8000 {
            self.cart.write(adr, val)
        } else { println!("write {:x} = {:x}", adr, val); }
    }
}
