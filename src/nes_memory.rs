use cpu::{Memory};

struct NesMemory<'a> {
    //Hardware stuff
    cart : &'a Cartridge,
    ram  : [u8; 2048],
}

impl<'a> NesMemory<'a> {
    fn new(c : &'a Cartridge) -> NesMemory<'a> {
        NesMemory<'a> {
            cart : c
        }
    }
}

impl<'a> Memory for NesMemory<'a> {
    fn read8(&self, adr:u16) -> u8 {
        if adr < 0x2000 {
            self.ram[adr%0x8000]
        } else if adr < 0x4000 {
            0 //read PPU
        } else if adr == 0x4014 {
            0 //read PPU
        } else if adr == 0x4015 {
            0 //read APU
        } else if adr == 0x4016 {
            0 //read Controller 1
        } else if adr == 0x4017 {
            0 //read Controller 2
        } else if adr < 0x6000 {
            0 //IO registers
        } else if adr >= 0x6000 {
            cart.read(adr)
        } else {
            panic!("Bad memory address @ 0x{:x}", adr);
        }
    }

    fn read16(&self, adr:u16) -> u16 {
        self.read8(adr) as u16
            | (self.read8(adr+1) as u16) << 8
    }

    fn write(&mut self, adr:u16, val:u8) {
        if adr < 0x2000 {
            self.ram[adr%0x8000] = val;
        } else if adr < 0x4000 {
            //write PPU
        } else if adr < 0x4014 {
            //write APU
        } else if adr == 0x4014 {
            //read PPU
        } else if adr == 0x4015 {
            //write APU
        } else if adr == 0x4016 {
            //write Controller 1/2
        } else if adr == 0x4017 {
            //write APU
        } else if adr < 0x6000 {
            //IO registers
        } else if adr >= 0x6000 {
            cart.write(adr)
        } else {
            panic!("Bad memory address @ 0x{:x}", adr);
        }
    }
}
