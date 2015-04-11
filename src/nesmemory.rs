use cpu;
use cartridge;

struct NESMemory {
    pub RAM : [u8; 2048];
    pub cart : cartridge::Cartridge;

}

impl Memory for NESMemory {
    fn read(&self, addr:u16) -> u8 {
        if addr < 0x2000 { RAM[addr%0x0800] }
        else if(addr >= 0x6000) { cart.read(addr) }
    }

    fn write(&mut self, addr:u16, v:u8) {
        if addr < 0x2000 { RAM[addr%0x0800] = v; }
        else if addr >= 0x6000 { cart.write(addr, v); }
    }
}
