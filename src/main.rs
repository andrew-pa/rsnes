mod cpu;
mod cartridge;

fn main() {
    let mut cart = cartridge::Cartridge::new_from_ines("C:\\Users\\andre_000\\Desktop\\Apps\\Rust\\source\\rsnes\\test_roms\\TETRIS.NES");

    let mut c = cpu::CPU6502::new(0, cpu::FlatMemory{main_mem: [0; 65536]});
    c.mem.main_mem[0] = 0xC6;
    c.mem.main_mem[1] = 0x0f;
    c.mem.main_mem[2] = 0x4C;
    c.mem.main_mem[3] = 0x00;
    c.mem.main_mem[4] = 0x00;

    loop
    {
        c.dispatch_instr();
    }
}
