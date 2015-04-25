#![feature(collections)]

mod cpu;
use cpu::{Memory};
mod cartridge;

use std::io::*;

fn main() {
    //let mut cart = cartridge::Cartridge::new_from_ines("C:\\Users\\andre_000\\Source\\rsnes\\test_roms\\TETRIS.NES");
    let mut cpu = cpu::CPU::new(cpu::FlatMemory::new());
    cpu.memory.write(0xfffe, 0x00);
    cpu.memory.write(0xffff, 0x00);
    cpu.memory.write(0x0000, 0xe8);
    cpu.memory.write(0x0001, 0x6c);
    cpu.memory.write(0x0002, 0x00);
    cpu.memory.write(0x0003, 0x00);
    let mut ln = String::new();
    loop {
        cpu.step();
        cpu.write_state();
        stdin().read_line(&mut ln);
    }
}
