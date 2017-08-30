#![feature(vec_resize_default)]
#![feature(ord_max_min)]
#[allow(unused_variables)]
#[allow(dead_code)]

mod cpu;
mod ppu;
mod cartridge;
mod nes_memory;
use cpu::*;
use ppu::*;
use cartridge::*;
use nes_memory::*;

use std::env;
use std::io::*;

fn main() {
    let mut cart = cartridge::Cartridge::new_from_ines(
        env::args().nth(1).or_else(|| env::var("ROMPATH").ok()).expect("cart path")).expect("load cart");
    let mut ppu = PictureProcessor {};
    let mut cpu = cpu::CPU::new(NesMemory::new(&mut cart, &mut ppu));
    /*cpu.memory.write(0xfffe, 0x00); //start CPU at 0x0000
    cpu.memory.write(0xffff, 0x00);
    cpu.memory.write(0x0000, 0xe8); // inx
    cpu.memory.write(0x0001, 0x4c); // jmp 0x0000
    cpu.memory.write(0x0002, 0x00);*/
    let mut ln = String::new();
    let mut count = 0;
    while count < 30 {
        let last_pc = cpu.pc;
        cpu.step();
        if cpu.pc == last_pc { count += 1; }
        cpu.write_state();
    }
}
