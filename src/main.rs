#![feature(vec_resize_default)]
#![feature(ord_max_min)]
#[allow(unused_variables)]
#[allow(dead_code)]
extern crate sdl2;

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
use std::rc::Rc;
use std::cell::RefCell;

use sdl2::event::Event;

fn main() {
    let sdl_context = sdl2::init().expect("init SDL2");
    let sdl_video = sdl_context.video().expect("init SDL2 video");
    let window = sdl_video.window("rsnes", 800, 600).position_centered().opengl().build().expect("create window");

    let mut cart = cartridge::Cartridge::new_from_ines(
        env::args().nth(1).or_else(|| env::var("ROMPATH").ok()).expect("cart path")).expect("load cart");
    let mut ppu = Rc::new(RefCell::new(PictureProcessor::new()));
    let mut cpu = cpu::CPU::new(NesMemory::new(&mut cart, ppu.clone()));

    let mut event_pump = sdl_context.event_pump().expect("obtain event pump");
    'frameloop: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} => { break 'frameloop },
                _ => {}
            }
        }

        let mut y = 0;
        let mut cycles = 0;
        let mut rep = 0;
        while y < 263 {
            print!("scanline = {} ", y);
            cpu.write_state();
            let _pc = cpu.pc;
            cycles += cpu.step();
            if _pc == cpu.pc { rep += 1; if rep > 40 { break 'frameloop } }
            if cycles > 113 {
                for _ in 0..(cycles%113) { // advance scanlines for time on cpu
                    y += 1; //next scanline
                    if y == 240 { // v-blank
                        println!("v blank!");
                        let mut _ppu = ppu.borrow_mut();
                        _ppu.vblank();
                        if _ppu.nmi_on_vblank {
                            cpu.trigger_nmi();
                        }
                    }
                }
            }
        }
    } //frameloop
}
