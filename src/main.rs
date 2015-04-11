#![feature(collections)]

mod cpu;
mod cartridge;

fn main() {
    let mut cart = cartridge::Cartridge::new_from_ines("C:\\Users\\andre_000\\Source\\rsnes\\test_roms\\TETRIS.NES");
}
