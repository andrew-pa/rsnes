extern crate byteorder;
use cpu;
use std::error::Error;
use std::path::Path;
use std::fs::File;
use std::mem;
use std::io::{ Read, Cursor, Seek, SeekFrom, Error as IoError };
use self::byteorder::{LittleEndian, ReadBytesExt};

enum Mapper {
    NROM
}

pub struct Cartridge {
    data: Vec<u8>,
    pub sram: Vec<u8>,
    num_prgs: u8, num_chrs: u8,
    prg_offset: usize, chr_offset: usize,
    mapper: Mapper,
    mirror_md: u8,
    battery: u8,
}

#[derive(Debug)]
pub enum CartridgeLoadError {
    Io(IoError),
    BadHeaderMagic,
    UnknownMapper(u8)
}

use std::fmt;
use std::convert;
impl fmt::Display for CartridgeLoadError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            CartridgeLoadError::Io(ref e) => write!(f, "IO Error: {}", e),
            CartridgeLoadError::BadHeaderMagic => write!(f, "Bad Header Magic"),
            CartridgeLoadError::UnknownMapper(i) => write!(f, "Unknown mapper {:x}", i)
        }
    }
}

impl Error for CartridgeLoadError {
    fn description(&self) -> &str { "Cartridge Load Error" }
    fn cause(&self) -> Option<&Error> {
        match self {
            &CartridgeLoadError::Io(ref e) => Some(e),
            _ => None
        }
    }
}

impl From<IoError> for CartridgeLoadError {
    fn from(e: IoError) -> Self { CartridgeLoadError::Io(e) }
}



impl Cartridge {
    pub fn new_from_ines<P: AsRef<Path>>(path: P) -> Result<Cartridge, CartridgeLoadError> {
        let mut data = Vec::new();
        println!("Loading file: {}", path.as_ref().display());
        let mut f = File::open(path)?;
        f.read_to_end(&mut data)?;
        if data[3] != 0x1a && data[2] != 0x53 && data[1] != 0x45 && data[0] != 0x4e {
            return Err(CartridgeLoadError::BadHeaderMagic);
        }
        let mapper = (data[6] >> 4) | (data[7] & 0xf0);
        let mirror = (data[6] & 1) | (((data[6] >> 3) & 1) << 1);
        let battery = (data[6] >> 1) & 1;
        let trainer_present = data[6] & 0b0000_0100 == 0b0000_01000;
        println!("mapper = {}, mirror = {}, battery = {}, trainer = {}", mapper, mirror, battery, trainer_present);
        println!("num prg = {}, num_chr = {}, data.len() = {}", data[4], data[5], data.len());
        Ok(Cartridge {
            sram: { let mut v = Vec::new(); v.resize_default((data[8] as usize * 8 * 1024).max(8*1024)); v },
            num_prgs: data[4], num_chrs: data[5],
            prg_offset: 16 + (if trainer_present { 512 } else { 0 }),
            chr_offset: 16 + (if trainer_present { 512 } else { 0 }) + 16*1024*data[4] as usize,
            mapper: match mapper {
                0 => Mapper::NROM,
                _ => return Err(CartridgeLoadError::UnknownMapper(mapper))
            },
            mirror_md: mirror, battery, data
        })
    }

    pub fn read(&self, adr : u16) -> u8 {
        match self.mapper {
            Mapper::NROM => {
                if adr <= 0xbfff {
                    self.data[self.prg_offset + (adr-0x8000) as usize]
                } else if adr >= 0xc000 {
                    self.data[self.prg_offset + 16*1024 + (adr-0xc000) as usize]
                } else {
                    println!("???");
                    0
                }
            }
        }
    }

    pub fn write(&mut self, adr : u16, v : u8) {
        match self.mapper {
            Mapper::NROM => {
                println!("no ram @ {:x} (v = {})", adr, v);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::*;
    use std::fs::DirEntry;
    #[test]
    fn load_cart() {
        for rom_path in Path::new("roms\\").read_dir().expect("read roms\\ dir").map(|p| p.map(|x| x.path())) {
            match rom_path {
                Ok(p) => match Cartridge::new_from_ines(p) {
                    Ok(_) => println!("sucess!"),
                    Err(CartridgeLoadError::UnknownMapper(m)) => println!("unknown mapper {}", m),
                    Err(e) => panic!("bad cart load: {}", e)
                },
                Err(e) => println!("path error = {:?}", e)
            }
        }
    }
}
