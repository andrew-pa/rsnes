extern crate byteorder;
use cpu;
use std::fs::File;
use std::mem;
use std::io::{Result, Read, Cursor, Seek, SeekFrom};
use self::byteorder::{LittleEndian, ReadBytesExt};

pub struct Cartridge {
    PRG : Vec<u8>,
    CHR : Vec<u8>,
    SRAM : [u8; 0x2001],
    map_type : u8,
    mirror_md : u8,
    battery : u8,
}

struct iNESFileHeader {
    magic : u32,
    numPRG : u8,
    numCHR : u8,
    cntrl1 : u8,
    cntrl2 : u8,
    numRAM : u8,
}

impl Cartridge {
    fn read_header(bmp_data : &mut Cursor<Vec<u8>>) -> Result<iNESFileHeader> {
        let hdr = iNESFileHeader {
            magic:      try!(bmp_data.read_u32::<LittleEndian>()),
            numPRG:     try!(bmp_data.read_u8()),
            numCHR:     try!(bmp_data.read_u8()),
            cntrl1:     try!(bmp_data.read_u8()),
            cntrl2:     try!(bmp_data.read_u8()),
            numRAM:     try!(bmp_data.read_u8()),
        };
        Ok(hdr)
    }
    pub fn new_from_ines(path : &str) -> Result<Cartridge> {
        let mut byts = Vec::new();
        let mut f = try!(File::open(path));
        try!(f.read_to_end(&mut byts));
        let mut data = Cursor::new(byts);
        let hdr = try!(Cartridge::read_header(&mut data));
        if hdr.magic != 0x1a53454e {
            panic!("Header magic is bad");
        }

        let mapper = (hdr.cntrl1 >> 4) | hdr.cntrl2;
        let mirror = (hdr.cntrl1 & 1) | (((hdr.cntrl1 >> 3) & 1) << 1);
        let battery = (hdr.cntrl1 >> 1) & 1;
        if hdr.cntrl1 & 4 == 4 { //nobody cares about trainer data
            try!(data.seek(SeekFrom::Current(512)));
        }

        let mut PRGbuf : Vec<u8> = Vec::new();
        PRGbuf.resize((hdr.numPRG*16384) as usize, 0);
        try!(data.read(&mut PRGbuf[0..]));
        let mut CHRbuf : Vec<u8> = Vec::new();
        CHRbuf.resize((hdr.numCHR*8192) as usize, 0);
        try!(data.read(&mut CHRbuf[0..]));

        Ok(Cartridge{
            PRG:        PRGbuf,
            CHR:        CHRbuf,
            SRAM:       [0; 0x2001],
            map_type:   mapper,
            mirror_md:  mirror,
            battery:    battery,
        })
    }
    pub fn read(&self, adr : u16) -> u8 {
        0
    }
    pub fn write(&mut self, adr : u16, val : u8) {
    }
}
