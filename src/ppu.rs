
pub struct PictureProcessor {
    name_table_addr: u16,
    addr_incr: u16,
    sprite_table: u8,
    background_table: u8,
    tall_sprites: bool,
    pub nmi_on_vblank: bool,
    color_mode: bool,
    clip_background: bool,
    clip_sprites: bool,
    show_background: bool,
    show_sprites: bool,
    color_intensity: u8,

    scanline_sprite_limit: bool,
    s0_hit_flag: bool,
    vblanking: bool,

    spr_ram: [u8; 256],
    spr_addr: u8,

    vram_addr: u16, scroll: u16
}

impl PictureProcessor {
    pub fn new() -> PictureProcessor {
        PictureProcessor {
            name_table_addr: 0x2000,
            addr_incr: 1,
            sprite_table: 0,
            background_table: 0,
            tall_sprites: false,
            nmi_on_vblank: false,
            color_mode: false,
            clip_background: false,
            clip_sprites: false,
            show_background: false,
            show_sprites: false,
            color_intensity: 0,

            scanline_sprite_limit: false,
            s0_hit_flag: false,
            vblanking: false,

            spr_ram: [0u8; 256],
            spr_addr: 0, vram_addr: 0, scroll: 0
        }
    }
    pub fn vblank(&mut self) { self.vblanking = true; }
    pub fn read(&mut self, addr: u16) -> u8 {
        println!("ppu read @ ${:x}", addr);
        match addr {
            2 => {
                let a = if self.vblanking || !self.show_background || !self.show_sprites { 0x10 } else { 0 };
                let b = if self.scanline_sprite_limit { 0x20 } else { 0 };
                let c = if self.s0_hit_flag { 0x40 } else { 0 };
                let d = if self.vblanking { 0x80 } else { 0 };
                self.vblanking = false;
                self.vram_addr = 0;
                a|b|c|d
            },
            7 => {
                println!("read vram @ {:x}", addr);
                0
            },
            _ => 0
        }
    }
    pub fn write(&mut self, addr: u16, val: u8) {
        match addr {
            0 => {
                self.name_table_addr = 0x2000 + 0x400 * ((val & 0x03) >> 1) as u16;
                self.addr_incr = if (val & 0x04) == 0x04 { 32 } else { 1 };
                self.sprite_table = if (val & 0x08) == 0x08 { 0 } else { 0x1000 };
                self.background_table = if (val & 0x10) == 0x10 { 0 } else { 0x1000 };
                self.tall_sprites = (val & 0x20) == 0x20;
                self.nmi_on_vblank = (val & 0x80) == 0x80;
            },
            1 => {
                self.color_mode = (val & 0x01) == 0x01;
                self.clip_background = (val & 0x02) == 0x02;
                self.clip_sprites = (val & 0x04) == 0x04;
                self.show_background = (val & 0x08) == 0x08;
                self.show_sprites = (val & 0x10) == 0x10;
                self.color_intensity = (val & 0xe0) >> 1;
            },
            3 => {
                self.spr_addr = val;
            },
            4 => {
                self.spr_ram[self.spr_addr as usize] = val;
            },
            5 => {
                self.scroll = (self.scroll << 1) | (val as u16);
            },
            6 => {
                self.vram_addr = (self.vram_addr << 1) | (val as u16);
            },
            7 => {
                println!("write vram @ {:x} = {:x}", addr, val);
                
                self.vram_addr += self.addr_incr;
            }
            _ => {},
        }
        println!("ppu write ${:x} = {:x}", addr, val);
    }
    pub fn dma(&mut self, data: Vec<u8>) {
        println!("spr dma {:?}", data);
        self.spr_ram.copy_from_slice(&data);
    }
}
