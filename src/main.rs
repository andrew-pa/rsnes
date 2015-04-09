

struct Memory {
    main_mem : [u8; 0xFF]
}

impl Memory {

    fn read(&self, adr : u16) -> u8 {
        self.main_mem[adr as usize]
    }

    fn write(&mut self, adr : u16, val : u8) {
        self.main_mem[adr as usize] = val;
    }
}

struct CPU {
    pc : u16,
    sp : u8,
    reg_a : u8,
    reg_x : u8,
    reg_y : u8,

    flg_carry :bool,
    flg_zero : bool,
    flg_interrupt : bool,
    flg_decimal : bool,
    flg_break : bool,
    flg_ovrflow : bool,
    flg_negative : bool,
    mem :  Memory,
}

impl CPU {
    fn new(pc : u16, mm : Memory) -> CPU {
        CPU {
            pc: pc,
            sp: 0,
            reg_a: 0,
            reg_x: 0,
            reg_y: 0,

            flg_carry: false,
            flg_zero: false,
            flg_interrupt: false,
            flg_decimal: false,
            flg_break: false,
            flg_ovrflow: false,
            flg_negative: false,
            mem: mm
        }
    }

    fn next_program_byte(&mut self) -> u8 { let x = self.mem.read(self.pc); self.pc += 1; x  }

    fn execute_ORA(&mut self, v:u8) {
        self.reg_a = self.reg_a | v;
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = (self.reg_a & 0x80) == 0x80;
    }
    fn execute_AND(&mut self, v:u8) {
        self.reg_a = self.reg_a & v;
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = (self.reg_a & 0x80) == 0x80;
    }
    fn execute_EOR(&mut self, v:u8) {
        self.reg_a = self.reg_a ^ v;
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = (self.reg_a & 0x80) == 0x80;
    }
    fn execute_ADC(&mut self, v:u8) {
        let val = self.reg_a as i16 + v as i16 + if self.flg_carry { 1 } else { 0 };
        self.flg_carry = val > 0xff;
        self.reg_a = val as u8;
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = (self.reg_a & 0x80) == 0x80;
    }
    fn execute_SBC(&mut self, v:u8) {
        let val = self.reg_a as i16 - v as i16 - if self.flg_carry { 0 } else { 1 };
        self.flg_carry = val > 0xff;
        self.reg_a = val as u8;
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = (self.reg_a & 0x80) == 0x80;
    }
    fn execute_STA(&mut self, addr:u16) {
        self.mem.write(addr, self.reg_a);
    }
    fn execute_LDA(&mut self, v:u8) {
        self.reg_a = v;
    }
    fn execute_CMP(&mut self, v:u8) {
        self.flg_carry = self.reg_a >= v;
        self.flg_zero = self.reg_a == v;
        self.flg_negative = self.reg_a < v;
    }
    fn execute_ASL(&mut self, addr:u16) {
        let val = self.mem.read(addr);
        self.flg_carry = (val & 0x80) == 0x80;
        let fvl = (val * 2) | 1;
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.mem.write(addr, fvl);
    }
    fn execute_ROL(&mut self, addr:u16) {
        let val = self.mem.read(addr);
        self.flg_carry = (val & 0x80) == 0x80;
        let fvl = (val<<1) | (if self.flg_carry { 1 } else { 0 });
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.flg_zero = self.reg_a == 0;
        self.mem.write(addr, fvl);
    }
    fn execute_LSR(&mut self, addr:u16) {
        let val = self.mem.read(addr);
        self.flg_carry = (val & 0x01) == 0x01;
        let fvl = (val / 2) | 0x80;
        self.flg_zero = fvl == 0;
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.mem.write(addr, fvl);
    }
    fn execute_ROR(&mut self, addr:u16) {
        let val = self.mem.read(addr);
        self.flg_carry = (val & 0x01) == 0x01;
        let fvl = (val>>1) | (if self.flg_carry { 0x80 } else { 0 });
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.flg_zero = self.reg_a == 0;
        self.mem.write(addr, fvl);
    }
    fn execute_ASL_A(&mut self) {
        let val = self.reg_a;
        self.flg_carry = (val & 0x80) == 0x80;
        let fvl = (val * 2) | 1;
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.reg_a = fvl;
    }
    fn execute_ROL_A(&mut self) {
        let val = self.reg_a;
        self.flg_carry = (val & 0x80) == 0x80;
        let fvl = (val<<1) | (if self.flg_carry { 1 } else { 0 });
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.flg_zero = self.reg_a == 0;
        self.reg_a = fvl;
    }
    fn execute_LSR_A(&mut self) {
        let val = self.reg_a;
        self.flg_carry = (val & 0x01) == 0x01;
        let fvl = (val / 2) | 0x80;
        self.flg_zero = fvl == 0;
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.reg_a = fvl;
    }
    fn execute_ROR_A(&mut self) {
        let val = self.reg_a;
        self.flg_carry = (val & 0x01) == 0x01;
        let fvl = (val>>1) | (if self.flg_carry { 0x80 } else { 0 });
        self.flg_negative = (fvl & 0x80) == 0x80;
        self.flg_zero = self.reg_a == 0;
        self.reg_a = fvl;
    }
    fn execute_STX(&mut self, addr:u16) {
        self.mem.write(addr, self.reg_x);
    }
    fn execute_LDX(&mut self, v:u8) {
        self.reg_x = v;
    }
    fn execute_DEC(&mut self, addr:u16) {
        let v = (self.mem.read(addr)as i8)-1;
        self.flg_zero = (v == 0);
        self.flg_negative = (v&0x80) == 0x80;
        self.mem.write(addr, v as u8);
    }
    fn execute_INC(&mut self, addr:u16) {
        let v = (self.mem.read(addr)as i8)+1;
        self.flg_zero = (v == 0);
        self.flg_negative = (v&0x80) == 0x80;
        self.mem.write(addr, v as u8);
    }



    //address_* fns read correct # of bytes in program and
    //  return the ADDRESS of memory referenced! does not read/write memory for the op

    fn address_absolute(&mut self) -> u16 {
        let lo = self.next_program_byte() as u16;
        let hi = self.next_program_byte() as u16;
        (hi << 8) | lo
    }
    fn address_absoluteX(&mut self) -> u16 {
        let lo = self.next_program_byte() as u16;
        let hi = self.next_program_byte() as u16;
        ((hi << 8) | lo) + self.reg_x as u16
    }
    fn address_absoluteY(&mut self) -> u16 {
        let lo = self.next_program_byte() as u16;
        let hi = self.next_program_byte() as u16;
        ((hi << 8) | lo) + self.reg_y as u16
    }
    fn address_zeropage(&mut self) -> u16 {
        self.next_program_byte() as u16
    }
    fn address_zeropageX(&mut self) -> u16 {
        (self.next_program_byte()+self.reg_x) as u16
    }
    fn address_zeropageY(&mut self) -> u16 {
        (self.next_program_byte()+self.reg_y) as u16
    }
    fn address_indirect(&mut self) -> u16 {
        let lo = self.next_program_byte() as u16;
        let hi = self.next_program_byte() as u16;
        let addr = (hi << 8) | lo;
        self.mem.read(addr)as u16 | (self.mem.read(addr+1)<<8)as u16
    }
    fn address_idxidr(&mut self) -> u16 {
        let lo = self.next_program_byte() as u16;
        let hi = self.next_program_byte() as u16;
        let addr = (self.next_program_byte() + self.reg_x) as u16;
        self.mem.read(addr)as u16 | (self.mem.read(addr+1)<<8)as u16
    }
    fn address_idridx(&mut self) -> u16 {
        self.address_indirect() + self.reg_y as u16
    }


    fn execute_instr_type0(&mut self, opc:u8, adrm:u8) {
        //immediate addressing
        if adrm == 0x2 {
            let v = self.next_program_byte();
            match opc {
                0x0 => self.execute_ORA(v),
                0x1 => self.execute_AND(v),
                0x2 => self.execute_EOR(v),
                0x3 => self.execute_ADC(v),
                0x4 => panic!("invalid opcode"),
                0x5 => self.execute_LDA(v),
                0x6 => self.execute_CMP(v),
                0x7 => self.execute_SBC(v),
            };
        } else {
            let addr = match adrm {
                0x0 => self.address_idxidr(),
                0x1 => self.address_zeropage(),
                0x3 => self.address_absolute(),
                0x4 => self.address_idridx(),
                0x5 => self.address_zeropageX(),
                0x6 => self.address_absoluteY(),
                0x7 => self.address_absoluteX(),
            };
            match opc {
                0x0 => self.execute_ORA(self.mem.read(addr)),
                0x1 => self.execute_AND(self.mem.read(addr)),
                0x2 => self.execute_EOR(self.mem.read(addr)),
                0x3 => self.execute_ADC(self.mem.read(addr)),
                0x4 => self.execute_STA(addr),
                0x5 => self.execute_LDA(self.mem.read(addr)),
                0x6 => self.execute_CMP(self.mem.read(addr)),
                0x7 => self.execute_SBC(self.mem.read(addr)),
            };
        };
    }

    fn execute_instr_type1(&mut self, opc:u8, adrm:u8) {
        if adrm == 0x0 && opc == 0x5 { //LDX immediate
            self.execute_LDX(self.next_program_byte());
        } else if adrm == 0x2 {
            match opc {
                0x0 => self.execute_ASL_A(),
                0x1 => self.execute_ROL_A(),
                0x2 => self.execute_LSR_A(),
                0x3 => self.execute_ROR_A(),
            }
        } else {
            let addr = match adrm {
                0x1 => self.address_zeropage(),
                0x3 => self.address_absolute(),
                0x5 => self.address_zeropageX(),
                0x6 => self.address_absoluteX(),
            };
            match opc {
                0x0 => self.execute_ASL(addr),
                0x1 => self.execute_ROL(addr),
                0x2 => self.execute_LSR(addr),
                0x3 => self.execute_ROR(addr),
                0x4 => self.execute_STX(addr),
                0x5 => self.execute_LDX(self.mem.read(addr)),
                0x6 => self.execute_DEC(addr),
                0x7 => self.execute_INC(addr),
            };
        }
    }

    fn execute_instr_type2(&mut self, opc:u8, adrm:u8) {
        match (opc, adrm) {
            (_,   0x4) => self.execute_BRANCH(opc),
            (0x1, 0x1) => self.execute_BIT(self.address_zeropage()),                //BIT
            (0x1, 0x3) => self.execute_BIT(self.address_absolute()),
            (0x2, 0x3) => self.execute_JMP(self.address_absolute()),                //JMP
            (0x3, 0x3) => self.execute_JMPABS(self.address_absolute()),             //JMP-ABS
            (0x4, 0x1) => self.execute_STY(self.address_zeropage()),                //STY
            (0x4, 0x3) => self.execute_STY(self.address_absolute()),
            (0x4, 0x5) => self.execute_STY(self.address_zeropageX()),
            (0x5, 0x0) => self.execute_LDY(self.next_program_byte()),               //LDA
            (0x5, 0x1) => self.execute_LDY(self.mem.read(self.address_zeropage())),
            (0x5, 0x3) => self.execute_LDY(self.mem.read(self.address_absolute())),
            (0x5, 0x5) => self.execute_LDY(self.mem.read(self.address_zeropageX())),
            (0x5, 0x7) => self.execute_LDY(self.mem.read(self.address_absoluteX())),
            (0x6, 0x0) => self.execute_CPY(self.next_program_byte()),               //CPY
            (0x6, 0x1) => self.execute_CPY(self.mem.read(self.address_zeropage())),
            (0x6, 0x3) => self.execute_CPY(self.mem.read(self.address_absolute())),
            (0x7, 0x0) => self.execute_CPX(self.next_program_byte()),               //CPX
            (0x7, 0x1) => self.execute_CPX(self.mem.read(self.address_zeropage())),
            (0x7, 0x3) => self.execute_CPX(self.mem.read(self.address_absolute())),
        }
    }

    fn dispatch_instr(&mut self) {
        let istr = self.mem.read(self.pc);
        self.pc+=1;
        let opc = (istr&0xE0) >> 5;
        let adm = (istr&0x1C) >> 2;
        match istr {
            0x00 => self.execute_BRK(),
            0x20 => self.execute_JSR(self.address_absolute()),
            0x40 => self.execute_RTI(),
            0x60 => self.execute_RTS(),

            0x08 => self.execute_PHP(),
            0x28 => self.execute_PLP(),
            0x48 => self.execute_PHA(),
            0x68 => self.execute_PLA(),
            0x88 => self.execute_DEY(),
            0xA8 => self.execute_T('a', 'y'),
            0xC8 => self.execute_INY(),
            0xE8 => self.execute_INX(),
            0x18 => self.execute_CLC(),
            0x38 => self.execute_SEC(),
            0x58 => self.execute_CLI(),
            0x78 => self.execute_SEI(),
            0x98 => self.execute_T('y', 'a'),
            0xB8 => self.execute_CLV(),
            0xD8 => self.execute_CLD(),
            0xF8 => self.execute_SED(),
            0x8A => self.execute_T('x', 'a'),
            0x9A => self.execute_T('x', 's'),
            0xAA => self.execute_T('a', 'x'),
            0xBA => self.execute_T('s', 'x'),
            0xCA => self.execute_DEX(),
            0xEA => self.execute_NOP(),
        };
        match istr & 0x03 {
            0x01 => self.execute_instr_type0(opc, adm),
            0x02 => self.execute_instr_type1(opc, adm),
            0x00 => self.execute_instr_type2(opc, adm),
            0x03 => panic!("invalid opcode"),
        };
    }
}


fn main() {
    let mut c = CPU::new(0, Memory{main_mem: [0; 0xff]});
    c.mem.main_mem[0] = 0xa9;
    c.mem.main_mem[1] = 0x0f;
    c.mem.main_mem[2] = 0x29;
    c.mem.main_mem[3] = 0x05;

    c.dispatch_instr();
}
