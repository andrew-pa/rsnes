pub trait Memory {
    fn read8(&self, adr:u16)->u8;
    fn read16(&self, adr:u16)->u16;
    fn write(&mut self, adr:u16, val:u8);
}

pub struct FlatMemory {
    pub main_mem : [u8; 65536]
}

impl FlatMemory {
    fn new() -> FlatMemory {
        FlatMemory { main_mem: [0; 65536] }
    }

}

impl Memory for FlatMemory {

    fn read8(&self, adr : u16) -> u8 {
        println!("reading ${:x} = #${:x}", adr, self.main_mem[adr as usize]);
        self.main_mem[adr as usize]
    }
    fn read16(&self, adr: u16) -> u16 {
        self.main_mem[adr as usize] as u16
            | (self.main_mem[adr as usize + 1] as u16) << 8
    }

    fn write(&mut self, adr : u16, val : u8) {
        println!("writing ${:x} <= #${:x}", adr, val);
        self.main_mem[adr as usize] = val;
    }
}

macro_rules! check_bit {
    ( $value:expr, $bit:expr ) => {{ (($value >> $bit) & 1)==1 }}
}


enum AddressingMode {
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Accumulator,
    Immediate,
    Implied,
    IndexedIndirect,
    Indirect,
    IndirectIndexed,
    Relative,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
}


type CPUInstr<M:Memory> = fn(&mut CPU<M>, u16,u16,AddressingMode) -> ();

enum InterruptType {
    None, NMI, IRQ
}

enum CpuFlag {
    Carry, Zero, Interrupt, Decimal, Break, Overflow, Negative
}

struct CPU<M : Memory> {
    memory: M,
    cycles: u64,

    pc: u16,
    sp: u8,
    ra: u8,
    rx: u8,
    ry: u8,
    flg: u8,

    interrupt: InterruptType,
    stall: i32,

    instr_table: Vec<CPUInstr>,
}

impl<M:Memory> CPU<M> {
    pub fn new(m : M) -> CPU {
         CPU {
            memory: m,
            cycles: 0,
            pc: m.read(0xfffc),
            sp: 0xfd,
            ra: 0, rx: 0, ry: 0,
            flg: 0x24,
            interrupt: 0,
            stall: 0,
            instr_table: vec![
                brk, ora, kil, slo, nop, ora, asl, slo,
                php, ora, asl, anc, nop, ora, asl, slo,
                bpl, ora, kil, slo, nop, ora, asl, slo,
                clc, ora, nop, slo, nop, ora, asl, slo,
                jsr, and, kil, rla, bit, and, rol, rla,
                plp, and, rol, anc, bit, and, rol, rla,
                bmi, and, kil, rla, nop, and, rol, rla,
                sec, and, nop, rla, nop, and, rol, rla,
                rti, eor, kil, sre, nop, eor, lsr, sre,
                pha, eor, lsr, alr, jmp, eor, lsr, sre,
                bvc, eor, kil, sre, nop, eor, lsr, sre,
                cli, eor, nop, sre, nop, eor, lsr, sre,
                rts, adc, kil, rra, nop, adc, ror, rra,
                pla, adc, ror, arr, jmp, adc, ror, rra,
                bvs, adc, kil, rra, nop, adc, ror, rra,
                sei, adc, nop, rra, nop, adc, ror, rra,
                nop, sta, nop, sax, sty, sta, stx, sax,
                dey, nop, txa, xaa, sty, sta, stx, sax,
                bcc, sta, kil, ahx, sty, sta, stx, sax,
                tya, sta, txs, tas, shy, sta, shx, ahx,
                ldy, lda, ldx, lax, ldy, lda, ldx, lax,
                tay, lda, tax, lax, ldy, lda, ldx, lax,
                bcs, lda, kil, lax, ldy, lda, ldx, lax,
                clv, lda, tsx, las, ldy, lda, ldx, lax,
                cpy, cmp, nop, dcp, cpy, cmp, dec, dcp,
                iny, cmp, dex, axs, cpy, cmp, dec, dcp,
                bne, cmp, kil, dcp, nop, cmp, dec, dcp,
                cld, cmp, nop, dcp, nop, cmp, dec, dcp,
                cpx, sbc, nop, isc, cpx, sbc, inc, isc,
                inx, sbc, nop, sbc, cpx, sbc, inc, isc,
                beq, sbc, kil, isc, nop, sbc, inc, isc,
                sed, sbc, nop, isc, nop, sbc, inc, isc],
        }
    }

    pub fn reset(&mut self) {
        self.pc = self.memory.read16(0xfffc);
        self.sp = 0xfd;
        self.flg = 0x24;
    }

    pub fn trigger_nmi(&mut self) {
        self.interrupt = InterruptType::NMI;
    }
    pub fn trigger_irq(&mut self) {
        if !flag(CpuFlag::Interrupt) {
            self.interrupt = InterruptType::IRQ;
        }
    }

    pub fn step(&mut self) -> u64 {
        if self.stall > 0 {
            self.stall -= 1;
            return 1;
        }

        match self.interrupt {
            NMI => self.nmi(),
            IRQ => self.irq(),
            None => (),
        };
        self.interrupt = InterruptType::None;

        let cpc = self.pc;
        let opcode = self.memory.read8(cpc);
        let mode = instruction_modes[opcode];
        let (address, crossed_page) = match mode {
            Absolute        => (self.memory.read16(cpc+1), false),
            AbsoluteX       => { let adr = self.memory.read16(cpc+1); let x = self.rx as u16;
                                    (adr+x, page_differ(adr, adr+x)) },
            AbsoluteY       => { let adr = self.memory.read16(cpc+1); let y = self.ry as u16;
                                    (adr+y, page_differ(adr, adr+y)) },
            Accumulator     => (0, false),
            Immediate       => (cpc, false),
            Implied         => (0, false),
            IndexedIndirect => (self.read16_bug(self.memory.read8(cpc+1) + self.rx), false),
            Indirect        => (self.read16_buf(self.memory.read16(cpc+1)), false),
            IndirectIndexed => { let y = self.ry as u16;
                                 let adr = self.memory.read8(cpc+1);
                                 let addr = self.read16_buf(adr)+y;
                                    (addr, page_differ(addr-y, addr)) },
            Relative        => {

                                },
            ZeroPage        => (self.memory.read(cpc+1) as u16, false),
            ZeroPageX       => ((self.memory.read(cpc+1) as u16) + (self.rx as u16), false),
            ZeroPageY       => ((self.memory.read(cpc+1) as u16) + (self.ry as u16), false),
        };

        self.pc += instruction_size[opcode];
        let delta_cycles = instruction_cycles[opcode] + if crossed_page { instruction_page_cycles[opcode] } else {0};
        (self.instr_table[opcode])(self, address, self.pc, mode);

        self.cycles += delta_cycles;
        delta_cycles
    }
//----------------------------------------Helper Functions-----------------------------------------
    fn page_differ(a : u16, b : u16) -> bool {
        a&0xFF00 != b & 0xFF00
    }
    pub fn flag(&self, flag : CpuFlag) -> bool {
        match flag {
            Carry       => check_bit(self.flg, 0),
            Zero        => check_bit(self.flg, 1),
            Interrupt   => check_bit(self.flg, 2),
            Decimal     => check_bit(self.flg, 3),
            Break       => check_bit(self.flg, 4),
            Overflow    => check_bit(self.flg, 6),
            Negative    => check_bit(self.flg, 7),
        }
    }
    pub fn set_flag(&mut self, flag : CpuFlag, v : bool) {
        let oflg = self.flg;
        let iv = if v {1} else {0};
        self.flg = match flag {
            Carry       => oflg | (iv<<0),
            Zero        => oflg | (iv<<1),
            Interrupt   => oflg | (iv<<2),
            Decimal     => oflg | (iv<<3),
            Break       => oflg | (iv<<4),
            Overflow    => oflg | (iv<<6),
            Negative    => oflg | (iv<<7),
        };
    }

    fn compare(&mut self, a : u8, b : u8) {
        let r = a-b;
        self.set_flag(CpuFlag::Zero,        r == 0);
        self.set_flag(CpuFlag::Negative,    r&0x80 == 0);
        self.set_flag(CpuFlag::Carry,       a>=b);
    }
    fn read16_bug(&self, adr: u16) -> u16 {
        let lo = self.memory.read8(adr) as u16;
        let hi = self.memory.read8(adr & 0xFF00 | ((adr as u8 + 1) as u16)) as u16;
        hi<<8 | lo
    }
    fn push(&mut self, v : u8) {
        let spadr = 0x100 | self.sp as u16;
        self.memory.write(spadr, v);
        self.sp -= 1;
    }
    fn pull(&mut self) -> u8 {
        self.sp += 1;
        let spadr = 0x100 | self.sp as u16;
        self.memory.read8(spadr)
    }

    fn push16(&mut self, v : u16) {
        self.push(v >> 8);
        self.push(v & 0xFF);
    }
    fn pull16(&mut self) -> u16 {
        cpu.pull() as u16 | ((cpu.pull() as u16) << 8)
    }
//-------------------------------------------------------------------------------------------------


}




























/*pub struct CPU6502<M: Memory> {
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
    pub mem :  M,
}

impl<M : Memory> CPU6502<M> {
    pub fn new(pc : u16, mm : M) -> CPU6502<M> {
        CPU6502 {
            pc: pc,
            sp: 0xff,
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

    fn push_stack(&mut self, v:u8) {
        self.mem.write(0x0100+self.sp as u16, v);
        self.sp = if self.sp == 0x00 {0xff} else {self.sp-1};
    }
    fn pull_stack(&mut self) -> u8 {
        let v = self.mem.read(0x0100+self.sp as u16);
        self.sp = if self.sp == 0xff {0x0} else {self.sp+1};
        v
    }
    fn make_state_byte(&self) -> u8 {
        (if self.flg_carry {0x80} else {0}) |
        (if self.flg_zero {0x40} else {0}) |
        (if self.flg_interrupt {0x20} else {0}) |
        (if self.flg_decimal {0x10} else {0}) |
        (if self.flg_break {0x08} else {0}) |
        (if self.flg_ovrflow {0x04} else {0}) |
        (if self.flg_negative {0x02} else {0})
    }

    fn load_state_byte(&mut self, sb : u8) {
        self.flg_carry = sb & 0x80 == 0x80;
        self.flg_zero = sb & 0x40 == 0x40;
        self.flg_interrupt = sb & 0x20 == 0x20;
        self.flg_decimal = sb & 0x10 == 0x10;
        self.flg_break = sb & 0x08 == 0x08;
        self.flg_ovrflow = sb & 0x04 == 0x04;
        self.flg_negative = sb & 0x02 == 0x02;
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
        let v = (self.mem.read(addr)as i16)-1;
        self.flg_zero = (v == 0);
        self.flg_negative = (v&0x80) == 0x80;
        self.mem.write(addr, v as u8);
    }
    fn execute_INC(&mut self, addr:u16) {
        let v = (self.mem.read(addr)as i16)+1;
        self.flg_zero = (v == 0);
        self.flg_negative = (v&0x80) == 0x80;
        self.mem.write(addr, v as u8);
    }
    fn execute_BRANCH(&mut self, opc:u8) {
        let x = opc & 0xC0;
        let y = opc & 0x20 == 0x20;
        let off = self.next_program_byte() as i16;
        if match x {
            0x0 => self.flg_negative,
            0x1 => self.flg_ovrflow,
            0x2 => self.flg_carry,
            0x3 => self.flg_zero,
            _ => panic!("invalid branching flag"),
        } == y {
            let npc = (self.pc as i16) + off;
            self.pc = npc as u16;
        };
    }
    fn execute_BIT(&mut self, addr:u16) {
        let mval = self.mem.read(addr);
        self.flg_zero = self.reg_a & mval == 0;
        self.flg_ovrflow = mval & 0x40 == 0x40;
        self.flg_negative = mval & 0x80 == 0x80;
    }
    fn execute_JMP(&mut self, addr:u16) {
        self.pc = addr;
    }
    fn execute_STY(&mut self, addr:u16) {
        self.mem.write(addr, self.reg_y);
    }
    fn execute_LDY(&mut self, v:u8) {
        self.reg_y = v;
    }
    fn execute_CPY(&mut self, v:u8) {
        self.flg_carry = self.reg_y >= v;
        self.flg_zero = self.reg_y == v;
        self.flg_negative = self.reg_y < v;
    }
    fn execute_CPX(&mut self, v:u8) {
        self.flg_carry = self.reg_x >= v;
        self.flg_zero = self.reg_x == v;
        self.flg_negative = self.reg_x < v;
    }
    fn execute_BRK(&mut self) {
        let pc = self.pc;
        self.push_stack(pc as u8);
        self.push_stack((pc >> 8) as u8);
        let stb = self.make_state_byte();
        self.push_stack(stb);
        self.flg_break = true;
        self.pc = (self.mem.read(0xFFFE)as u16)<<8 | (self.mem.read(0xFFFF)as u16);
    }
    fn execute_JSR(&mut self, addr:u16) {
        let pc = self.pc;
        self.push_stack(pc as u8 - 1);
        self.push_stack((pc>>8) as u8);
        self.pc = addr;
    }
    fn execute_RTI(&mut self) {
        let stb = self.pull_stack();
        self.load_state_byte(stb);
        self.pc = (self.pull_stack()as u16) << 8 | self.pull_stack()as u16;
    }
    fn execute_RTS(&mut self) {
        self.pc = (self.pull_stack()as u16) << 8 | self.pull_stack()as u16;
    }
    fn execute_PHP(&mut self) {
        let sb = self.make_state_byte();
        self.push_stack(sb);
    }
    fn execute_PLP(&mut self) {
        let sb = self.pull_stack();
        self.load_state_byte(sb);
    }
    fn execute_PHA(&mut self) {
        let ra = self.reg_a;
        self.push_stack(ra);
    }
    fn execute_PLA(&mut self) {
        self.reg_a = self.pull_stack();
        self.flg_zero = self.reg_a == 0;
        self.flg_negative = self.reg_a & 0x80 == 0x80;
    }
    fn execute_DEY(&mut self) { let v = self.reg_y as i16; self.reg_y = (v-1) as u8; }
    fn execute_DEX(&mut self) { let v = self.reg_x as i16; self.reg_x = (v-1) as u8; }
    fn execute_INY(&mut self) { let v = self.reg_y as i16; self.reg_y = (v+1) as u8; }
    fn execute_INX(&mut self) { let v = self.reg_x as i16; self.reg_x = (v+1) as u8; }
    fn execute_CLC(&mut self) { self.flg_carry = false; }
    fn execute_SEC(&mut self) { self.flg_carry = true; }
    fn execute_CLI(&mut self) { self.flg_interrupt = false; }
    fn execute_SEI(&mut self) { self.flg_interrupt = true; }
    fn execute_CLV(&mut self) { self.flg_ovrflow = false; }
    fn execute_CLD(&mut self) { self.flg_decimal = false; }
    fn execute_SED(&mut self) { self.flg_decimal = true; }
    fn execute_NOP(&mut self) { }
    fn execute_T(&mut self, S : char, D : char) {
        assert!(S != D);
        let val = match S {
            'a' => self.reg_a,
            'x' => self.reg_x,
            'y' => self.reg_y,
            _ => unreachable!(),
        };
        match D {
            'a' => self.reg_a = val,
            'x' => self.reg_x = val,
            'y' => self.reg_y = val,
            _ => unreachable!(),
        };
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
        self.mem.read(addr)as u16 | (self.mem.read(addr+1)as u16)<<8
    }
    fn address_idxidr(&mut self) -> u16 {
        let lo = self.next_program_byte() as u16;
        let hi = self.next_program_byte() as u16;
        let addr = (self.next_program_byte() + self.reg_x) as u16;
        self.mem.read(addr)as u16 | (self.mem.read(addr+1)as u16)<<8
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
                0x5 => self.execute_LDA(v),
                0x6 => self.execute_CMP(v),
                0x7 => self.execute_SBC(v),
                _ => panic!("invalid opcode"),
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
                _ => panic!("invalid addressing mode"),
            };
            let v = if opc != 0x4 { self.mem.read(addr) } else {0};
            match opc {
                0x0 => self.execute_ORA(v),
                0x1 => self.execute_AND(v),
                0x2 => self.execute_EOR(v),
                0x3 => self.execute_ADC(v),
                0x4 => self.execute_STA(addr),
                0x5 => self.execute_LDA(v),
                0x6 => self.execute_CMP(v),
                0x7 => self.execute_SBC(v),
                _ => panic!("invalid opcode"),
            };
        };
    }

    fn execute_instr_type1(&mut self, opc:u8, adrm:u8) {
        if adrm == 0x0 && opc == 0x5 { //LDX immediate
            let npb = self.next_program_byte();
            self.execute_LDX(npb);
        } else if adrm == 0x2 {
            match opc {
                0x0 => self.execute_ASL_A(),
                0x1 => self.execute_ROL_A(),
                0x2 => self.execute_LSR_A(),
                0x3 => self.execute_ROR_A(),
                _ => panic!("invalid opcode"),
            }
        } else {
            let addr = match adrm {
                0x1 => self.address_zeropage(),
                0x3 => self.address_absolute(),
                0x5 => self.address_zeropageX(),
                0x6 => self.address_absoluteX(),
                _ => panic!("invalid addressing mode"),
            };
            let v = if opc == 0x5 { self.mem.read(addr) } else {0};
            match opc {
                0x0 => self.execute_ASL(addr),
                0x1 => self.execute_ROL(addr),
                0x2 => self.execute_LSR(addr),
                0x3 => self.execute_ROR(addr),
                0x4 => self.execute_STX(addr),
                0x5 => self.execute_LDX(v),
                0x6 => self.execute_DEC(addr),
                0x7 => self.execute_INC(addr),
                _ => panic!("invalid opcode"),
            };
        }
    }

    fn execute_instr_type2(&mut self, opc:u8, adrm:u8) {
        match (opc, adrm) {
            (_,   0x4) => self.execute_BRANCH(opc),
            (0x1, 0x1) => { let addr = self.address_zeropage();
                                self.execute_BIT(addr) },
            (0x1, 0x3) => { let addr = self.address_absolute();
                                self.execute_BIT(addr) },
            (0x2, 0x3) => { let addr = self.address_absolute();
                                self.execute_JMP(addr) },
            (0x3, 0x3) => { let addr = self.address_indirect();
                                self.execute_JMP(addr) },
            (0x4, 0x1) => { let addr = self.address_zeropage();
                                self.execute_STY(addr) },
            (0x4, 0x3) => { let addr = self.address_absolute();
                                self.execute_STY(addr) },
            (0x4, 0x5) => { let addr = self.address_zeropageX();
                                self.execute_STY(addr) },
            (0x5, 0x0) => { let v = self.next_program_byte();
                                self.execute_LDY(v) },
            (0x5, 0x1) => { let addr = self.address_zeropage();
                            let v = self.mem.read(addr);
                            self.execute_LDY(v) },
            (0x5, 0x3) => { let addr = self.address_absolute();
                            let v = self.mem.read(addr);
                            self.execute_LDY(v) },
            (0x5, 0x5) => { let addr = self.address_zeropageX();
                            let v = self.mem.read(addr);
                            self.execute_LDY(v) },
            (0x5, 0x7) => { let addr = self.address_absoluteX();
                            let v = self.mem.read(addr);
                            self.execute_LDY(v) },
            (0x6, 0x0) => { let v = self.next_program_byte();
                            self.execute_CPY(v) },
            (0x6, 0x1) => { let addr = self.address_zeropage();
                            let v = self.mem.read(addr);
                            self.execute_CPY(v) },
            (0x6, 0x3) => { let addr = self.address_absolute();
                            let v = self.mem.read(addr);
                            self.execute_CPY(v) },
            (0x7, 0x0) => { let v = self.next_program_byte();
                            self.execute_CPX(v) },
            (0x7, 0x1) => { let addr = self.address_zeropage();
                            let v = self.mem.read(addr);
                            self.execute_CPX(v) },
            (0x7, 0x3) => { let addr = self.address_absolute();
                            let v = self.mem.read(addr);
                            self.execute_CPX(v) },
            (_, _) => panic!("invalid opcode/addressing mode"),
        }
    }

    pub fn dispatch_instr(&mut self) {
        println!("PC=${:x} SP=${:x} A=#${:x} X=#${:x} Y=#${:x}", self.pc, self.sp, self.reg_a, self.reg_x, self.reg_y);
        let istr = self.mem.read(self.pc);
        self.pc+=1;
        let opc = (istr&0xE0) >> 5;
        let adm = (istr&0x1C) >> 2;
        match istr {
            0x00 => self.execute_BRK(),
            0x20 => {   let addr = self.address_absolute();
                        self.execute_JSR(addr) },
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
            _ => match istr & 0x03 {
                0x01 => self.execute_instr_type0(opc, adm),
                0x02 => self.execute_instr_type1(opc, adm),
                0x00 => self.execute_instr_type2(opc, adm),
                _ => panic!("invalid opcode"),
            },
        };
    }
}*/
