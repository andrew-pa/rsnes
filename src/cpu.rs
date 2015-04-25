use std::iter::*;

pub trait Memory {
    fn read8(&self, adr:u16)->u8;
    fn read16(&self, adr:u16)->u16;
    fn write(&mut self, adr:u16, val:u8);
}

pub struct FlatMemory {
    pub main_mem : [u8; 65536]
}

impl FlatMemory {
    pub fn new() -> FlatMemory {
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
macro_rules! branch_inst {
    ( set $flag:expr, $fname:ident ) => {
        fn $fname(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
            if self.flag($flag) {
                self.pc = addr;
                self.add_branch_cycles(pc, addr);
            };
        }
    };
    ( clr $flag:expr, $fname:ident ) => {
        fn $fname(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
            if !self.flag($flag) {
                self.pc = addr;
                self.add_branch_cycles(pc, addr);
            };
        }
    }
}
macro_rules! transfer_inst {
    ( $to:ident, $from:ident, $instnm:ident ) => {
        fn $instnm(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
            let v = self.$from;
            self.$to = v;
            self.set_flag(CpuFlag::Zero, v == 0);
            self.set_flag(CpuFlag::Negative, v&0x80 != 0);
        }
    }
}

#[derive(Copy, Clone)]
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

pub struct CPU<M : Memory> {
    pub memory: M,
    pub cycles: u64,

    pc: u16,
    sp: u8,
    pub ra: u8,
    pub rx: u8,
    pub ry: u8,
    pub flg: u8,

    interrupt: InterruptType,
    stall: i32,

    instr_table: Vec<CPUInstr<M>>,
}

impl<M:Memory> CPU<M> {
    pub fn new(m : M) -> CPU<M> {
        let npc = m.read16(0xfffc);
         CPU {
            memory: m,
            cycles: 0,
            pc: npc,
            sp: 0xfd,
            ra: 0, rx: 0, ry: 0,
            flg: 0x24,
            interrupt: InterruptType::None,
            stall: 0,
            instr_table: vec![
            CPU::<M>::brk, CPU::<M>::ora, CPU::<M>::kil, CPU::<M>::slo, CPU::<M>::nop, CPU::<M>::ora, CPU::<M>::asl, CPU::<M>::slo,
            CPU::<M>::php, CPU::<M>::ora, CPU::<M>::asl, CPU::<M>::anc, CPU::<M>::nop, CPU::<M>::ora, CPU::<M>::asl, CPU::<M>::slo,
            CPU::<M>::bpl, CPU::<M>::ora, CPU::<M>::kil, CPU::<M>::slo, CPU::<M>::nop, CPU::<M>::ora, CPU::<M>::asl, CPU::<M>::slo,
            CPU::<M>::clc, CPU::<M>::ora, CPU::<M>::nop, CPU::<M>::slo, CPU::<M>::nop, CPU::<M>::ora, CPU::<M>::asl, CPU::<M>::slo,
            CPU::<M>::jsr, CPU::<M>::and, CPU::<M>::kil, CPU::<M>::rla, CPU::<M>::bit, CPU::<M>::and, CPU::<M>::rol, CPU::<M>::rla,
            CPU::<M>::plp, CPU::<M>::and, CPU::<M>::rol, CPU::<M>::anc, CPU::<M>::bit, CPU::<M>::and, CPU::<M>::rol, CPU::<M>::rla,
            CPU::<M>::bmi, CPU::<M>::and, CPU::<M>::kil, CPU::<M>::rla, CPU::<M>::nop, CPU::<M>::and, CPU::<M>::rol, CPU::<M>::rla,
            CPU::<M>::sec, CPU::<M>::and, CPU::<M>::nop, CPU::<M>::rla, CPU::<M>::nop, CPU::<M>::and, CPU::<M>::rol, CPU::<M>::rla,
            CPU::<M>::rti, CPU::<M>::eor, CPU::<M>::kil, CPU::<M>::sre, CPU::<M>::nop, CPU::<M>::eor, CPU::<M>::lsr, CPU::<M>::sre,
            CPU::<M>::pha, CPU::<M>::eor, CPU::<M>::lsr, CPU::<M>::alr, CPU::<M>::jmp, CPU::<M>::eor, CPU::<M>::lsr, CPU::<M>::sre,
            CPU::<M>::bvc, CPU::<M>::eor, CPU::<M>::kil, CPU::<M>::sre, CPU::<M>::nop, CPU::<M>::eor, CPU::<M>::lsr, CPU::<M>::sre,
            CPU::<M>::cli, CPU::<M>::eor, CPU::<M>::nop, CPU::<M>::sre, CPU::<M>::nop, CPU::<M>::eor, CPU::<M>::lsr, CPU::<M>::sre,
            CPU::<M>::rts, CPU::<M>::adc, CPU::<M>::kil, CPU::<M>::rra, CPU::<M>::nop, CPU::<M>::adc, CPU::<M>::ror, CPU::<M>::rra,
            CPU::<M>::pla, CPU::<M>::adc, CPU::<M>::ror, CPU::<M>::arr, CPU::<M>::jmp, CPU::<M>::adc, CPU::<M>::ror, CPU::<M>::rra,
            CPU::<M>::bvs, CPU::<M>::adc, CPU::<M>::kil, CPU::<M>::rra, CPU::<M>::nop, CPU::<M>::adc, CPU::<M>::ror, CPU::<M>::rra,
            CPU::<M>::sei, CPU::<M>::adc, CPU::<M>::nop, CPU::<M>::rra, CPU::<M>::nop, CPU::<M>::adc, CPU::<M>::ror, CPU::<M>::rra,
            CPU::<M>::nop, CPU::<M>::sta, CPU::<M>::nop, CPU::<M>::sax, CPU::<M>::sty, CPU::<M>::sta, CPU::<M>::stx, CPU::<M>::sax,
            CPU::<M>::dey, CPU::<M>::nop, CPU::<M>::txa, CPU::<M>::xaa, CPU::<M>::sty, CPU::<M>::sta, CPU::<M>::stx, CPU::<M>::sax,
            CPU::<M>::bcc, CPU::<M>::sta, CPU::<M>::kil, CPU::<M>::ahx, CPU::<M>::sty, CPU::<M>::sta, CPU::<M>::stx, CPU::<M>::sax,
            CPU::<M>::tya, CPU::<M>::sta, CPU::<M>::txs, CPU::<M>::tas, CPU::<M>::shy, CPU::<M>::sta, CPU::<M>::shx, CPU::<M>::ahx,
            CPU::<M>::ldy, CPU::<M>::lda, CPU::<M>::ldx, CPU::<M>::lax, CPU::<M>::ldy, CPU::<M>::lda, CPU::<M>::ldx, CPU::<M>::lax,
            CPU::<M>::tay, CPU::<M>::lda, CPU::<M>::tax, CPU::<M>::lax, CPU::<M>::ldy, CPU::<M>::lda, CPU::<M>::ldx, CPU::<M>::lax,
            CPU::<M>::bcs, CPU::<M>::lda, CPU::<M>::kil, CPU::<M>::lax, CPU::<M>::ldy, CPU::<M>::lda, CPU::<M>::ldx, CPU::<M>::lax,
            CPU::<M>::clv, CPU::<M>::lda, CPU::<M>::tsx, CPU::<M>::las, CPU::<M>::ldy, CPU::<M>::lda, CPU::<M>::ldx, CPU::<M>::lax,
            CPU::<M>::cpy, CPU::<M>::cmp, CPU::<M>::nop, CPU::<M>::dcp, CPU::<M>::cpy, CPU::<M>::cmp, CPU::<M>::dec, CPU::<M>::dcp,
            CPU::<M>::iny, CPU::<M>::cmp, CPU::<M>::dex, CPU::<M>::axs, CPU::<M>::cpy, CPU::<M>::cmp, CPU::<M>::dec, CPU::<M>::dcp,
            CPU::<M>::bne, CPU::<M>::cmp, CPU::<M>::kil, CPU::<M>::dcp, CPU::<M>::nop, CPU::<M>::cmp, CPU::<M>::dec, CPU::<M>::dcp,
            CPU::<M>::cld, CPU::<M>::cmp, CPU::<M>::nop, CPU::<M>::dcp, CPU::<M>::nop, CPU::<M>::cmp, CPU::<M>::dec, CPU::<M>::dcp,
            CPU::<M>::cpx, CPU::<M>::sbc, CPU::<M>::nop, CPU::<M>::isc, CPU::<M>::cpx, CPU::<M>::sbc, CPU::<M>::inc, CPU::<M>::isc,
            CPU::<M>::inx, CPU::<M>::sbc, CPU::<M>::nop, CPU::<M>::sbc, CPU::<M>::cpx, CPU::<M>::sbc, CPU::<M>::inc, CPU::<M>::isc,
            CPU::<M>::beq, CPU::<M>::sbc, CPU::<M>::kil, CPU::<M>::isc, CPU::<M>::nop, CPU::<M>::sbc, CPU::<M>::inc, CPU::<M>::isc,
            CPU::<M>::sed, CPU::<M>::sbc, CPU::<M>::nop, CPU::<M>::isc, CPU::<M>::nop, CPU::<M>::sbc, CPU::<M>::inc, CPU::<M>::isc],
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
        if !self.flag(CpuFlag::Interrupt) {
            self.interrupt = InterruptType::IRQ;
        }
    }

    fn page_differ(a : u16, b : u16) -> bool {
        a&0xFF00 != b & 0xFF00
    }
    pub fn step(&mut self) -> u64 {
        let instruction_size : [u16; 256] = [
            1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
            3, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
            1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
            1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 0, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 0, 3, 0, 0,
            2, 2, 2, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
            2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
        ];
        //don't worry, I used find and replace
        let instruction_modes : [AddressingMode; 256] = [
           AddressingMode::Implied,AddressingMode::IndexedIndirect,AddressingMode::Implied,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Accumulator,AddressingMode::Immediate,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,
           AddressingMode::Absolute,AddressingMode::IndexedIndirect,AddressingMode::Implied,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Accumulator,AddressingMode::Immediate,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,
           AddressingMode::Implied,AddressingMode::IndexedIndirect,AddressingMode::Implied,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Accumulator,AddressingMode::Immediate,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,
           AddressingMode::Implied,AddressingMode::IndexedIndirect,AddressingMode::Implied,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Accumulator,AddressingMode::Immediate,AddressingMode::Indirect,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,
           AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageY,AddressingMode::ZeroPageY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteY,AddressingMode::AbsoluteY,
           AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageY,AddressingMode::ZeroPageY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteY,AddressingMode::AbsoluteY,
           AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,
           AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::Immediate,AddressingMode::IndexedIndirect,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::ZeroPage,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Implied,AddressingMode::Immediate,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,AddressingMode::Absolute,
           AddressingMode::Relative,AddressingMode::IndirectIndexed,AddressingMode::Implied,AddressingMode::IndirectIndexed,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::ZeroPageX,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::Implied,AddressingMode::AbsoluteY,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,AddressingMode::AbsoluteX,
        ];
        let instruction_cycles : [u64; 256] = [
            7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
            2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
            6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
            2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
            6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
            2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
            6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
            2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
            2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
            2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
            2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
            2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
            2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
            2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
            2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
            2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
        ];
        let instruction_page_cycles : [u8; 256] = [
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
        ];
        if self.stall > 0 {
            self.stall -= 1;
            return 1;
        }

        match self.interrupt {
            InterruptType::NMI => self.nmi(),
            InterruptType::IRQ => self.irq(),
            InterruptType::None => (),
        };
        self.interrupt = InterruptType::None;

        let cpc = self.pc;
        let opcode = self.memory.read8(cpc);
        let mode = instruction_modes[opcode as usize];
        let (address, crossed_page) : (u16,bool) = match mode {
            AddressingMode::Absolute        => (self.memory.read16(cpc+1), false),
            AddressingMode::AbsoluteX       => { let adr = self.memory.read16(cpc+1); let x = self.rx as u16;
                                    (adr+x, CPU::<M>::page_differ(adr, adr+x)) },
            AddressingMode::AbsoluteY       => { let adr = self.memory.read16(cpc+1); let y = self.ry as u16;
                                    (adr+y, CPU::<M>::page_differ(adr, adr+y)) },
            AddressingMode::Accumulator     => (0, false),
            AddressingMode::Immediate       => (cpc, false),
            AddressingMode::Implied         => (0, false),
            AddressingMode::IndexedIndirect => (self.read16_bug(self.memory.read8(cpc+1) as u16 + self.rx as u16), false),
            AddressingMode::Indirect        => (self.read16_bug(self.memory.read16(cpc+1)), false),
            AddressingMode::IndirectIndexed => { let y = self.ry as u16;
                                 let adr = self.memory.read8(cpc+1) as u16;
                                 let addr = self.read16_bug(adr)+y;
                                    (addr, CPU::<M>::page_differ(addr-y, addr)) },
            AddressingMode::Relative        => {
                                    let offset = self.memory.read8(cpc+1) as u16;
                                    (if offset < 0x80 { cpc + 2 + offset }
                                        else { cpc+2+offset-0x100 }, false)
                                },
            AddressingMode::ZeroPage        => (self.memory.read8(cpc+1) as u16, false),
            AddressingMode::ZeroPageX       => ((self.memory.read8(cpc+1) as u16) + (self.rx as u16), false),
            AddressingMode::ZeroPageY       => ((self.memory.read8(cpc+1) as u16) + (self.ry as u16), false),
        };

        self.pc += instruction_size[opcode as usize];
        let delta_cycles = instruction_cycles[opcode as usize] +
                    if crossed_page {
                        instruction_page_cycles[opcode as usize] as u64 }
                    else {0};
        let _pc = self.pc;
        (self.instr_table[opcode as usize])(self, address, _pc, mode);

        self.cycles += delta_cycles;
        delta_cycles
    }
//----------------------------------------Helper Functions-----------------------------------------
    pub fn write_state(&self) {
        println!("PC=0x{:x}, SP=0x{:x}, A=0x{:x}, X=0x{:x}, Y=0x{:x}, Flag=0b{:b}",
            self.pc, self.sp, self.ra, self.rx, self.ry, self.flg);
    }
    pub fn flag(&self, flag : CpuFlag) -> bool {
        match flag {
            CpuFlag::Carry       => check_bit!(self.flg, 0),
            CpuFlag::Zero        => check_bit!(self.flg, 1),
            CpuFlag::Interrupt   => check_bit!(self.flg, 2),
            CpuFlag::Decimal     => check_bit!(self.flg, 3),
            CpuFlag::Break       => check_bit!(self.flg, 4),
            CpuFlag::Overflow    => check_bit!(self.flg, 6),
            CpuFlag::Negative    => check_bit!(self.flg, 7),
        }
    }
    pub fn set_flag(&mut self, flag : CpuFlag, v : bool) {
        let oflg = self.flg;
        let iv = if v {1} else {0};
        self.flg = match flag {
            CpuFlag::Carry       => oflg | (iv<<0),
            CpuFlag::Zero        => oflg | (iv<<1),
            CpuFlag::Interrupt   => oflg | (iv<<2),
            CpuFlag::Decimal     => oflg | (iv<<3),
            CpuFlag::Break       => oflg | (iv<<4),
            CpuFlag::Overflow    => oflg | (iv<<6),
            CpuFlag::Negative    => oflg | (iv<<7),
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
        self.push((v >> 8) as u8);
        self.push((v & 0xFF) as u8);
    }
    fn pull16(&mut self) -> u16 {
        self.pull() as u16 | ((self.pull() as u16) << 8)
    }

    fn add_branch_cycles(&mut self, pc:u16, addr:u16) {
        self.cycles += if CPU::<M>::page_differ(pc, addr) {2} else {1};
    }
//-------------------------------------------------------------------------------------------------

    fn nmi(&mut self) {
        let (pc, flg) = (self.pc, self.flg);
        self.push16(pc);
        self.push(flg);
        self.pc = self.memory.read16(0xfffa);
        self.set_flag(CpuFlag::Interrupt, true);
        self.cycles += 7;
    }

    fn irq(&mut self) {
        let (pc, flg) = (self.pc, self.flg);
        self.push16(pc);
        self.push(flg);
        self.pc = self.memory.read16(0xfffe);
        self.set_flag(CpuFlag::Interrupt, true);
        self.cycles += 7;
    }

//-------------------------------------[Instructions]----------------------------------------------

    fn adc(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let a = self.ra as u16;
        let b = self.memory.read8(addr) as u16;
        let c = if self.flag(CpuFlag::Carry) {1u16} else {0u16};
        let res = a+b+c;
        self.ra = res as u8;
        self.set_flag(CpuFlag::Zero, res == 0);
        self.set_flag(CpuFlag::Negative, (res as u8)&0x80 != 0);
        self.set_flag(CpuFlag::Carry, res>0xff);
        self.set_flag(CpuFlag::Overflow, (a^b)&0x80 == 0 && (a ^ res)&0x80 != 0);
    }

    fn and(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let a = self.ra;
        let b = self.memory.read8(addr);
        let res = a & b;
        self.ra = res;
        self.set_flag(CpuFlag::Zero, res == 0);
        self.set_flag(CpuFlag::Negative, res&0x80 != 0);
    }

    fn asl(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = match addrmd {
            AddressingMode::Accumulator => self.ra,
            _ => self.memory.read8(addr),
        };
        self.set_flag(CpuFlag::Carry, (v >> 7) & 1 == 1);
        let res = v << 1;
        match addrmd {
            AddressingMode::Accumulator => self.ra = res,
            _ => self.memory.write(addr, res),
        };
        self.set_flag(CpuFlag::Zero, res == 0);
        self.set_flag(CpuFlag::Negative, res&0x80 != 0);
    }

    branch_inst!(clr CpuFlag::Carry,        bcc);
    branch_inst!(set CpuFlag::Carry,        bcs);
    branch_inst!(set CpuFlag::Zero,         beq);
    branch_inst!(set CpuFlag::Negative,     bmi);
    branch_inst!(clr CpuFlag::Zero,         bne);
    branch_inst!(clr CpuFlag::Negative,     bpl);
    branch_inst!(clr CpuFlag::Overflow,     bvc);
    branch_inst!(set CpuFlag::Overflow,     bvs);

    fn bit(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.memory.read8(addr);
        self.set_flag(CpuFlag::Overflow, (v>>6)&1==1);
        let a = self.ra;
        self.set_flag(CpuFlag::Zero, v&a == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn brk(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.push16(pc);
        let f = self.flg;
        self.push(f);
        self.set_flag(CpuFlag::Interrupt, true);
        let npc = self.memory.read16(0xFFFE);
        self.pc = npc;
    }

    fn clc(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.set_flag(CpuFlag::Carry, false);
    }
    fn cld(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.set_flag(CpuFlag::Decimal, false);
    }
    fn cli(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.set_flag(CpuFlag::Interrupt, false);
    }
    fn clv(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.set_flag(CpuFlag::Overflow, false);
    }

    fn cmp(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let (a, v) = (self.ra, self.memory.read8(addr));
        self.compare(a, v);
    }
    fn cpx(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let (x, v) = (self.rx, self.memory.read8(addr));
        self.compare(x, v);
    }
    fn cpy(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let (y, v) = (self.ry, self.memory.read8(addr));
        self.compare(y, v);
    }

    fn dec(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.memory.read8(addr)-1;
        self.memory.write(addr, v);
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }
    fn dex(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.rx-1;
        self.rx = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }
    fn dey(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.ry-1;
        self.ry = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn eor(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.ra ^ self.memory.read8(addr);
        self.ra = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn inc(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let mut v = self.memory.read8(addr);
        v = if v == 0xff {0x0} else {v+1};
        self.memory.write(addr, v);
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }
    fn inx(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let mut v = self.rx;
        v = if v == 0xff {0x0} else {v+1};
        self.rx = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }
    fn iny(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let mut v = self.ry;
        v = if v == 0xff {0x0} else {v+1};
        self.ry = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn jmp(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.pc = addr;
    }

    fn jsr(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.push16(pc-1);
        self.pc = addr;
    }

    fn lda(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.memory.read8(addr);
        self.ra = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }
    fn ldx(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.memory.read8(addr);
        self.rx = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }
    fn ldy(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.memory.read8(addr);
        self.ry = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn lsr(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let vin = match addrmd {
            AddressingMode::Accumulator => self.ra,
            _ => self.memory.read8(addr),
        };
        self.set_flag(CpuFlag::Carry, vin&1 == 1);
        let v = vin >> 1;
        self.memory.write(addr, v);
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn nop(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}

    fn ora(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let memv = self.memory.read8(addr);
        let v = self.ra | memv;
        self.ra = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn pha(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let a = self.ra;
        self.push(a);
    }

    fn php(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let f = self.flg;
        self.push(f | 0x10);
    }

    fn pla(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = self.pull();
        self.ra = v;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
    }

    fn plp(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.flg = (self.pull() & 0xEF) | 0x20; //bit magic
    }

    fn rol(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = match addrmd {
            AddressingMode::Accumulator => self.ra,
            _ => self.memory.read8(addr),
        };
        let c = if self.flag(CpuFlag::Carry) {1} else {0};
        self.set_flag(CpuFlag::Carry, (v >> 7) & 1 == 1);
        let fv = (v << 1) | c;
        match addrmd {
            AddressingMode::Accumulator => self.ra = fv,
            _ => self.memory.write(addr, fv)
        };

        self.set_flag(CpuFlag::Zero, fv == 0);
        self.set_flag(CpuFlag::Negative, fv&0x80 != 0);
    }
    fn ror(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let v = match addrmd {
            AddressingMode::Accumulator => self.ra,
            _ => self.memory.read8(addr),
        };
        let c = if self.flag(CpuFlag::Carry) {1} else {0};
        self.set_flag(CpuFlag::Carry, v & 1 == 1);
        let fv = (v >> 1) | (c << 7);
        match addrmd {
            AddressingMode::Accumulator => self.ra = fv,
            _ => self.memory.write(addr, fv)
        };

        self.set_flag(CpuFlag::Zero, fv == 0);
        self.set_flag(CpuFlag::Negative, fv&0x80 != 0);
    }

    fn rti(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.plp(addr, pc, addrmd);
        self.pc = self.pull16();
    }

    fn rts(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.pc = self.pull16() - 1;
    }

    fn sbc(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let (a,b,c) = (self.ra, self.memory.read8(addr),
                        if self.flag(CpuFlag::Carry) {1} else {0});
        let v = a as i32 - b as i32 - (1-c) as i32;
        self.ra = v as u8;
        self.set_flag(CpuFlag::Zero, v == 0);
        self.set_flag(CpuFlag::Negative, v&0x80 != 0);
        self.set_flag(CpuFlag::Carry, v >= 0);
        self.set_flag(CpuFlag::Overflow, (a^b)&0x80 != 0 && (a^a)&0x80 != 0);
    }

    fn sec(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.set_flag(CpuFlag::Carry, true);
    }
    fn sed(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.set_flag(CpuFlag::Decimal, true);
    }
    fn sei(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        self.set_flag(CpuFlag::Interrupt, true);
    }

    fn sta(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let a = self.ra;
        self.memory.write(addr, a);
    }
    fn stx(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let a = self.rx;
        self.memory.write(addr, a);
    }
    fn sty(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {
        let a = self.ry;
        self.memory.write(addr, a);
    }

    transfer_inst!(rx, ra, tax);
    transfer_inst!(ry, ra, tay);
    transfer_inst!(ra, rx, txa);
    transfer_inst!(ra, ry, tya);
    transfer_inst!(rx, sp, tsx);
    transfer_inst!(sp, rx, txs);

    fn ahx(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn alr(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn anc(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn arr(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn axs(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn dcp(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn isc(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn kil(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn las(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn lax(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn rla(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn rra(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn sax(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn shx(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn shy(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn slo(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn sre(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn tas(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
    fn xaa(&mut self, addr:u16, pc:u16, addrmd:AddressingMode) {}
//-------------------------------------------------------------------------------------------------
}
