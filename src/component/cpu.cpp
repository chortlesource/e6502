////////////////////////////////////////////////////////////////////////////
//
// e6502 - cpu.cpp
//
// Copyright (c) 2020 Christopher M. Short
//
// This file is part of e6502.
//
// e6502 is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// e6502 is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with e6502. If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////

#include "e6502.hpp"


/////////////////////////////////////////////////////////////
// Public CPU methods
//

CPU::CPU() {

  // Fill the table initially with invalid opcodes
  INSTRUCTION invalid { OPCODE::INV, ADMODE::INV, this, &CPU::addr_mode_imp, &CPU::opcode_invalid };
  instructions.fill(invalid);

  // Manually Populate optable
  instructions[0x69] = INSTRUCTION {OPCODE::ADS, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_ads };
  instructions[0x6D] = INSTRUCTION {OPCODE::ADS, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_ads };
  instructions[0x65] = INSTRUCTION {OPCODE::ADS, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_ads };
  instructions[0x61] = INSTRUCTION {OPCODE::ADS, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_ads };
  instructions[0x71] = INSTRUCTION {OPCODE::ADS, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_ads };
  instructions[0x75] = INSTRUCTION {OPCODE::ADS, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_ads };
  instructions[0x7D] = INSTRUCTION {OPCODE::ADS, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_ads };
  instructions[0x79] = INSTRUCTION {OPCODE::ADS, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_ads };

  instructions[0x29] = INSTRUCTION {OPCODE::AND, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_and };
  instructions[0x2D] = INSTRUCTION {OPCODE::AND, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_and };
  instructions[0x25] = INSTRUCTION {OPCODE::AND, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_and };
  instructions[0x21] = INSTRUCTION {OPCODE::AND, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_and };
  instructions[0x31] = INSTRUCTION {OPCODE::AND, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_and };
  instructions[0x35] = INSTRUCTION {OPCODE::AND, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_and };
  instructions[0x3D] = INSTRUCTION {OPCODE::AND, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_and };
  instructions[0x39] = INSTRUCTION {OPCODE::AND, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_and };

  instructions[0x0E] = INSTRUCTION {OPCODE::ASL, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_asl };
  instructions[0x06] = INSTRUCTION {OPCODE::ASL, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_asl };
  instructions[0x0A] = INSTRUCTION {OPCODE::ASL, ADMODE::ACC, this, &CPU::addr_mode_acc, &CPU::opcode_asl };
  instructions[0x16] = INSTRUCTION {OPCODE::ASL, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_asl };
  instructions[0x1E] = INSTRUCTION {OPCODE::ASL, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_asl };

  instructions[0x90] = INSTRUCTION {OPCODE::BCC, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_bcc };

  instructions[0xB0] = INSTRUCTION {OPCODE::BCS, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_bcs };

  instructions[0xF0] = INSTRUCTION {OPCODE::BEQ, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_beq };

  instructions[0x2C] = INSTRUCTION {OPCODE::BIT, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_bit };
  instructions[0x24] = INSTRUCTION {OPCODE::BIT, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_bit };

  instructions[0x30] = INSTRUCTION {OPCODE::BMI, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_bmi };

  instructions[0xD0] = INSTRUCTION {OPCODE::BNE, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_bne };

  instructions[0x10] = INSTRUCTION {OPCODE::BPL, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_bpl };

  instructions[0x00] = INSTRUCTION {OPCODE::BRK, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_brk };

  instructions[0x50] = INSTRUCTION {OPCODE::BVC, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_bvc };

  instructions[0x70] = INSTRUCTION {OPCODE::BVS, ADMODE::REL, this, &CPU::addr_mode_rel, &CPU::opcode_bvs };

  instructions[0x18] = INSTRUCTION {OPCODE::CLC, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_clc };

  instructions[0xD8] = INSTRUCTION {OPCODE::CLD, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_cld };

  instructions[0x58] = INSTRUCTION {OPCODE::CLI, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_cli };

  instructions[0xB8] = INSTRUCTION {OPCODE::CLV, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_clv };

  instructions[0xC9] = INSTRUCTION {OPCODE::CMP, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_cmp };
  instructions[0xCD] = INSTRUCTION {OPCODE::CMP, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_cmp };
  instructions[0xC5] = INSTRUCTION {OPCODE::CMP, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_cmp };
  instructions[0xC1] = INSTRUCTION {OPCODE::CMP, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_cmp };
  instructions[0xD1] = INSTRUCTION {OPCODE::CMP, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_cmp };
  instructions[0xD5] = INSTRUCTION {OPCODE::CMP, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_cmp };
  instructions[0xDD] = INSTRUCTION {OPCODE::CMP, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_cmp };
  instructions[0xD9] = INSTRUCTION {OPCODE::CMP, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_cmp };

  instructions[0xE0] = INSTRUCTION {OPCODE::CPX, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_cpx };
  instructions[0xEC] = INSTRUCTION {OPCODE::CPX, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_cpx };
  instructions[0xE4] = INSTRUCTION {OPCODE::CPX, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_cpx };

  instructions[0xC0] = INSTRUCTION {OPCODE::CPY, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_cpy };
  instructions[0xCC] = INSTRUCTION {OPCODE::CPY, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_cpy };
  instructions[0xC4] = INSTRUCTION {OPCODE::CPY, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_cpy };

  instructions[0xCE] = INSTRUCTION {OPCODE::DEC, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_dec };
  instructions[0xC6] = INSTRUCTION {OPCODE::DEC, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_dec };
  instructions[0xD6] = INSTRUCTION {OPCODE::DEC, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_dec };
  instructions[0xDE] = INSTRUCTION {OPCODE::DEC, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_dec };

  instructions[0xCA] = INSTRUCTION {OPCODE::DEX, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_dex };

  instructions[0x88] = INSTRUCTION {OPCODE::DEY, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_dey };

  instructions[0x49] = INSTRUCTION {OPCODE::EOR, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_eor };
  instructions[0x4D] = INSTRUCTION {OPCODE::EOR, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_eor };
  instructions[0x45] = INSTRUCTION {OPCODE::EOR, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_eor };
  instructions[0x41] = INSTRUCTION {OPCODE::EOR, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_eor };
  instructions[0x51] = INSTRUCTION {OPCODE::EOR, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_eor };
  instructions[0x55] = INSTRUCTION {OPCODE::EOR, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_eor };
  instructions[0x5D] = INSTRUCTION {OPCODE::EOR, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_eor };
  instructions[0x59] = INSTRUCTION {OPCODE::EOR, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_eor };

  instructions[0xEE] = INSTRUCTION {OPCODE::INC, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_inc };
  instructions[0xE6] = INSTRUCTION {OPCODE::INC, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_inc };
  instructions[0xF6] = INSTRUCTION {OPCODE::INC, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_inc };
  instructions[0xFE] = INSTRUCTION {OPCODE::INC, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_inc };

  instructions[0xE8] = INSTRUCTION {OPCODE::INX, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_inx };

  instructions[0xC8] = INSTRUCTION {OPCODE::INY, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_iny };

  instructions[0x4C] = INSTRUCTION {OPCODE::JMP, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_jmp };
  instructions[0x6C] = INSTRUCTION {OPCODE::JMP, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_jmp };

  instructions[0x20] = INSTRUCTION {OPCODE::JSR, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_jsr };

  instructions[0xA9] = INSTRUCTION {OPCODE::LDA, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_lda };
  instructions[0xAD] = INSTRUCTION {OPCODE::LDA, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_lda };
  instructions[0xA5] = INSTRUCTION {OPCODE::LDA, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_lda };
  instructions[0xA1] = INSTRUCTION {OPCODE::LDA, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_lda };
  instructions[0xB1] = INSTRUCTION {OPCODE::LDA, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_lda };
  instructions[0xB5] = INSTRUCTION {OPCODE::LDA, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_lda };
  instructions[0xBD] = INSTRUCTION {OPCODE::LDA, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_lda };
  instructions[0xB9] = INSTRUCTION {OPCODE::LDA, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_lda };

  instructions[0xA2] = INSTRUCTION {OPCODE::LDX, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_ldx };
  instructions[0xAE] = INSTRUCTION {OPCODE::LDX, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_ldx };
  instructions[0xA6] = INSTRUCTION {OPCODE::LDX, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_ldx };
  instructions[0xBE] = INSTRUCTION {OPCODE::LDX, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_ldx };
  instructions[0xB6] = INSTRUCTION {OPCODE::LDX, ADMODE::ZPY, this, &CPU::addr_mode_zpy, &CPU::opcode_ldx };

  instructions[0xA0] = INSTRUCTION {OPCODE::LDY, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_ldy };
  instructions[0xAC] = INSTRUCTION {OPCODE::LDY, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_ldy };
  instructions[0xA4] = INSTRUCTION {OPCODE::LDY, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_ldy };
  instructions[0xB4] = INSTRUCTION {OPCODE::LDY, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_ldy };
  instructions[0xBC] = INSTRUCTION {OPCODE::LDY, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_ldy };

/*


  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_lsr;
  instrtable[0x4E] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_lsr;
  instrtable[0x46] = instr;
  instr.addr_mode = &m6502::addrmode_acc;
  instr.opcode = &m6502::opcode_lsr_acc;
  instrtable[0x4A] = instr;
  instr.addr_mode = &m6502::addrmode_zpx;
  instr.opcode = &m6502::opcode_lsr;
  instrtable[0x56] = instr;
  instr.addr_mode = &m6502::addrmode_abx;
  instr.opcode = &m6502::opcode_lsr;
  instrtable[0x5E] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_nop;
  instrtable[0xEA] = instr;

  instr.addr_mode = &m6502::addrmode_imm;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x09] = instr;
  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x0D] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x05] = instr;
  instr.addr_mode = &m6502::addrmode_inx;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x01] = instr;
  instr.addr_mode = &m6502::addrmode_iny;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x11] = instr;
  instr.addr_mode = &m6502::addrmode_zpx;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x15] = instr;
  instr.addr_mode = &m6502::addrmode_abx;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x1D] = instr;
  instr.addr_mode = &m6502::addrmode_aby;
  instr.opcode = &m6502::opcode_ora;
  instrtable[0x19] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_pha;
  instrtable[0x48] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_php;
  instrtable[0x08] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_pla;
  instrtable[0x68] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_plp;
  instrtable[0x28] = instr;

  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_rol;
  instrtable[0x2E] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_rol;
  instrtable[0x26] = instr;
  instr.addr_mode = &m6502::addrmode_acc;
  instr.opcode = &m6502::opcode_rol_acc;
  instrtable[0x2A] = instr;
  instr.addr_mode = &m6502::addrmode_zpx;
  instr.opcode = &m6502::opcode_rol;
  instrtable[0x36] = instr;
  instr.addr_mode = &m6502::addrmode_abx;
  instr.opcode = &m6502::opcode_rol;
  instrtable[0x3E] = instr;

  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_ror;
  instrtable[0x6E] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_ror;
  instrtable[0x66] = instr;
  instr.addr_mode = &m6502::addrmode_acc;
  instr.opcode = &m6502::opcode_ror_acc;
  instrtable[0x6A] = instr;
  instr.addr_mode = &m6502::addrmode_zpx;
  instr.opcode = &m6502::opcode_ror;
  instrtable[0x76] = instr;
  instr.addr_mode = &m6502::addrmode_abx;
  instr.opcode = &m6502::opcode_ror;
  instrtable[0x7E] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_rti;
  instrtable[0x40] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_rts;
  instrtable[0x60] = instr;

  instr.addr_mode = &m6502::addrmode_imm;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xE9] = instr;
  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xED] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xE5] = instr;
  instr.addr_mode = &m6502::addrmode_inx;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xE1] = instr;
  instr.addr_mode = &m6502::addrmode_iny;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xF1] = instr;
  instr.addr_mode = &m6502::addrmode_zpx;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xF5] = instr;
  instr.addr_mode = &m6502::addrmode_abx;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xFD] = instr;
  instr.addr_mode = &m6502::addrmode_aby;
  instr.opcode = &m6502::opcode_sbc;
  instrtable[0xF9] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_sec;
  instrtable[0x38] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_sed;
  instrtable[0xF8] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_sei;
  instrtable[0x78] = instr;

  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_sta;
  instrtable[0x8D] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_sta;
  instrtable[0x85] = instr;
  instr.addr_mode = &m6502::addrmode_inx;
  instr.opcode = &m6502::opcode_sta;
  instrtable[0x81] = instr;
  instr.addr_mode = &m6502::addrmode_iny;
  instr.opcode = &m6502::opcode_sta;
  instrtable[0x91] = instr;
  instr.addr_mode = &m6502::addrmode_zpx;
  instr.opcode = &m6502::opcode_sta;
  instrtable[0x95] = instr;
  instr.addr_mode = &m6502::addrmode_abx;
  instr.opcode = &m6502::opcode_sta;
  instrtable[0x9D] = instr;
  instr.addr_mode = &m6502::addrmode_aby;
  instr.opcode = &m6502::opcode_sta;
  instrtable[0x99] = instr;

  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_stx;
  instrtable[0x8E] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_stx;
  instrtable[0x86] = instr;
  instr.addr_mode = &m6502::addrmode_zpy;
  instr.opcode = &m6502::opcode_stx;
  instrtable[0x96] = instr;

  instr.addr_mode = &m6502::addrmode_abs;
  instr.opcode = &m6502::opcode_sty;
  instrtable[0x8C] = instr;
  instr.addr_mode = &m6502::addrmode_zer;
  instr.opcode = &m6502::opcode_sty;
  instrtable[0x84] = instr;
  instr.addr_mode = &m6502::addrmode_zpx;
  instr.opcode = &m6502::opcode_sty;
  instrtable[0x94] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_tax;
  instrtable[0xAA] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_tay;
  instrtable[0xA8] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_tsx;
  instrtable[0xBA] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_txa;
  instrtable[0x8A] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_txs;
  instrtable[0x9A] = instr;

  instr.addr_mode = &m6502::addrmode_imp;
  instr.opcode = &m6502::opcode_tya;
  instrtable[0x98] = instr;
  */

  reset();  // Reset the CPUSTATE
}

void CPU::nmi() {
  set_break(false, state.flags);

  stack_push((state.pc >> 8) & 0xFF);   // Push high byte addr
  stack_push(state.pc & 0xFF);          // Push low byte addr
  stack_push(state.flags);

  set_interrupt(true, state.flags);
  state.pc = (memory[NMI_HIGH] << 8) + memory[NMI_LOW];
}


void CPU::irq() {
  if(is_interrupt_set(state.flags)) {
    set_break(false, state.flags);
    stack_push((state.pc >> 8) & 0xFF);   // Push high byte addr
    stack_push(state.pc & 0xFF);          // Push low byte addr
    stack_push(state.flags);

    set_interrupt(true, state.flags);
    state.pc = (memory[IRQ_HIGH] << 8) + memory[IRQ_LOW];
  }
}


uint8_t const& CPU::step() {
  // Do something
  return state.cycles;
}


void CPU::load(std::string const& path, std::uint16_t const& mstart) {

}


void CPU::reset() {
  state.a              = 0x00;
  state.y              = 0x00;
  state.x              = 0x00;
  state.pc             = (memory[RESET_HIGH] << 8) + (memory[RESET_LOW]);
  state.sp             = 0xFD;
  state.flags         |= FLAG_CON_MASK;
  state.cycles         = 6;
  state.invalid_opcode = false;
  state.initialized    = true;
}


/////////////////////////////////////////////////////////////
// Private CPU methods
//

void CPU::stack_push(std::uint16_t const& data) {
  memory[0x100 + state.sp] = data;

  // Check for overflow
  if(state.sp == 0x00) {
    ERROR("CPU Error: Stack Overflow");
    return;
  }

  --state.sp;
}


uint8_t const& CPU::stack_pop() {
  // Check for underflow
  if(state.sp == 0xFF)
    ERROR("CPU Error: Stack is empty");
  else
    ++state.sp;

  return memory[0x100 + state.sp];
}


/////////////////////////////////////////////////////////////
// Private CPU methods
//

// Addressing Modes
uint16_t const& CPU::addr_mode_acc() {
  // Accumulator- Use the accumulator
  return state.addr = state.a;
}


uint16_t const& CPU::addr_mode_imm() {
  // Immediate- the next byte in the program counter is the operand
  return state.pc += 1;
}


uint16_t const& CPU::addr_mode_zer() {
  // Zero page has only an 8 bit address operand which is the next byte
  return state.addr = memory[state.pc++];
}


uint16_t const& CPU::addr_mode_zpx() {
  // Zero Page X - take zero page and add to the current value of X
  // then wrap to constrain to < 0xFF
  state.addr = (memory[state.pc++] + state.x) % 256;
  return state.addr;
}


uint16_t const& CPU::addr_mode_zpy() {
  // Zero Page Y - take zero page and add to the current value of Y
  // then wrap to constrain to < 0xFF
  state.addr = (memory[state.pc++] + state.y) % 256;
  return state.addr;
}


uint16_t const& CPU::addr_mode_rel() {
  // Relative - if(condition == true) a signed 8 bit relative offset
  // is added to program counter
  state.addr = memory[state.pc++];
  if(state.addr & 0x80) state.addr |= 0xFF0;
  return state.addr += state.pc;
}


uint16_t const& CPU::addr_mode_abs() {
  // Absolute- stores in next two bytes as low order then high order
  state.addr          = memory[state.pc++];
  std::uint16_t high  = memory[state.pc++] << 8;
  return state.addr  += high;
}


uint16_t const& CPU::addr_mode_abx() {
  // Absolute X- stores in next two bytes as low order then high order + X
  state.addr          = memory[state.pc++];
  std::uint16_t high  = memory[state.pc++] << 8;
  return state.addr  += high + state.x;
}


uint16_t const& CPU::addr_mode_aby() {
  // Absolute Y- stores in next two bytes as low order then high order + Y
  state.addr         = memory[state.pc++];
  uint8_t high  = memory[state.pc++] << 8;
  return state.addr += high + state.y;
}


uint16_t const& CPU::addr_mode_imp() {
  // Implicit - The address is specified in the instruction so return junk data
  state.addr = 0;
  return state.addr;
}


uint16_t const& CPU::addr_mode_ind() {
  // Indirect - Instr identifies the location of the least significant byte
  // of another 16 bit memory address which is the target of the instruction
  std::uint16_t low  = memory[state.pc++];
  std::uint16_t high = memory[state.pc++] << 8;

  std::uint16_t next = memory[low + high + 1] << 8;
  return state.addr  = memory[low + high] + next + 0x100;
}


uint16_t const& CPU::addr_mode_inx() {
  // Indexed Indirect - Addr taken from the instruction and the X register added
  // to it (with zero page wrap around)
  state.addr         = (memory[state.pc++] + state.x) % 256;
  std::uint16_t high = memory[((state.addr + 1) % 256)];
  return state.addr  = memory[state.addr] + (high << 8);
}


uint16_t const& CPU::addr_mode_iny() {
  // Indirect Indexed -  instruction contains the zero page location of the
  // lsb. The Y register is added to generate the target address for operation.
  state.addr         = memory[state.pc++];
  std::uint16_t high = memory[(state.addr + 1) % 256];
  return state.addr  = memory[state.addr] + (high << 8) + state.y;
}
