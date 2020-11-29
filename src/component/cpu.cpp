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

CPU::CPU(STATE const& s) {
  // Initialize the CPU log
  log = s.log;

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
  instructions[0x0A] = INSTRUCTION {OPCODE::ASL, ADMODE::ACC, this, &CPU::addr_mode_acc, &CPU::opcode_asl_acc };
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

  instructions[0x4E] = INSTRUCTION {OPCODE::LSR, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_lsr };
  instructions[0x46] = INSTRUCTION {OPCODE::LSR, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_lsr };
  instructions[0x4A] = INSTRUCTION {OPCODE::LSR, ADMODE::ACC, this, &CPU::addr_mode_acc, &CPU::opcode_lsr_acc };
  instructions[0x56] = INSTRUCTION {OPCODE::LSR, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_lsr };
  instructions[0x5E] = INSTRUCTION {OPCODE::LSR, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_lsr };

  instructions[0xEA] = INSTRUCTION {OPCODE::NOP, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_nop };

  instructions[0x09] = INSTRUCTION {OPCODE::ORA, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_ora };
  instructions[0x0D] = INSTRUCTION {OPCODE::ORA, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_ora };
  instructions[0x05] = INSTRUCTION {OPCODE::ORA, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_ora };
  instructions[0x01] = INSTRUCTION {OPCODE::ORA, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_ora };
  instructions[0x11] = INSTRUCTION {OPCODE::ORA, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_ora };
  instructions[0x15] = INSTRUCTION {OPCODE::ORA, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_ora };
  instructions[0x1D] = INSTRUCTION {OPCODE::ORA, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_ora };
  instructions[0x19] = INSTRUCTION {OPCODE::ORA, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_ora };

  instructions[0x48] = INSTRUCTION {OPCODE::PHA, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_pha };

  instructions[0x08] = INSTRUCTION {OPCODE::PHP, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_php };

  instructions[0x68] = INSTRUCTION {OPCODE::PLA, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_pla };

  instructions[0x28] = INSTRUCTION {OPCODE::PLP, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_plp };

  instructions[0x2E] = INSTRUCTION {OPCODE::ROL, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_rol };
  instructions[0x26] = INSTRUCTION {OPCODE::ROL, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_rol };
  instructions[0x2A] = INSTRUCTION {OPCODE::ROL, ADMODE::ACC, this, &CPU::addr_mode_acc, &CPU::opcode_rol_acc };
  instructions[0x36] = INSTRUCTION {OPCODE::ROL, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_rol };
  instructions[0x3E] = INSTRUCTION {OPCODE::ROL, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_rol };

  instructions[0x6E] = INSTRUCTION {OPCODE::ROR, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_ror };
  instructions[0x66] = INSTRUCTION {OPCODE::ROR, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_ror };
  instructions[0x6A] = INSTRUCTION {OPCODE::ROR, ADMODE::ACC, this, &CPU::addr_mode_acc, &CPU::opcode_ror_acc };
  instructions[0x76] = INSTRUCTION {OPCODE::ROR, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_ror };
  instructions[0x7E] = INSTRUCTION {OPCODE::ROR, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_ror };

  instructions[0x40] = INSTRUCTION {OPCODE::RTI, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_rti };

  instructions[0x60] = INSTRUCTION {OPCODE::RTS, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_rts };

  instructions[0xE9] = INSTRUCTION {OPCODE::SBC, ADMODE::IMM, this, &CPU::addr_mode_imm, &CPU::opcode_sbc };
  instructions[0xED] = INSTRUCTION {OPCODE::SBC, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_sbc };
  instructions[0xE5] = INSTRUCTION {OPCODE::SBC, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_sbc };
  instructions[0xE1] = INSTRUCTION {OPCODE::SBC, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_sbc };
  instructions[0xF1] = INSTRUCTION {OPCODE::SBC, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_sbc };
  instructions[0xF5] = INSTRUCTION {OPCODE::SBC, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_sbc };
  instructions[0xFD] = INSTRUCTION {OPCODE::SBC, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_sbc };
  instructions[0xF9] = INSTRUCTION {OPCODE::SBC, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_sbc };

  instructions[0x38] = INSTRUCTION {OPCODE::SEC, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_sec };

  instructions[0xF8] = INSTRUCTION {OPCODE::SED, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_sed };

  instructions[0x78] = INSTRUCTION {OPCODE::SEI, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_sei };

  instructions[0x8D] = INSTRUCTION {OPCODE::STA, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_sta };
  instructions[0x85] = INSTRUCTION {OPCODE::STA, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_sta };
  instructions[0x81] = INSTRUCTION {OPCODE::STA, ADMODE::INX, this, &CPU::addr_mode_inx, &CPU::opcode_sta };
  instructions[0x91] = INSTRUCTION {OPCODE::STA, ADMODE::INY, this, &CPU::addr_mode_iny, &CPU::opcode_sta };
  instructions[0x95] = INSTRUCTION {OPCODE::STA, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_sta };
  instructions[0x9D] = INSTRUCTION {OPCODE::STA, ADMODE::ABX, this, &CPU::addr_mode_abx, &CPU::opcode_sta };
  instructions[0x99] = INSTRUCTION {OPCODE::STA, ADMODE::ABY, this, &CPU::addr_mode_aby, &CPU::opcode_sta };

  instructions[0x8E] = INSTRUCTION {OPCODE::STX, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_stx };
  instructions[0x86] = INSTRUCTION {OPCODE::STX, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_stx };
  instructions[0x96] = INSTRUCTION {OPCODE::STX, ADMODE::ZPY, this, &CPU::addr_mode_zpy, &CPU::opcode_stx };

  instructions[0x8C] = INSTRUCTION {OPCODE::STY, ADMODE::ABS, this, &CPU::addr_mode_abs, &CPU::opcode_stx };
  instructions[0x84] = INSTRUCTION {OPCODE::STY, ADMODE::ZER, this, &CPU::addr_mode_zer, &CPU::opcode_stx };
  instructions[0x94] = INSTRUCTION {OPCODE::STY, ADMODE::ZPX, this, &CPU::addr_mode_zpx, &CPU::opcode_stx };

  instructions[0xAA] = INSTRUCTION {OPCODE::TAX, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_tax };

  instructions[0xA8] = INSTRUCTION {OPCODE::TAY, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_tay };

  instructions[0xBA] = INSTRUCTION {OPCODE::TSX, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_tsx };

  instructions[0x8A] = INSTRUCTION {OPCODE::TXA, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_txa };

  instructions[0x9A] = INSTRUCTION {OPCODE::TXS, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_txs };

  instructions[0x98] = INSTRUCTION {OPCODE::TYA, ADMODE::IMP, this, &CPU::addr_mode_imp, &CPU::opcode_tya };

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
  state.opcode = read(state.pc++);  // Fetch
  instructions[state.opcode](); // Decode and execute
  log->log_cpu(state, instructions[state.opcode]);

  // Do something
  return state.cycles;
}


void CPU::run_cpu() {
  for(unsigned int i = 0; i < 10000 && !state.invalid_opcode; i++)
    step();
}


void CPU::load(std::string const& path, uint16_t const& mstart) {
  // Attempt to load our binary file
  memory.load(path);

  // Set the PC to point to the beginning of the program
  state.pc = mstart;
}


void CPU::reset() {
  // initialize the CPU default state
  state.a              = 0x00;
  state.y              = 0x00;
  state.x              = 0x00;
  state.pc             = (memory[RESET_HIGH] << 8) + (memory[RESET_LOW]);
  state.sp             = 0xFD;
  state.flags         |= FLAG_CON_MASK;
  state.cycles         = 6;
  state.invalid_opcode = false;
  state.initialized    = true;
  memory.reset();
}


std::uint8_t const& CPU::read(std::uint16_t const& addr) {
  log->log_read(addr, memory[addr]);
  return memory[addr];
}


void CPU::write(std::uint16_t const& addr, std::uint8_t const& val) {
  log->log_write(addr, val);
  memory[addr] = val;
}


/////////////////////////////////////////////////////////////
// Private CPU methods
//

void CPU::stack_push(uint16_t const& data) {
  write(0x100 + state.sp, data);

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

  return read(0x100 + state.sp);
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
  return state.pc;
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
  state.addr = read(state.pc++);
  if(state.addr & 0x80) state.addr |= 0xFF00;
  return state.addr += state.pc;
}


uint16_t const& CPU::addr_mode_abs() {
  // Absolute- stores in next two bytes as low order then high order
  state.addr     = read(state.pc++);
  uint16_t high  = read(state.pc) << 8;
  return state.addr  += high;
}


uint16_t const& CPU::addr_mode_abx() {
  // Absolute X- stores in next two bytes as low order then high order + X
  state.addr          = memory[state.pc++];
  uint16_t high  = memory[state.pc++] << 8;
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
  uint16_t low  = memory[state.pc++];
  uint16_t high = memory[state.pc++] << 8;

  uint16_t next = memory[low + high + 1] << 8;
  return state.addr  = memory[low + high] + next + 0x100;
}


uint16_t const& CPU::addr_mode_inx() {
  // Indexed Indirect - Addr taken from the instruction and the X register added
  // to it (with zero page wrap around)
  state.addr         = (memory[state.pc++] + state.x) % 256;
  uint16_t high = memory[((state.addr + 1) % 256)];
  return state.addr  = memory[state.addr] + (high << 8);
}


uint16_t const& CPU::addr_mode_iny() {
  // Indirect Indexed -  instruction contains the zero page location of the
  // lsb. The Y register is added to generate the target address for operation.
  state.addr         = memory[state.pc++];
  uint16_t high = memory[(state.addr + 1) % 256];
  return state.addr  = memory[state.addr] + (high << 8) + state.y;
}
