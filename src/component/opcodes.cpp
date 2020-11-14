////////////////////////////////////////////////////////////////////////////
//
// e6502 - opcodes.cpp
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
// Private CPU methods - Opcode Implementation
//


// Opcode Functions
void CPU::opcode_invalid(uint16_t instr) {}

void CPU::opcode_ads(uint16_t instr) {}
void CPU::opcode_and(uint16_t instr) {}
void CPU::opcode_asl(uint16_t instr) {}
void CPU::opcode_asl_acc(uint16_t instr) {}

void CPU::opcode_bcc(uint16_t instr) {}
void CPU::opcode_bcs(uint16_t instr) {}
void CPU::opcode_beq(uint16_t instr) {}
void CPU::opcode_bit(uint16_t instr) {}
void CPU::opcode_bmi(uint16_t instr) {}
void CPU::opcode_bne(uint16_t instr) {}
void CPU::opcode_bpl(uint16_t instr) {}
void CPU::opcode_brk(uint16_t instr) {}
void CPU::opcode_bvc(uint16_t instr) {}
void CPU::opcode_bvs(uint16_t instr) {}

void CPU::opcode_clc(uint16_t instr) {}
void CPU::opcode_cld(uint16_t instr) {}
void CPU::opcode_cli(uint16_t instr) {}
void CPU::opcode_clv(uint16_t instr) {}
void CPU::opcode_cmp(uint16_t instr) {}
void CPU::opcode_cpx(uint16_t instr) {}
void CPU::opcode_cpy(uint16_t instr) {}

void CPU::opcode_dec(uint16_t instr) {}
void CPU::opcode_dex(uint16_t instr) {}
void CPU::opcode_dey(uint16_t instr) {}

void CPU::opcode_eor(uint16_t instr) {}

void CPU::opcode_inc(uint16_t instr) {}
void CPU::opcode_inx(uint16_t instr) {}
void CPU::opcode_iny(uint16_t instr) {}

void CPU::opcode_jmp(uint16_t instr) {}
void CPU::opcode_jsr(uint16_t instr) {}

void CPU::opcode_lda(uint16_t instr) {}
void CPU::opcode_ldx(uint16_t instr) {}
void CPU::opcode_ldy(uint16_t instr) {}
void CPU::opcode_lsr(uint16_t instr) {}
void CPU::opcode_lsr_acc(uint16_t instr) {}


void CPU::opcode_nop(uint16_t instr) {}

void CPU::opcode_ora(uint16_t instr) {}

void CPU::opcode_pha(uint16_t instr) {}
void CPU::opcode_php(uint16_t instr) {}
void CPU::opcode_pla(uint16_t instr) {}
void CPU::opcode_plp(uint16_t instr) {}

void CPU::opcode_rol(uint16_t instr) {}
void CPU::opcode_rol_acc(uint16_t instr) {}
void CPU::opcode_ror(uint16_t instr) {}
void CPU::opcode_ror_acc(uint16_t instr) {}
void CPU::opcode_rti(uint16_t instr) {}
void CPU::opcode_rts(uint16_t instr) {}

void CPU::opcode_sbc(uint16_t instr) {}
void CPU::opcode_sec(uint16_t instr) {}
void CPU::opcode_sed(uint16_t instr) {}
void CPU::opcode_sei(uint16_t instr) {}
void CPU::opcode_sta(uint16_t instr) {}
void CPU::opcode_stx(uint16_t instr) {}
void CPU::opcode_sty(uint16_t instr) {}

void CPU::opcode_tax(uint16_t instr) {}
void CPU::opcode_tay(uint16_t instr) {}
void CPU::opcode_tsx(uint16_t instr) {}
void CPU::opcode_txa(uint16_t instr) {}
void CPU::opcode_txs(uint16_t instr) {}
void CPU::opcode_tya(uint16_t instr) {}
