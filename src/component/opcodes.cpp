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
void CPU::opcode_invalid(uint16_t const& value) {
  // Halt the CPU in the event of an invalid op.
  state.invalid_opcode = true;
}


// ADS - Add with Carry
void CPU::opcode_ads(uint16_t const& value) {
  // Add A, Mem[Value] and Carry flag
  uint16_t ads = memory[value] + state.a + is_carry_set(state.flags);

  // BCD Support
  if(is_decimal_set(state.flags)) {
    // First add the two low nibbles and carry
    ads = (memory[value] & 0x0F) + (state.a & 0x0F) + is_carry_set(state.flags);
    // Second verify the low nibble is a valid BCD (ie. < 9) if not correct
    if(ads > 0x9) ads += 0x06;
    // Thirdly add the high nibble
    ads += (memory[value & 0xF0] + (state.a & 0xF0));
    // Fourthly check the overflow; is the carry into bit 7 is different from the carry out
    set_overflow(!((state.a ^ memory[value]) & 0x80) && ((state.a ^ ads) & 0x80), state.flags);
    // Fifthly verify our high nibble is valid if not corrct (+6)
    if(((ads & 0xF0) >> 4) > 0x9) ads += 0x60;

    // Check flags
    set_zero(!(ads & 0xFF), state.flags);    // If zero set flag
    set_carry(ads > 0xFF, state.flags);      // If exceeds 8 bits set carry
    set_negative(ads & 0x80, state.flags);  // If top bit set number is negative
    return;
  }

  set_overflow(!((state.a ^ memory[value]) & 0x80) && ((state.a ^ ads) & 0x80), state.flags);
  set_zero(!(ads & 0xFF), state.flags);
  set_carry(ads > 0xFF, state.flags);
  set_negative(ads & 0x80, state.flags);
}

// AND - Logical AND (Accumulator)
void CPU::opcode_and(uint16_t const& value) {
  state.a &= memory[value];
  // Check flags
  set_zero(!(state.a & 0xFF), state.flags);    // If zero set flag
  set_negative(state.a & 0x80, state.flags);   // If top bit set number is negative
}


// ASL - AArithmetic Shift Left (Memory)
void CPU::opcode_asl(uint16_t const& value) {
  uint8_t m = memory[value];
  memory[value] = m << 1;
  // Check flags
  set_zero(!(m & 0xFF), state.flags);    // If zero set flag
  set_carry(m & 0x80, state.flags);      // If bit 7 is set, set carry
  set_negative(m & 0x80, state.flags);   // If bit 7 is set number is negative
}


// ASL - AArithmetic Shift Left (Accumulator)
void CPU::opcode_asl_acc(uint16_t const& value) {
  uint8_t a = state.a;
  state.a = a << 1;
  // Check flags
  set_zero(!(state.a & 0xFF), state.flags); // If zero set flag
  set_carry(a & 0x80, state.flags);         // If 7 bit is set, set carry
  set_negative(a & 0x80, state.flags);      // If bit 7 is set number is negative
}


// BCC - Branch if Carry Clear
void CPU::opcode_bcc(uint16_t const& value) {
  if(!is_carry_set(state.flags))
    state.pc = value;
}

// BCS - Branch if Carry Set
void CPU::opcode_bcs(uint16_t const& value) {
  if(is_carry_set(state.flags))
    state.pc = value;
}

// BEQ - Branch if Equal
void CPU::opcode_beq(uint16_t const& value) {
  if(is_zero_set(state.flags))
    state.pc = value;
}


// BIT - Bit Test
void CPU::opcode_bit(uint16_t const& value) {
  uint8_t v = memory[value] & state.a;
  // Check flags
  set_zero(!(v & 0xFF), state.flags);  // Value not kept but used to set zero flag
  set_negative(memory[value] & 0x80, state.flags);  // Set to m(7)
  set_overflow(memory[value] & 0x40, state.flags);  // Set to m(6)
}


// BMI - Branch if Minus
void CPU::opcode_bmi(uint16_t const& value) {
  if(is_negative_set(state.flags))
    state.pc = value;
}


// BNE - Branch if Not Equal
void CPU::opcode_bne(uint16_t const& value) {
  if(!is_zero_set(state.flags))
    state.pc = value;
}

// BPL - Branch if Positive
void CPU::opcode_bpl(uint16_t const& value) {
  if(!is_negative_set(state.flags))
    state.pc = value;
}


// BRK - Force Interrupt
void CPU::opcode_brk(uint16_t const& value) {
  stack_push((state.pc >> 8) & 0xFF); // Push the high byte first
  stack_push(state.pc & 0xFF);        // Push the low byte next
  stack_push(state.flags);            // now push the flags
  set_break(true, state.flags);       // Set the IRQ
  state.pc  = memory[IRQ_HIGH] << 8;
  state.pc += memory[IRQ_LOW];
}


// BVC - Branch if Overflow Clear
void CPU::opcode_bvc(uint16_t const& value) {
  if(!is_overflow_set(state.flags))
    state.pc = value;
}


// BVS - Branch if Overflow Set
void CPU::opcode_bvs(uint16_t const& value) {
  if(is_overflow_set(state.flags))
    state.pc = value;
}


// CLC - Clear Carry Flag
void CPU::opcode_clc(uint16_t const& value) {
  set_carry(false, state.flags);
}


// CLD - Clear Decimal Mode
void CPU::opcode_cld(uint16_t const& value) {
  set_decimal(false, state.flags);
}


// CLI - Clear Interrupt Disable
void CPU::opcode_cli(uint16_t const& value) {
  set_interrupt(false, state.flags);
}


// CLV - Clear Overflow Flag
void CPU::opcode_clv(uint16_t const& value) {
  set_overflow(false, state.flags);
}


// CMP - Compare
void CPU::opcode_cmp(uint16_t const& value) {
  uint8_t v = state.a - memory[value];
  // Check flags
  set_zero(!(v & 0xFF), state.flags);
  set_negative(v & 0x80, state.flags);
  set_carry((state.a >= memory[value]), state.flags);
}


void CPU::opcode_cpx(uint16_t const& value) {}
void CPU::opcode_cpy(uint16_t const& value) {}

void CPU::opcode_dec(uint16_t const& value) {}
void CPU::opcode_dex(uint16_t const& value) {}
void CPU::opcode_dey(uint16_t const& value) {}

void CPU::opcode_eor(uint16_t const& value) {}

void CPU::opcode_inc(uint16_t const& value) {}
void CPU::opcode_inx(uint16_t const& value) {}
void CPU::opcode_iny(uint16_t const& value) {}

void CPU::opcode_jmp(uint16_t const& value) {}
void CPU::opcode_jsr(uint16_t const& value) {}

void CPU::opcode_lda(uint16_t const& value) {}
void CPU::opcode_ldx(uint16_t const& value) {}
void CPU::opcode_ldy(uint16_t const& value) {}
void CPU::opcode_lsr(uint16_t const& value) {}
void CPU::opcode_lsr_acc(uint16_t const& value) {}

void CPU::opcode_nop(uint16_t const& value) {}

void CPU::opcode_ora(uint16_t const& value) {}

void CPU::opcode_pha(uint16_t const& value) {}
void CPU::opcode_php(uint16_t const& value) {}
void CPU::opcode_pla(uint16_t const& value) {}
void CPU::opcode_plp(uint16_t const& value) {}

void CPU::opcode_rol(uint16_t const& value) {}
void CPU::opcode_rol_acc(uint16_t const& value) {}
void CPU::opcode_ror(uint16_t const& value) {}
void CPU::opcode_ror_acc(uint16_t const& value) {}
void CPU::opcode_rti(uint16_t const& value) {}
void CPU::opcode_rts(uint16_t const& value) {}

void CPU::opcode_sbc(uint16_t const& value) {}
void CPU::opcode_sec(uint16_t const& value) {}
void CPU::opcode_sed(uint16_t const& value) {}
void CPU::opcode_sei(uint16_t const& value) {}
void CPU::opcode_sta(uint16_t const& value) {}
void CPU::opcode_stx(uint16_t const& value) {}
void CPU::opcode_sty(uint16_t const& value) {}

void CPU::opcode_tax(uint16_t const& value) {}
void CPU::opcode_tay(uint16_t const& value) {}
void CPU::opcode_tsx(uint16_t const& value) {}
void CPU::opcode_txa(uint16_t const& value) {}
void CPU::opcode_txs(uint16_t const& value) {}
void CPU::opcode_tya(uint16_t const& value) {}
