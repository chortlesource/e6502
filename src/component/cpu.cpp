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
