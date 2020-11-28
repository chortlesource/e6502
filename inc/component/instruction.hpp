////////////////////////////////////////////////////////////////////////////
//
// e6502 - instruction.hpp
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

#ifndef _E6502_INSTRUCTION_HPP
#define _E6502_INSTRUCTION_HPP


/////////////////////////////////////////////////////////////
// OPCODE / ADMODE Enumeration
//

enum class OPCODE {
  INV, ADS, AND, ASL, BCC, BCS, BEQ, BIT, BMI, BNE, BPL, BRK, BVC,
  BVS, CLC, CLD, CLI, CLV, CMP, CPX, CPY, DEC, DEX, DEY, EOR, INC,
  INX, INY, JMP, JSR, LDA, LDX, LDY, LSR, NOP, ORA, PHA, PHP, PLA,
  PLP, ROL, ROR, RTI, RTS, SBC, SEC, SED, SEI, STA, STX, STY, TAX,
  TAY, TSX, TXA, TXS, TYA
};

enum class ADMODE {
  INV, ACC, IMM, ZER, ZPX, ZPY, REL, ABS, ABX, ABY, IMP, IND, INX, INY
};


/////////////////////////////////////////////////////////////
// INSTRUCTION Class
//
// The INSTRUCTION class handles calls to specific CPU
// functions for use within the instruction table

class INSTRUCTION {
  typedef void (CPU::*INSTR)(uint16_t const&);
  typedef uint16_t const& (CPU::*ADDRMODE)();

public:
  INSTRUCTION()
    : id(OPCODE::INV), mode(ADMODE::INV), cpu(nullptr), address_mode(nullptr), instruction(nullptr) {}
  INSTRUCTION(OPCODE const& i, ADMODE const& m, CPU *c, ADDRMODE addr, INSTR const& instr)
    : id(i), mode(m), cpu(c), address_mode(addr), instruction(instr) {};

  // Public INSTRUCTION methods
  void operator()() const {
    // Execure the delegate function by us of ();
    std::uint16_t data = (cpu->*address_mode)();
    (cpu->*instruction)(data);
  };

  bool operator==(INSTRUCTION const& rhs) const noexcept {
    // Compare delegates to see if the pointers match
    return (id == rhs.id) && (mode == rhs.mode);
  };

  OPCODE const& get_opcode() const noexcept { return id; }
  ADMODE const& get_admode() const noexcept { return mode; }

private:
  // Private INSTRUCTION attributes
  OPCODE   id;
  ADMODE   mode;
  CPU*     cpu;
  ADDRMODE address_mode;
  INSTR    instruction;

};


#endif // _E6502_INSTRUCTION_HPP
