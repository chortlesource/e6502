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
// INSTRUCTION Class
//
// The INSTRUCTION class handles calls to specific CPU
// functions for use within the instruction table

class INSTRUCTION {
  using INSTR = std::function<void(std::uint16_t const&)>;
  using ADDRS = std::function<std::uint16_t()>;

public:
  INSTRUCTION(std::uint8_t const& i, INSTR const& instr, ADDRS addr) : id(i), instruction(instr), address_mode(addr) {};

  // Public INSTRUCTION methods
  void operator()() const {
    // Execure the delegate function by us of ();
    std::uint16_t data = address_mode();
    instruction(data);
  };

  bool operator==(INSTRUCTION const& rhs) const noexcept {
    // Compare delegates to see if the pointers match
    return (id == rhs.id);
  };


private:
  // Private INSTRUCTION attributes
  std::uint8_t id;
  INSTR        instruction;
  ADDRS        address_mode;

};


#endif // _E6502_INSTRUCTION_HPP
