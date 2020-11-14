////////////////////////////////////////////////////////////////////////////
//
// e6502 - cpu.hpp
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

#ifndef _E6502_CPU_HPP
#define _E6502_CPU_HPP


/////////////////////////////////////////////////////////////
// CPUSTATE struct
//
// The CPUSTATE is the container for the current CPU state

struct CPUSTATE {
  bool initialized;
  bool invalid_opcode;

  std::uint64_t cycles;  // Record the number of cycles

  std::uint8_t a;        // Accumulator
  std::uint8_t x;        // Index register
  std::uint8_t y;        // Index register
  std::uint8_t sp;       // Stack pointer
  std::uint16_t pc;      // Program counter
  std::uint8_t flags;    // CPU Status flags
};


/////////////////////////////////////////////////////////////
// CPU Class
//
// The CPU Class handles CPU logic manipulating the CPUSTATE


class CPU {
public:
  // Public CPU methods
  CPU();

  void nmi();
  void irq();
  void run();
  void load(std::string const& path, std::uint16_t const& mstart);
  void reset();

private:
  // Private CPU attributes
  CPUSTATE state;
  MEMORY   memory;

  // Private CPU methods
};


#endif // _E6502_CPU_HPP
