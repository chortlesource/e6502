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

}

void CPU::nmi();
void CPU::irq();
void CPU::run();
void CPU::load(std::string const& path, std::uint16_t const& mstart);


void CPU::reset() {
  state.a = 0x00;
  state.y = 0x00;
  state.x = 0x00;

  
}

/////////////////////////////////////////////////////////////
// Private CPU methods
//
