////////////////////////////////////////////////////////////////////////////
//
// e6502 - memory.hpp
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

#ifndef _E6502_MEMORY_HPP
#define _E6502_MEMORY_HPP


/////////////////////////////////////////////////////////////
// DEFINITIONS
//

static const std::uint16_t RAM_SIZE = 65535;

namespace fs = std::experimental::filesystem;


/////////////////////////////////////////////////////////////
// MEMORY Class
//
// The MEMORY Class emulates 65535 bytes of RAM

class MEMORY {
public:
  MEMORY();
  ~MEMORY() {};

  // Public MEMORY methods
  void load(std::string const& path, std::uint16_t const& mstart);

  std::uint8_t &      operator[](std::uint16_t const& addr) { return memory[addr]; }
  std::uint8_t const& operator[](std::uint16_t const& addr) const { return memory[addr]; }

private:
  // Private MEMORY attributes
  bool                               initialized;
  std::array<std::uint8_t, RAM_SIZE> memory;

  // Private MEMORY methods
  static inline bool fexist(std::string const& path) {
    // Identify if file exists and return true if exists
    return (fs::exists(fs::path(path)) && fs::is_regular_file(fs::path(path)));
  }

};


#endif // _E6502_MEMORY_HPP
