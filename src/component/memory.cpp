////////////////////////////////////////////////////////////////////////////
//
// e6502 - memory.cpp
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
// Public MEMORY methods
//

MEMORY::MEMORY() {
  initialized = false;
  memory.fill(0);
};


void MEMORY::load(std::string const& path, std::uint16_t const& mstart) {
  // Perform some generic verification
  if(fexist(path)) {

    // Open a file stream
    std::ifstream input(path, std::ios::binary);
    if(input.is_open()) {

      // Clarify the file size
      input.seekg(0, input.end);
      auto file_size = input.tellg();
      input.seekg(0, input.beg);

      // Verify we can manage the file size
      if(file_size > (RAM_SIZE + mstart)) {
        ERROR("Failed to open file; File exceeds RAM capacity: ", path);
        return;
      }

      // Read in the file to memory
      char *buffer   = new char[file_size];
      input.read(buffer, file_size);

      for(unsigned int i = 0; i < file_size; i++)
        memory[mstart + i] = static_cast<std::uint8_t>(buffer[i]);

      // Report success and free memory
      DEBUG("Binary file loaded: ", path);
      delete [] buffer; // Free memory
      initialized    = true;
      input.close();

    }

    ERROR("Failed to open file; cannot access: ", path);
    return;
  }

  ERROR("Failed to open; file does not exist: ", path);
}
