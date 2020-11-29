////////////////////////////////////////////////////////////////////////////
//
// e6502 - emulator.hpp
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

#ifndef _E6502_EMULATOR_HPP
#define _E6502_EMULATOR_HPP


/////////////////////////////////////////////////////////////
// STATUS Enumeration
//

enum class STATUS {
  INIT, RUN, PAUSE, EXIT
};

/////////////////////////////////////////////////////////////
// STATE Struct
//

struct STATE {
  STATUS  status;
  int     key;

  DISPLAY_PTR display;
  LOG_PTR     log;
  CPU_PTR     cpu;
};


/////////////////////////////////////////////////////////////
// EMULATOR Class
//
// The EMULATOR class executes the application logic

class EMULATOR {
public:
  // Public EMULATOR methods
  void initialize(int const& argc, const char *argv[]);
  void run();
  void finalize();

private:
  // Private EMULATOR attributes;
  bool  initialized;
  STATE state;

  // Private EMULATOR methods
  void parse(CLIPARSE const& parse);
};


#endif // _E6502_EMULATOR_HPP
