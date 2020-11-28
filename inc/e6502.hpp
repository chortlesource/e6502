////////////////////////////////////////////////////////////////////////////
//
// e6502 - e6502.hpp
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

#ifndef _E6502_HPP
#define _E6502_HPP


/////////////////////////////////////////////////////////////
// DEPENDENCIES
//

// Ncurses
#include <ncurses.h>

// Standard Libraries
#include <iostream>
#include <iomanip>
#include <fstream>
#include <functional>
#include <sstream>
#include <experimental/filesystem>

#include <string>
#include <array>
#include <mutex>
#include <bitset>


/////////////////////////////////////////////////////////////
// PROGRAM INFORMATION
//

static const std::string _APP_NAME    = "e6502";
static const std::string _APP_VERSION = "0.0.1-ALPHA";
static const std::string _APP_AUTHOR  = "C. M. Short";
static const std::string _APP_SOURCE  = "http://www.github.com/chortlesoft/e6502";


/////////////////////////////////////////////////////////////
// LOCAL INCLUDES
//

#include "util/forwards.hpp"
#include "util/debug.hpp"
#include "util/timer.hpp"
#include "util/cliparse.hpp"
#include "component/instruction.hpp"
#include "component/memory.hpp"
#include "component/cpu.hpp"
#include "emulator/log.hpp"
#include "emulator/display.hpp"
#include "emulator/emulator.hpp"




#endif // _E6502_HPP
