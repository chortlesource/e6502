////////////////////////////////////////////////////////////////////////////
//
// e6502 - emulator.cpp
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
// Public EMULATOR methods
//

void EMULATOR::initialize(int const& argc, const char *argv[]) {
  if(initialized) return;

  CLIPARSE parser(argc, argv);
  std::string fpath = parser.using_opt_value("-p");
  std::string start = parser.using_opt_value("-o");
  if(fpath.empty() || start.empty()) return;

  std::stringstream converter(start);
  unsigned int value;
  converter >> std::hex >> value;

  // Initialize ncurses
  initscr();
  cbreak();
  noecho();
  curs_set(0);

  // Check terminal for colours
  if(has_colors() == false) {
    std::cerr << "[ERROR]: This terminal does not support colours.\n";
    endwin();
    return;
  }

  start_color();
  init_pair(2, COLOR_BLACK, COLOR_WHITE);

  // Initiaize the emulator state
  state.log     = std::make_shared<LOG>();
  state.display = std::make_shared<DISPLAY>();
  state.cpu     = std::make_shared<CPU>(state);

  state.status  = STATUS::RUN;
  state.display->initialize();
  state.cpu->load(fpath, value);
  initialized    = true;
}


void EMULATOR::run() {
  if(!initialized) return;

  while(state.status != STATUS::EXIT) {
    state.display->update(state);

    // Await key press and add to state
    int opt = wgetch(state.display->get_window());

    // Handle key press
    switch(opt) {
      case KEY_RESIZE:
        endwin();
        refresh();
        break;
      case 'x':
        state.status = STATUS::EXIT;
        break;
      case 's':
        state.cpu->step();
        break;
      case 'o':
        state.log->export_log();
        break;
      case 'r':
        state.cpu->run_cpu();
        break;
      case KEY_UP:
        {
          state.display->key_up();
        }
        break;
      case KEY_DOWN:
        {
          state.display->key_down(state);
        }
        break;
      default:
        state.key = opt;
        break;
    }
  }
}


void EMULATOR::finalize() {
  if(!initialized) return;

  state.display->finalize();
  state.cpu       = nullptr;
  state.display   = nullptr;
  state.log       = nullptr;

  initialized = false;
  endwin();
}

/////////////////////////////////////////////////////////////
// Private EMULATOR methods
//
