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

void EMULATOR::initialize() {
  if(initialized) return;
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
  state = std::make_shared<STATE>();
  state->status  = STATUS::RUN;
  state->display.initialize();
  state->cpu.load("./asset/6502_functional_test.bin", 0x0);
  initialized    = true;
}


void EMULATOR::run() {
  while(state->status != STATUS::EXIT) {
    state->display.update(*state);

    // Await key press and add to state
    int opt = wgetch(state->display.get_window());

    // Handle key press
    switch(opt) {
      case KEY_RESIZE:
        endwin();
        refresh();
        break;
      case 'x':
        state->status = STATUS::EXIT;
        break;
      case 's':
        state->cpu.step(*state);
        break;
      case KEY_UP:
        {
          state->display.key_up();
        }
        break;
      case KEY_DOWN:
        {
          state->display.key_down(*state);
        }
        break;
      default:
        state->key = opt;
        break;
    }
  }
}


void EMULATOR::finalize() {
  state->display.finalize();
  state       = nullptr;
  initialized = false;
  endwin();
}

/////////////////////////////////////////////////////////////
// Private EMULATOR methods
//
