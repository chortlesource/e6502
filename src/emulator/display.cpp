////////////////////////////////////////////////////////////////////////////
//
// e6502 - display.cpp
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
// Public DISPLAY methods
//


void DISPLAY::initialize() {
  if(initialized) return;
  // Obtain the current term dimensions and create the screen
  getmaxyx(stdscr, height, width);

  std::shared_ptr<WINDOW> temp(newwin(height, width, 0, 0), [=](WINDOW *window){ delwin(window);});
  window = temp;

  // Capture the keyboard
  keypad(window.get(), true);
  initialized = true;
}


void DISPLAY::update(STATE const& state) {
  // Protect against premature calls to update
  if(!initialized) return;

  // Ensure the window is the correct size;
  getmaxyx(stdscr, height, width);
  wresize(window.get(), height, width);

  // Clear the window
  werase(window.get());



  // Print the header and footer
  print_header();
  print_logger(state);
  print_footer();

  wrefresh(window.get());
}


void DISPLAY::finalize() {
  // Reset components to original values
  wclear(window.get());

  initialized = false;
  window      = nullptr;
  width       = 0;
  height      = 0;
  offset      = 0;
}


void DISPLAY::key_down(STATE const& state) {
  if(state.log->size() < (unsigned)(height - 2))
    return;

  if(offset < (state.log->size() - height + 3))
    offset += 1;
}



void DISPLAY::key_up() {
 if(offset > 0)
  --offset;
}


/////////////////////////////////////////////////////////////
// Private DISPLAY methods
//

void DISPLAY::print_header() {
  // Create our header string
  std::string header = _APP_NAME +  " - " + _APP_VERSION;

  // Print the header border to the window
  wattron(window.get(), COLOR_PAIR(2));
  for(int i = 0; i < width; i++)
    mvwaddch(window.get(), 0, i, ' ');

  // Print the header string and component name
  mvwprintw(window.get(), 0, 2, header.c_str());
  wattroff(window.get(), COLOR_PAIR(2));
}


void DISPLAY::print_logger(STATE const& state) {
  int max_height = height - 2;
  int log_size   = state.log->size();
  int max_lines  = log_size;

  // Ensure lines do not escape the bounds of the buffer
  if(max_lines > max_height)
    max_lines = max_height;


  for(int line = 0; line < max_lines && (line + (int)offset) < log_size; line++)
      mvwprintw(window.get(), line + 1, 0, (*state.log)[offset + line].c_str());
}


void DISPLAY::print_footer() {
  // Paint our footer border
  wattron(window.get(), COLOR_PAIR(2));
  for(int i = 0; i < width; i++)
    mvwaddch(window.get(), height - 1, i, ' ');

  // Add our menu items
  mvwprintw(window.get(), height -1 , 8, "EXIT");
  mvwprintw(window.get(), height -1 , 22, "STEP CPU");
  wattroff(window.get(), COLOR_PAIR(2));

  // Add our key binding
  mvwprintw(window.get(), height -1 , 2, "[X]");
  mvwprintw(window.get(), height -1 , 16, "[S]");
}
