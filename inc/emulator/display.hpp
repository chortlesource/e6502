////////////////////////////////////////////////////////////////////////////
//
// e6502 - display.hpp
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

#ifndef _E6502_DISPLAY_HPP
#define _E6502_DISPLAY_HPP


/////////////////////////////////////////////////////////////
// DISPLAY Class
//
// The DISPLAY class renders debug information for the user to
// review


class DISPLAY {
public:
  DISPLAY() : window(nullptr), width(0), height(0), initialized(false) {};
  ~DISPLAY() { finalize(); }

  // Public DISPLAY methods
  void initialize();
  void update(STATE const& state);
  void finalize();

  void key_down(STATE const& state);
  void key_up();

  WINDOW*         get_window() { return window.get(); }
  uint64_t const& get_offset() { return offset; }

private:
  // Private DISPLAY attributes
  std::shared_ptr<WINDOW> window;
  int                     width;
  int                     height;
  bool                    initialized;
  uint64_t                offset;

  // Private DISPLAY methods
  void print_header();
  void print_logger(STATE const& state);
  void print_footer();

};

#endif // _E6502_DISPLAY_HPP
