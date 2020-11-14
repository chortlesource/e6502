////////////////////////////////////////////////////////////////////////////
//
// e6502 - timer.hpp
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

#ifndef _E6502_TIMER_HPP
#define _E6502_TIMER_HPP


/////////////////////////////////////////////////////////////
// TIMER Class
//
// TIMER Class is a header only class for handling time within
// the application

class TIMER {

private:
  mutable std::mutex mute;
  double start_time;
  double last_time;
  double delta_time;
  double elapsed_time;
  double ms;

public:
  TIMER() : start_time(0), last_time(0), delta_time(0), elapsed_time(0), ms(0) {};
  ~TIMER() {};

  /////////////////////////////////////////////////////////////
  // start() - Initialize the timer and begin the count

  void start() {
    std::scoped_lock<std::mutex> lock(mute);
    start_time = this->time();
  }

  /////////////////////////////////////////////////////////////
  // update() - Update the timer to calculate delta and elapsed

  void update() {
    std::scoped_lock<std::mutex> lock(mute);
    double new_time = this->time();
    delta_time = new_time - last_time;
    last_time = new_time;

    elapsed_time += delta_time;
  }


  double const& delta() const noexcept {
    std::scoped_lock<std::mutex> lock(mute);
    return delta_time;
  }


  double const& elapsed() const noexcept {
    std::scoped_lock<std::mutex> lock(mute);
    return elapsed_time;
  }


  double const& time() {
    const auto duration = std::chrono::high_resolution_clock::now().time_since_epoch();
    return ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000;
  }

};


#endif // _E6502_TIMER_HPP
