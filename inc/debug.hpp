////////////////////////////////////////////////////////////////////////////
//
// atg - debug.hpp
//
// Copyright (c) 2020 Christopher M. Short
//
// This file is part of atg.
//
// atg is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// atg is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with atg. If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////

#ifndef _DEBUG_HPP
#define _DEBUG_HPP


namespace ATG {

/////////////////////////////////////////////////////////////////
// DEBUG Functions
//
// This is a header only file containing general debug macros for
// use within an application. All macro's operate within a debug
// build. But only errors are reported in other builds.


/////////////////////////////////////////////////////////////////
// ANSI escape constants
//

static const std::string colour_error   { "\033[1;31m" };
static const std::string colour_info    { "\033[1;36m" };
static const std::string colour_warn    { "\033[1;35m" };
static const std::string colour_default { "\033[1;90m" };
static const std::string colour_reset   { "\033[0m" };


/////////////////////////////////////////////////////////////////
// Enumeration
//

enum class LOG_TYPE { DEBUG, INFO, WARN, ERROR };


/////////////////////////////////////////////////////////////////
// Base write_debug function
//

template<typename TType>
void write_debug(std::ostream& out, TType const& t) {
    out << t;
}


/////////////////////////////////////////////////////////////////
// Base DEBUG_LOG struct
//

struct DEBUG_LOG {
  std::ostream& out;
  std::mutex mute;

  DEBUG_LOG(std::ostream& o, const LOG_TYPE& t, const char *f, int l) : out(o) {
    // Obtain and print current time stamp
    auto time = std::time(nullptr);
    auto tm = *std::localtime(&time);

    mute.lock();

    out << colour_default << std::put_time(&tm, "[ %d-%m-%Y %H-%M-%S ]");

    switch(t) {
      case LOG_TYPE::DEBUG:
        std::cout << colour_default << " [DEBUG]\t" << colour_default << f << ":" << l << ": " << colour_reset;
        break;
      case LOG_TYPE::INFO:
        std::cout << colour_info << " [INFO]\t" << colour_default << f << ":" << l << ": " << colour_reset;
        break;
      case LOG_TYPE::WARN:
        std::cout << colour_warn << " [WARN]\t" << colour_default << f << ":" << l << ": " << colour_reset;
        break;
      case LOG_TYPE::ERROR:
        std::cout << colour_error << " [ERROR]\t" << colour_default << f << ":" << l << ": " << colour_reset;
        break;
      default:
        break;
    };
  }


  ~DEBUG_LOG() {
    out << std::endl;
    mute.unlock();
  }

  /////////////////////////////////////////////////////////////////
  // Recursive write functions
  //

  template<typename TType, typename ... RType>
  void write(TType const& f, RType const& ... args) {
      write_debug(out, f);
      out << " ";
      write(args...);
  }

  template<typename TType>
  void write(TType const& f) {
      write_debug(out, f);
  }

  void write() {
      // Break recursion...
  }
};


/////////////////////////////////////////////////////////////////
// DEBUG Macro's
//

#ifdef DEBUG_BUILD
#define DEBUG(...) ATG::DEBUG_LOG(std::cout, ATG::LOG_TYPE::DEBUG, __FILE__, __LINE__).write(__VA_ARGS__)
#define INFO(...) ATG::DEBUG_LOG(std::cout, ATG::LOG_TYPE::INFO, __FILE__, __LINE__).write(__VA_ARGS__)
#define WARN(...) ATG::DEBUG_LOG(std::cout, ATG::LOG_TYPE::WARN, __FILE__, __LINE__).write(__VA_ARGS__)
#define ERROR(...) ATG::DEBUG_LOG(std::cout, ATG::LOG_TYPE::ERROR, __FILE__, __LINE__).write(__VA_ARGS__)
#else
// #define DEBUG(msg, ...) do {} while(0)
#define DEBUG(...) do {} while(0)
#define INFO(...) do {} while(0)
#define WARN(...) do {} while(0)
#define ERROR(...) ATG::DEBUG_LOG(std::cout, ATG::LOG_TYPE::ERROR, __FILE__, __LINE__).write(__VA_ARGS__)
#endif

};


#endif // _WAEYN_LOG_HPP
