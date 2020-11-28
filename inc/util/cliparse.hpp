////////////////////////////////////////////////////////////////////////////
//
// e6502 - cliparse.hpp
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

#ifndef _E6502_CLIPARSE_HPP
#define _E6502_CLIPARSE_HPP


/////////////////////////////////////////////////////////////
// CLIPARSE Class
//
// CLIPARSE is a header only class for parsing command line
// input for general purpose use within an application.


class CLIPARSE {
public:
  using ARGLIST   = std::vector<std::string>;
  using size_type = std::size_t;

public:
  // Public CLIPARSE methods

  ///////////////////////////////////////////////////////
  // CLIPARSE()
  //
  // Constructor for an empty container
  //

  CLIPARSE() : initialized(false), args() {};

  ///////////////////////////////////////////////////////
  // CLIPARSE(argc, argv)
  //
  // Constructor that will auto initialize container
  //

  CLIPARSE(int const& argc, const char *argv[]) {
    initialize(argc, argv);
  }

  ///////////////////////////////////////////////////////
  // initialize(argc, argv)
  //
  // Function to initialize an empty container
  //

  void initialize(int const& argc, const char *argv[]) {
    if(initialized) args.clear();

    // Initialize the container
    for(int i = 0; i < argc; i++)
      args.push_back(argv[i]);

    initialized = true;
  }

  ///////////////////////////////////////////////////////
  // at(index)
  //
  // Find the value at position "index"
  //

  std::string at(unsigned int const& index) { return args.at(index); }

  ///////////////////////////////////////////////////////
  // find(value)
  //
  // Check if option "value" is present
  //

  bool find(std::string const& value) {
    if(!initialized) return false;

    auto result = std::find(std::begin(args), std::end(args), value);
    if(result != std::end(args))
      return true;

    return false;
  }

  ///////////////////////////////////////////////////////
  // using_opt(value)
  //
  // Check if the opt "value" is set after indicator "-"
  //

  bool using_opt(std::string const& value) const noexcept {
    if(!initialized) return false;

    // Search for the specified option
    for(auto &it : args) {
      // First find the option indicator
      auto option = it.find("-");
      if(option != std::string::npos) {
        // Next clarify if value is specified
        auto found = it.find(value);
        if(found != std::string::npos)
          return true; // We found our option
      }
    }
    return false;
  }

  ///////////////////////////////////////////////////////
  // using_value(opt)
  //
  // Return the "value" of "opt" set after indicator "="
  //

  std::string using_value(std::string const& opt) const noexcept {
    if(!initialized) return "";

    // Search for the specified option
    for(auto &it : args) {
      // First find the value indicator
      auto option = it.find("=");
      if(option != std::string::npos) {
        // Second clarify it's the option specified
        auto found = it.find(opt);
        if(found != std::string::npos) {
          std::string result = it.substr(found + opt.length());
          return result; // We found our value
        }
      }
    }
    return "";
  }

  ///////////////////////////////////////////////////////
  // using_opt_value(opt)
  //
  // Return the next arg following "opt" set with indicator "-"
  //

  std::string using_opt_value(std::string const& value) {
    if(!initialized) return "";

    // Search for the specified option
    for(unsigned i = 0; i < args.size(); i++) {
      // First find the indicator
      auto option = args[i].find("-");
      if(option != std::string::npos) {
        // Second find our value
        auto found = args[i].find(value);
        if(found != std::string::npos)
          return this->at(++i); // We found our option
      }
    }

    return "";
  }

  ///////////////////////////////////////////////////////
  // Basic get functions
  //


  size_type   size()    { return args.size(); }
  bool const& is_init() { return initialized; }

private:
  // Private CLIPARSE attributes
  bool        initialized;
  ARGLIST     args;
};


#endif // _E6502_CLIPARSE_HPP
