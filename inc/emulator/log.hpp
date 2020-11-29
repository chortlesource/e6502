////////////////////////////////////////////////////////////////////////////
//
// e6502 - log.hpp
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

#ifndef _E6502_LOG_HPP
#define _E6502_LOG_HPP


/////////////////////////////////////////////////////////////
// LOG Class
//
// The LOG Class draws the current cpu status to the screen

class LOG {
public:
  // Public LOG methods
  void log_cpu(CPUSTATE const& state, INSTRUCTION const& i);
  void log_read(uint16_t const& addr, uint8_t const& val);
  void log_write(uint16_t const& addr, uint8_t const& val);
  void export_log();

  std::string&       operator[](unsigned int const& i) { return log[i]; }
  std::string const& operator[](unsigned int const& i) const { return log[i]; }
  std::size_t const size() const { return log.size(); }

private:
  // Private LOG attributes
  std::vector<std::string> log;
  std::vector<std::string> op_table { "INV", "ADS", "AND", "ASL", "BCC", "BCS", "BEQ", "BIT", "BMI", "BNE", "BPL", "BRK", "BVC",
  "BVS", "CLC", "CLD", "CLI", "CLV", "CMP", "CPX", "CPY", "DEC", "DEX", "DEY", "EOR", "INC",
  "INX", "INY", "JMP", "JSR", "LDA", "LDX", "LDY", "LSR", "NOP", "ORA", "PHA", "PHP", "PLA",
  "PLP", "ROL", "ROR", "RTI", "RTS", "SBC", "SEC", "SED", "SEI", "STA", "STX", "STY", "TAX",
  "TAY", "TSX", "TXA", "TXS", "TYA" };

  std::vector<std::string> ad_table {
    "INV", "ACC", "IMM", "ZER", "ZPX", "ZPY", "REL", "ABS", "ABX", "ABY", "IMP",
     "IND", "INX", "INY"
  };

  // Private LOG Methods
  std::string const& get_opcode(OPCODE const& op);
  std::string const& get_admode(ADMODE const& addr);
};


#endif // _E6502_LOG_HPP
