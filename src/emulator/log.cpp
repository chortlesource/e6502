////////////////////////////////////////////////////////////////////////////
//
// e6502 - log.cpp
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
// Public LOG methods
//

void LOG::log_cpu(CPUSTATE const& state, INSTRUCTION const& i) {
  std::stringstream ls;

  ls  << "|PC:0x" << std::hex << std::setw(4) << unsigned(state.pc)
      << "|0x" << std::hex << std::setw(4) << unsigned(state.opcode)
      << "|"   << get_opcode(i.get_opcode())
      << "|"   << get_admode(i.get_admode())
      << "|A:0x"   << std::hex << std::setw(2) << unsigned(state.a)
      << "|X:0x"   << std::hex << std::setw(2) << unsigned(state.x)
      << "|Y:0x"   << std::hex << std::setw(2) << unsigned(state.y)
      << "|SP:0x"  << std::hex << std::setw(2) << unsigned(state.sp)
      << "|FLAGS:" << std::bitset<8>(state.flags) << "|";
  log.push_back(ls.str());
}


void LOG::log_read(uint16_t const& addr, uint8_t const& val) {

  std::stringstream ls;
    ls  << "| MEMORY READ: 0x" << std::hex << std::setw(4) << unsigned(addr)
        << " | READ: 0x" << std::hex << std::setw(4) << unsigned(val);

  log.push_back(ls.str());
}


void LOG::log_write(uint16_t const& addr, uint8_t const& val) {

  std::stringstream ls;
    ls  << "| MEMORY WRITE: 0x" << std::hex << std::setw(4) << unsigned(addr)
        << " | WROTE: 0x" << std::hex << std::setw(2) << unsigned(val);

  log.push_back(ls.str());
}


void LOG::export_log() {
  if(log.size() < 1) return;

  std::ofstream outfile("./e6502.log", std::ofstream::out | std::ofstream::trunc);
  if(outfile.is_open()) {
    for(const auto &i: log)
      outfile << i << "\n";
  }
}

std::string const& LOG::get_opcode(OPCODE const& op) {
  switch(op) {
  case OPCODE::INV:
    return op_table[0];
  case OPCODE::ADS:
    return op_table[1];
  case OPCODE::AND:
    return op_table[2];
  case OPCODE::ASL:
    return op_table[3];
  case OPCODE::BCC:
    return op_table[4];
  case OPCODE::BCS:
    return op_table[5];
  case OPCODE::BEQ:
    return op_table[6];
  case OPCODE::BIT:
    return op_table[7];
  case OPCODE::BMI:
    return op_table[8];
  case OPCODE::BNE:
    return op_table[9];
  case OPCODE::BPL:
    return op_table[10];
  case OPCODE::BRK:
    return op_table[11];
  case OPCODE::BVC:
    return op_table[12];
  case OPCODE::BVS:
    return op_table[13];
  case OPCODE::CLC:
    return op_table[14];
  case OPCODE::CLD:
    return op_table[15];
  case OPCODE::CLI:
    return op_table[16];
  case OPCODE::CLV:
    return op_table[17];
  case OPCODE::CMP:
    return op_table[18];
  case OPCODE::CPX:
    return op_table[19];
  case OPCODE::CPY:
    return op_table[20];
  case OPCODE::DEC:
    return op_table[21];
  case OPCODE::DEX:
    return op_table[22];
  case OPCODE::DEY:
    return op_table[23];
  case OPCODE::EOR:
    return op_table[24];
  case OPCODE::INC:
    return op_table[25];
  case OPCODE::INX:
    return op_table[26];
  case OPCODE::INY:
    return op_table[27];
  case OPCODE::JMP:
    return op_table[28];
  case OPCODE::JSR:
    return op_table[29];
  case OPCODE::LDA:
    return op_table[30];
  case OPCODE::LDX:
    return op_table[31];
  case OPCODE::LDY:
    return op_table[32];
  case OPCODE::LSR:
    return op_table[33];
  case OPCODE::NOP:
    return op_table[34];
  case OPCODE::ORA:
    return op_table[35];
  case OPCODE::PHA:
    return op_table[36];
  case OPCODE::PHP:
    return op_table[37];
  case OPCODE::PLA:
    return op_table[38];
  case OPCODE::PLP:
    return op_table[39];
  case OPCODE::ROL:
    return op_table[40];
  case OPCODE::ROR:
    return op_table[41];
  case OPCODE::RTI:
    return op_table[42];
  case OPCODE::RTS:
    return op_table[43];
  case OPCODE::SBC:
    return op_table[44];
  case OPCODE::SEC:
    return op_table[45];
  case OPCODE::SED:
    return op_table[46];
  case OPCODE::SEI:
    return op_table[47];
  case OPCODE::STA:
    return op_table[48];
  case OPCODE::STX:
    return op_table[49];
  case OPCODE::STY:
    return op_table[50];
  case OPCODE::TAX:
    return op_table[51];
  case OPCODE::TAY:
    return op_table[52];
  case OPCODE::TSX:
    return op_table[53];
  case OPCODE::TXA:
    return op_table[54];
  case OPCODE::TXS:
    return op_table[55];
  case OPCODE::TYA:
    return op_table[56];
  default:
    return op_table[0];
  }
}


std::string const& LOG::get_admode(ADMODE const& addr) {
  switch(addr) {
    case ADMODE::INV:
      return ad_table[0];
    case ADMODE::ACC:
      return ad_table[1];
    case ADMODE::IMM:
      return ad_table[2];
    case ADMODE::ZER:
      return ad_table[3];
    case ADMODE::ZPX:
      return ad_table[4];
    case ADMODE::ZPY:
      return ad_table[5];
    case ADMODE::REL:
      return ad_table[6];
    case ADMODE::ABS:
      return ad_table[7];
    case ADMODE::ABX:
      return ad_table[8];
    case ADMODE::ABY:
      return ad_table[9];
    case ADMODE::IMP:
      return ad_table[10];
    case ADMODE::IND:
      return ad_table[11];
    case ADMODE::INX:
      return ad_table[12];
    case ADMODE::INY:
      return ad_table[13];
    default:
      return ad_table[0];
  }
}
