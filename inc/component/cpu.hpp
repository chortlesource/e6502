////////////////////////////////////////////////////////////////////////////
//
// e6502 - cpu.hpp
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

#ifndef _E6502_CPU_HPP
#define _E6502_CPU_HPP


/////////////////////////////////////////////////////////////
// CPUSTATE struct
//
// The CPUSTATE is the container for the current CPU state

struct CPUSTATE {
  bool initialized;
  bool invalid_opcode;

  std::uint8_t  a;        // Accumulator
  std::uint8_t  x;        // Index register
  std::uint8_t  y;        // Index register
  std::uint8_t  sp;       // Stack pointer
  std::uint16_t pc;       // Program counter
  std::uint8_t  flags;    // CPU Status flags
  std::uint8_t  cycles;   // Record the number of cycles

  std::uint16_t addr;    // For use by addressing modes
};


/////////////////////////////////////////////////////////////
// CPU Class
//
// The CPU Class handles CPU logic manipulating the CPUSTATE


class CPU {
public:
  // Public CPU methods
  CPU();

  void nmi();
  void irq();
  void load(std::string const& path, std::uint16_t const& mstart = 0);
  void reset();

  std::uint8_t const& step();

private:
  // Private CPU attributes
  CPUSTATE state;
  MEMORY   memory;

  // Private CPU methods
  void                stack_push(std::uint16_t const& data);
  std::uint8_t const& stack_pop();

  // Addressing Modes
  uint16_t const& addr_mode_acc();   // Accumulator
  uint16_t const& addr_mode_imm();   // Immediate
  uint16_t const& addr_mode_zer();   // Zero Page
  uint16_t const& addr_mode_zpx();   // Zero Page X
  uint16_t const& addr_mode_zpy();   // Zero Page Y
  uint16_t const& addr_mode_rel();   // Relative
  uint16_t const& addr_mode_abs();   // Absolute
  uint16_t const& addr_mode_abx();   // Absolute X
  uint16_t const& addr_mode_aby();   // Absolute Y
  uint16_t const& addr_mode_imp();   // Implicit
  uint16_t const& addr_mode_ind();   // Indirect
  uint16_t const& addr_mode_inx();   // Indexed indirect
  uint16_t const& addr_mode_iny();   // Indirect Indexed

  // Opcode Functions
  void opcode_invalid(uint16_t instr); // To catch unimplmented ops

  void opcode_ads(uint16_t instr);
  void opcode_and(uint16_t instr);
  void opcode_asl(uint16_t instr);
  void opcode_asl_acc(uint16_t instr);

  void opcode_bcc(uint16_t instr);
  void opcode_bcs(uint16_t instr);
  void opcode_beq(uint16_t instr);
  void opcode_bit(uint16_t instr);
  void opcode_bmi(uint16_t instr);
  void opcode_bne(uint16_t instr);
  void opcode_bpl(uint16_t instr);
  void opcode_brk(uint16_t instr);
  void opcode_bvc(uint16_t instr);
  void opcode_bvs(uint16_t instr);

  void opcode_clc(uint16_t instr);
  void opcode_cld(uint16_t instr);
  void opcode_cli(uint16_t instr);
  void opcode_clv(uint16_t instr);
  void opcode_cmp(uint16_t instr);
  void opcode_cpx(uint16_t instr);
  void opcode_cpy(uint16_t instr);

  void opcode_dec(uint16_t instr);
  void opcode_dex(uint16_t instr);
  void opcode_dey(uint16_t instr);

  void opcode_eor(uint16_t instr);

  void opcode_inc(uint16_t instr);
  void opcode_inx(uint16_t instr);
  void opcode_iny(uint16_t instr);

  void opcode_jmp(uint16_t instr);
  void opcode_jsr(uint16_t instr);

  void opcode_lda(uint16_t instr);
  void opcode_ldx(uint16_t instr);
  void opcode_ldy(uint16_t instr);
  void opcode_lsr(uint16_t instr);
  void opcode_lsr_acc(uint16_t instr);


  void opcode_nop(uint16_t instr);

  void opcode_ora(uint16_t instr);

  void opcode_pha(uint16_t instr);
  void opcode_php(uint16_t instr);
  void opcode_pla(uint16_t instr);
  void opcode_plp(uint16_t instr);

  void opcode_rol(uint16_t instr);
  void opcode_rol_acc(uint16_t instr);
  void opcode_ror(uint16_t instr);
  void opcode_ror_acc(uint16_t instr);
  void opcode_rti(uint16_t instr);
  void opcode_rts(uint16_t instr);

  void opcode_sbc(uint16_t instr);
  void opcode_sec(uint16_t instr);
  void opcode_sed(uint16_t instr);
  void opcode_sei(uint16_t instr);
  void opcode_sta(uint16_t instr);
  void opcode_stx(uint16_t instr);
  void opcode_sty(uint16_t instr);

  void opcode_tax(uint16_t instr);
  void opcode_tay(uint16_t instr);
  void opcode_tsx(uint16_t instr);
  void opcode_txa(uint16_t instr);
  void opcode_txs(uint16_t instr);
  void opcode_tya(uint16_t instr);
};


#endif // _E6502_CPU_HPP
