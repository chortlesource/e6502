////////////////////////////////////////////////////////////////////////////
//
// e6502 - forwards.hpp
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

#ifndef _E6502_FORWARDS_HPP
#define _E6502_FORWARDS_HPP


/////////////////////////////////////////////////////////////
// Forward Declarations
//

class MEMORY;
class CPU;


/////////////////////////////////////////////////////////////
// CPU static variables
//

static const uint8_t FLAG_NEG_MASK = 0x80;
static const uint8_t FLAG_OVR_MASK = 0x40;
static const uint8_t FLAG_CON_MASK = 0x20;
static const uint8_t FLAG_BRK_MASK = 0x10;
static const uint8_t FLAG_DEC_MASK = 0x08;
static const uint8_t FLAG_INT_MASK = 0x04;
static const uint8_t FLAG_ZER_MASK = 0x02;
static const uint8_t FLAG_CAR_MASK = 0x01;

static const uint16_t NMI_LOW     = 0xFFFA;
static const uint16_t NMI_HIGH    = 0xFFFB;
static const uint16_t RESET_LOW   = 0xFFFC;
static const uint16_t RESET_HIGHT = 0xFFFD;
static const uint16_t IRQ_LOW     = 0xFFFE;
static const uint16_t IRQ_HIGH    = 0xFFFF;


/////////////////////////////////////////////////////////////
// CPU inline functions
//

inline bool is_negative_set (uint8_t const& value) { return (value & FLAG_NEG_MASK); };
inline bool is_overflow_set (uint8_t const& value) { return (value & FLAG_OVR_MASK); };
inline bool is_const_set    (uint8_t const& value) { return (value & FLAG_CON_MASK); };
inline bool is_break_set    (uint8_t const& value) { return (value & FLAG_BRK_MASK); };
inline bool is_decimal_set  (uint8_t const& value) { return (value & FLAG_DEC_MASK); };
inline bool is_interrupt_set(uint8_t const& value) { return (value & FLAG_INT_MASK); };
inline bool is_zero_set     (uint8_t const& value) { return (value & FLAG_ZER_MASK); };
inline bool is_carry_set    (uint8_t const& value) { return (value & FLAG_CAR_MASK); };

inline void set_negative (bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_NEG_MASK) : (flags &= (~FLAG_NEG_MASK)); }
inline void set_overflow (bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_OVR_MASK) : (flags &= (~FLAG_OVR_MASK)); }
inline void set_const    (bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_CON_MASK) : (flags &= (~FLAG_CON_MASK)); }
inline void set_break    (bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_BRK_MASK) : (flags &= (~FLAG_BRK_MASK)); }
inline void set_decimal  (bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_DEC_MASK) : (flags &= (~FLAG_DEC_MASK)); }
inline void set_interrupt(bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_INT_MASK) : (flags &= (~FLAG_INT_MASK)); }
inline void set_zero     (bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_ZER_MASK) : (flags &= (~FLAG_ZER_MASK)); }
inline void set_carry    (bool const& value, std::uint8_t& flags) { value ? (flags |= FLAG_CAR_MASK) : (flags &= (~FLAG_CAR_MASK)); }

#endif // _E6502_FORWARDS_HPP
