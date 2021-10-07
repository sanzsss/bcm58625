/*
 * $Id: $
 * 
 *
 * Copyright © 2016 Broadcom.
 * The term “Broadcom” refers to Broadcom Limited and/or its
 * subsidiaries.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


/*
 * This config file defines all compilation-time specifications for the PHYMOD library.
 * 
 * Reasonable defaults are provided for all configuration options where
 * appropriate.
 * 
 * 
 * You should provide your own configuration options or overrides through
 * a combination of:
 * 
 *      1. The compiler command line, such as -D{OPTION}={VALUE}
 *      2. Create your own custom configuration file:
 *         a) Create a file called 'phymod_custom_config.h'
 *         b) Define all custom settings, using this file as the reference
 *         c) Add -DPHYMOD_INCLUDE_CUSTOM_CONFIG to your compilation
 *         d) Make sure the compilation include path includes 'phymod_custom_config.h'
 * 
 */
#ifndef __PHYMOD_CONFIG_H__
#define __PHYMOD_CONFIG_H__


/*
 * Include custom config file if specified:
 */
#ifdef PHYMOD_INCLUDE_CUSTOM_CONFIG
#include <phymod_custom_config.h>
#endif

/* System definitions for basic C types. */

/* Type uint8_t is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_UINT8_T
#define PHYMOD_CONFIG_DEFINE_UINT8_T            1
#endif

/* Default type definition for uint8_t */
#ifndef PHYMOD_CONFIG_TYPE_UINT8_T
#define PHYMOD_CONFIG_TYPE_UINT8_T              u8
#endif

/* Type uint16_t is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_UINT16_T
#define PHYMOD_CONFIG_DEFINE_UINT16_T           1
#endif

/* Default type definition for uint16_t */
#ifndef PHYMOD_CONFIG_TYPE_UINT16_T
#define PHYMOD_CONFIG_TYPE_UINT16_T             u16
#endif

/* Type uint32_t is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_UINT32_T
#define PHYMOD_CONFIG_DEFINE_UINT32_T           1
#endif

/* Default type definition for uint32_t */
#ifndef PHYMOD_CONFIG_TYPE_UINT32_T
#define PHYMOD_CONFIG_TYPE_UINT32_T             u32
#endif

/* Type uint64_t is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_UINT64_T
#define PHYMOD_CONFIG_DEFINE_UINT64_T           1
#endif

/* Default type definition for uint64_t */
#ifndef PHYMOD_CONFIG_TYPE_UINT64_T
#define PHYMOD_CONFIG_TYPE_UINT64_T             u64
#endif

/* Type size_t is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_SIZE_T
#define PHYMOD_CONFIG_DEFINE_SIZE_T             1
#endif

/* Default type definition for size_t */
#ifndef PHYMOD_CONFIG_TYPE_SIZE_T
#define PHYMOD_CONFIG_TYPE_SIZE_T               unsigned int
#endif

/* Formatting macro PHYMOD_PRIu32 is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_PRIu32
#define PHYMOD_CONFIG_DEFINE_PRIu32             1
#endif

/* Default definition for formatting macro PHYMOD_PRIu32 */
#ifndef PHYMOD_CONFIG_MACRO_PRIu32
#define PHYMOD_CONFIG_MACRO_PRIu32              "u"
#endif

/* Formatting macro PHYMOD_PRIx32 is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_PRIx32
#define PHYMOD_CONFIG_DEFINE_PRIx32             1
#endif

/* Default definition for formatting macro PHYMOD_PRIx32 */
#ifndef PHYMOD_CONFIG_MACRO_PRIx32
#define PHYMOD_CONFIG_MACRO_PRIx32              "x"
#endif

/* Formatting macro PHYMOD_PRIu64 is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_PRIu64
#define PHYMOD_CONFIG_DEFINE_PRIu64             1
#endif

/* Default definition for formatting macro PHYMOD_PRIu64 */
#ifndef PHYMOD_CONFIG_MACRO_PRIu64
#define PHYMOD_CONFIG_MACRO_PRIu64              "llu"
#endif

/* Formatting macro PHYMOD_PRIx64 is not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_PRIx64
#define PHYMOD_CONFIG_DEFINE_PRIx64             1
#endif

/* Default definition for formatting macro PHYMOD_PRIx64 */
#ifndef PHYMOD_CONFIG_MACRO_PRIx64
#define PHYMOD_CONFIG_MACRO_PRIx64              "llx"
#endif

/* Error codes are not provided by the system */
#ifndef PHYMOD_CONFIG_DEFINE_ERROR_CODES
#define PHYMOD_CONFIG_DEFINE_ERROR_CODES        1
#endif

/*
 * Host/Architecture/System definitions for timer operations.
 * These values have no defaults and are required for controlling
 * chip initialization sequences, etc.
 */

/*
 * OPTIONAL Feature definitions.
 * These all have reasonable defaults
 */

/* Include support for printing error messages. */
#ifndef PHYMOD_CONFIG_INCLUDE_ERROR_PRINT
#define PHYMOD_CONFIG_INCLUDE_ERROR_PRINT       1
#endif

/* Include support for printing debug messages. */
#ifndef PHYMOD_CONFIG_INCLUDE_DEBUG_PRINT
#define PHYMOD_CONFIG_INCLUDE_DEBUG_PRINT       1
#endif

/* Include support for printing diagnostics output. */
#ifndef PHYMOD_CONFIG_INCLUDE_DIAG_PRINT
#define PHYMOD_CONFIG_INCLUDE_DIAG_PRINT        1
#endif

/* Include support for floating point in diagnostics functions. */
#ifndef PHYMOD_CONFIG_INCLUDE_FLOATING_POINT
#define PHYMOD_CONFIG_INCLUDE_FLOATING_POINT    0
#endif

/* Include PHY register symbols for use by a debug shell. */
#ifndef PHYMOD_CONFIG_INCLUDE_CHIP_SYMBOLS
#define PHYMOD_CONFIG_INCLUDE_CHIP_SYMBOLS      1
#endif

/*
 * Include register and memory field information for the debug shell.
 * This provides encoding, decoding, and displaying individual field values
 * for each register and memory.
 * Requires more code space than just the chip symbols alone.
 */
#ifndef PHYMOD_CONFIG_INCLUDE_FIELD_INFO
#define PHYMOD_CONFIG_INCLUDE_FIELD_INFO        1
#endif

/*
 * Use symbolic names for all of the fields in a register or memory when
 * encoding or decoding them.
 * This is the most powerful option, but increases the code size a little
 * beyond the basic field information (which deals only with the bit spans
 * of the fields). Definitely enable if you have space.
 */
#ifndef PHYMOD_CONFIG_INCLUDE_FIELD_NAMES
#define PHYMOD_CONFIG_INCLUDE_FIELD_NAMES       1
#endif

/*
 * Include alternative symbol names for registers and memories.
 * Mainly for internal Broadcom use, so you can safely leave this option off.
 */
#ifndef PHYMOD_CONFIG_INCLUDE_ALIAS_NAMES
#define PHYMOD_CONFIG_INCLUDE_ALIAS_NAMES       1
#endif

/*
 * Include reset values for registers and memories.
 * Mainly for internal Broadcom use, so you can safely leave this option off.
 */
#ifndef PHYMOD_CONFIG_INCLUDE_RESET_VALUES
#define PHYMOD_CONFIG_INCLUDE_RESET_VALUES      0
#endif

/* Allow fast firmware downloads by increasing MDIO clock frequency */
#ifndef PHYMOD_CONFIG_MDIO_FAST_LOAD
#define PHYMOD_CONFIG_MDIO_FAST_LOAD            1
#endif

/* Maximum cores per logical port */
#ifndef PHYMOD_CONFIG_MAX_CORES_PER_PORT
#define PHYMOD_CONFIG_MAX_CORES_PER_PORT        3
#endif

/* Maximum lanes per core/chip */
#ifndef PHYMOD_CONFIG_MAX_LANES_PER_CORE
#define PHYMOD_CONFIG_MAX_LANES_PER_CORE        12
#endif

#endif /* __PHYMOD_CONFIG_H__ */

#ifdef CONFIG_OPTION
#ifdef PHYMOD_CONFIG_DEFINE_UINT8_T
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_UINT8_T)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_UINT16_T
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_UINT16_T)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_UINT32_T
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_UINT32_T)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_UINT64_T
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_UINT64_T)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_SIZE_T
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_SIZE_T)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_PRIu32
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_PRIu32)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_PRIx32
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_PRIx32)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_PRIu64
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_PRIu64)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_PRIx64
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_PRIx64)
#endif
#ifdef PHYMOD_CONFIG_DEFINE_ERROR_CODES
CONFIG_OPTION(PHYMOD_CONFIG_DEFINE_ERROR_CODES)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_ERROR_PRINT
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_ERROR_PRINT)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_DEBUG_PRINT
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_DEBUG_PRINT)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_DIAG_PRINT
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_DIAG_PRINT)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_FLOATING_POINT
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_FLOATING_POINT)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_CHIP_SYMBOLS
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_CHIP_SYMBOLS)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_FIELD_INFO
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_FIELD_INFO)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_FIELD_NAMES
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_FIELD_NAMES)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_ALIAS_NAMES
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_ALIAS_NAMES)
#endif
#ifdef PHYMOD_CONFIG_INCLUDE_RESET_VALUES
CONFIG_OPTION(PHYMOD_CONFIG_INCLUDE_RESET_VALUES)
#endif
#ifdef PHYMOD_CONFIG_MDIO_FAST_LOAD
CONFIG_OPTION(PHYMOD_CONFIG_MDIO_FAST_LOAD)
#endif
#ifdef PHYMOD_CONFIG_MAX_CORES_PER_PORT
CONFIG_OPTION(PHYMOD_CONFIG_MAX_CORES_PER_PORT)
#endif
#ifdef PHYMOD_CONFIG_MAX_LANES_PER_CORE
CONFIG_OPTION(PHYMOD_CONFIG_MAX_LANES_PER_CORE)
#endif
#endif /* CONFIG_OPTION */

