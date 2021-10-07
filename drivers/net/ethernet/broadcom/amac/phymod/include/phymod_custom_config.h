/*
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

#ifndef __PHYMOD_CUSTOM_CONFIG_H__
#define __PHYMOD_CUSTOM_CONFIG_H__

#ifdef LINUX
#include <linux/string.h>
#include <linux/kernel.h>
#endif


#define PHYMOD_EAGLE_SUPPORT 1
/*#define PHYMOD_FALCON_SUPPORT 0*/
#define PHYMOD_TSCE_SUPPORT 1
/*#define PHYMOD_TSCF_SUPPORT 0*/

#define PHYMOD_SPRINTF(a,b,c,d)
#ifdef LINUX
#define PHYMOD_SNPRINTF (snprintf)
#define PHYMOD_STRCMP (strcmp)
#define PHYMOD_MEMSET (memset)
#define PHYMOD_MEMCPY (memcpy)
#define PHYMOD_STRNCMP (strncmp)
#define PHYMOD_STRNCPY (strncpy)
#define PHYMOD_STRCHR (strchr)
#define PHYMOD_STRSTR (strstr)
#define PHYMOD_STRLEN (strlen)
#define PHYMOD_STRCPY (strcpy)
#define PHYMOD_STRCAT (strcat)
#define PHYMOD_STRNCAT (strncat)
#define PHYMOD_STRTOUL (strtoul)
#else
#define PHYMOD_SNPRINTF snprintf
#define PHYMOD_STRCMP strcmp
#define PHYMOD_MEMSET memset
#define PHYMOD_MEMCPY memcpy
#define PHYMOD_STRNCMP strncmp
#define PHYMOD_STRNCPY strncpy
#define PHYMOD_STRCHR strchr
#define PHYMOD_STRSTR strstr
#define PHYMOD_STRLEN strlen
#define PHYMOD_STRCPY strcpy
#define PHYMOD_STRCAT strcat
#define PHYMOD_STRNCAT strncat
#define PHYMOD_STRTOUL strtoul
#endif


#if defined(LINUX)

#define PHYMOD_CONFIG_DEFINE_UINT8_T 0
#define PHYMOD_CONFIG_DEFINE_UINT16_T 0
#define PHYMOD_CONFIG_DEFINE_UINT32_T 0
#define PHYMOD_CONFIG_DEFINE_SIZE_T 0
#define PHYMOD_CONFIG_DEFINE_UINT64_T 0

typedef u32 uint32;
typedef u16 uint16;
//extern int sprintf (char *,char *, ...);

#else

#if defined (UEFI)
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
#define PHYMOD_CONFIG_DEFINE_SIZE_T 0
#define FUNCTION_NAME() __FUNCTION__
#define __func__  __FUNCTION__
#endif



#ifndef  int32_t
typedef s32 int32_t;
#endif

#ifndef  int8_t
typedef s8 int8_t;
#endif

#ifndef  int16_t
typedef s16 int16_t;
#endif

#ifndef  uint16
typedef u16 uint16;
#endif

#ifndef  uint32
typedef u32 uint32;
#endif

#endif

#ifdef LINUX
#define PHYMOD_DEBUG_ERROR(A)  pr_err A
#else
#define PHYMOD_DEBUG_ERROR(A) phymod_printf A
#endif
#define PHYMOD_DEBUG_VERBOSE(A) phymod_printf A
#define PHYMOD_DIAG_OUT(A)  phymod_printf A

#define PHYMOD_USLEEP(x) phymod_usleep(x)
#define PHYMOD_SLEEP(x) phymod_sleep(x)
#define PHYMOD_MALLOC phymod_malloc
#define PHYMOD_FREE phymod_free

#define PHYMOD_ASSERT(x) 

void phymod_printf(char *s,...);

#endif /* __PHYMOD_CUSTOM_CONFIG_H__ */
