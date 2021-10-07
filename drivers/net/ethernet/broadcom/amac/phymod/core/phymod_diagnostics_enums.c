/*
 *         
 * $Id: phymod.xml,v 1.1.2.5 2013/09/12 10:43:06 nirf Exp $
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


#include <phymod/phymod.h>
#include <phymod/phymod_diagnostics.h>
#include <phymod/phymod_diagnostics_dispatch.h>

#ifdef PHYMOD_DIAG

enum_mapping_t phymod_prbs_poly_t_mapping[] = {
    {"phymodPrbsPoly7", phymodPrbsPoly7},
    {"phymodPrbsPoly9", phymodPrbsPoly9},
    {"phymodPrbsPoly11", phymodPrbsPoly11},
    {"phymodPrbsPoly15", phymodPrbsPoly15},
    {"phymodPrbsPoly23", phymodPrbsPoly23},
    {"phymodPrbsPoly31", phymodPrbsPoly31},
    {"phymodPrbsPoly58", phymodPrbsPoly58},
    {NULL, 0}
};

enum_mapping_t phymod_pmd_mode_t_mapping[] = {
    {"phymodPmdModeOs", phymodPmdModeOs},
    {"phymodPmdModeOsDfe", phymodPmdModeOsDfe},
    {"phymodPmdModeBrDfe", phymodPmdModeBrDfe},
    {NULL, 0}
};

enum_mapping_t phymod_eyescan_mode_t_mapping[] = {
    {"phymodEyescanModeFast", phymodEyescanModeFast},
    {"phymodEyescanModeLowBER", phymodEyescanModeLowBER},
    {"phymodEyescanModeBERProj", phymodEyescanModeBERProj},
    {NULL, 0}
};

enum_mapping_t phymod_link_monitor_mode_t_mapping[] = {
    {"phymodLinkMonPCS49_1x10G", phymodLinkMonPCS49_1x10G},
    {"phymodLinkMonPCS82_4x10G", phymodLinkMonPCS82_4x10G},
    {"phymodLinkMonPCS82_2x25G", phymodLinkMonPCS82_2x25G},
    {"phymodLinkMonPCS82_4x25G", phymodLinkMonPCS82_4x25G},
    {"phymodLinkMonFC4", phymodLinkMonFC4},
    {"phymodLinkMonFC8", phymodLinkMonFC8},
    {"phymodLinkMonFC16", phymodLinkMonFC16},
    {"phymodLinkMonFC32", phymodLinkMonFC32},
    {NULL, 0}
};

#endif /*PHYMOD_DIAG*/
