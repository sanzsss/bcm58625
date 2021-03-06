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

#ifdef PHYMOD_EAGLE_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_eagle_diagnostics_driver;
#endif
#ifdef PHYMOD_FALCON_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_falcon_diagnostics_driver;
#endif
#ifdef PHYMOD_QSGMIIE_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_qsgmiie_diagnostics_driver;
#endif
#ifdef PHYMOD_TSCE_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_tsce_diagnostics_driver;
#endif
#ifdef PHYMOD_TSCF_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_tscf_diagnostics_driver;
#endif
#ifdef PHYMOD_PHY8806X_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_phy8806x_diagnostics_driver;
#endif
#ifdef PHYMOD_FURIA_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_furia_diagnostics_driver;
#endif
#ifdef PHYMOD_VIPER_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_viper_diagnostics_driver;
#endif
#ifdef PHYMOD_SESTO_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_sesto_diagnostics_driver;
#endif
#ifdef PHYMOD_QUADRA28_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_quadra28_diagnostics_driver;
#endif
#ifdef PHYMOD_QTCE_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_qtce_diagnostics_driver;
#endif
#ifdef PHYMOD_HURACAN_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_huracan_diagnostics_driver;
#endif
#ifdef PHYMOD_MADURA_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_madura_diagnostics_driver;
#endif
#ifdef PHYMOD_FURIA_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_furia_82212_diagnostics_driver;
#endif
#ifdef PHYMOD_DINO_SUPPORT
extern __phymod_diagnostics__dispatch__t__ phymod_diagnostics_dino_diagnostics_driver;
#endif

__phymod_diagnostics__dispatch__t__* __phymod_diagnostics__dispatch__[phymodDispatchTypeCount] = {
#ifdef PHYMOD_EAGLE_SUPPORT
    &phymod_diagnostics_eagle_diagnostics_driver,
#endif
#ifdef PHYMOD_FALCON_SUPPORT
    &phymod_diagnostics_falcon_diagnostics_driver,
#endif
#ifdef PHYMOD_QSGMIIE_SUPPORT
    &phymod_diagnostics_qsgmiie_diagnostics_driver,
#endif
#ifdef PHYMOD_TSCE_SUPPORT
    &phymod_diagnostics_tsce_diagnostics_driver,
#endif
#ifdef PHYMOD_TSCF_SUPPORT
    &phymod_diagnostics_tscf_diagnostics_driver,
#endif
#ifdef PHYMOD_PHY8806X_SUPPORT
    &phymod_diagnostics_phy8806x_diagnostics_driver,
#endif
#ifdef PHYMOD_FURIA_SUPPORT
    &phymod_diagnostics_furia_diagnostics_driver,
#endif
#ifdef PHYMOD_VIPER_SUPPORT
    &phymod_diagnostics_viper_diagnostics_driver,
#endif
#ifdef PHYMOD_SESTO_SUPPORT
    &phymod_diagnostics_sesto_diagnostics_driver,
#endif
#ifdef PHYMOD_QUADRA28_SUPPORT
    &phymod_diagnostics_quadra28_diagnostics_driver,
#endif
#ifdef PHYMOD_QTCE_SUPPORT
    &phymod_diagnostics_qtce_diagnostics_driver,
#endif
#ifdef PHYMOD_HURACAN_SUPPORT
    &phymod_diagnostics_huracan_diagnostics_driver,
#endif
#ifdef PHYMOD_MADURA_SUPPORT
    &phymod_diagnostics_madura_diagnostics_driver,
#endif
#ifdef PHYMOD_FURIA_SUPPORT
    &phymod_diagnostics_furia_82212_diagnostics_driver,
#endif
#ifdef PHYMOD_DINO_SUPPORT
    &phymod_diagnostics_dino_diagnostics_driver,
#endif

};

int phymod_slicer_position_t_validate(const phymod_slicer_position_t* phymod_slicer_position)
{
        
    if(phymod_slicer_position == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_slicer_position_t_init(phymod_slicer_position_t* phymod_slicer_position)
{

        
    
    if(phymod_slicer_position == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_slicer_position NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_slicer_position, 0, sizeof(phymod_slicer_position_t));

        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_rx_slicer_position_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_slicer_position_t* position)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_slicer_position_t_validate(position)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("position validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_rx_slicer_position_set) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_rx_slicer_position_set(phy, flags, position);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_rx_slicer_position_set isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_rx_slicer_position_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_slicer_position_t* position)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(position == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("position NULL parameter")));
    }
    if(PHYMOD_E_OK != phymod_slicer_position_t_init(position)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("position initialization failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_rx_slicer_position_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_rx_slicer_position_get(phy, flags, position);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_rx_slicer_position_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_rx_slicer_position_max_get(const phymod_phy_access_t* phy, uint32_t flags, const phymod_slicer_position_t* position_min, const phymod_slicer_position_t* position_max)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_slicer_position_t_validate(position_min)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("position_min validation failed")));
    }

    if(PHYMOD_E_OK != phymod_slicer_position_t_validate(position_max)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("position_max validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_rx_slicer_position_max_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_rx_slicer_position_max_get(phy, flags, position_min, position_max);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_rx_slicer_position_max_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_prbs_poly_t_validate(phymod_prbs_poly_t phymod_prbs_poly)
{
        
    if(phymod_prbs_poly >= phymodPrbsPolyCount) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Parameter is out of range")));
    }
        
    return PHYMOD_E_NONE;
    
}

int phymod_prbs_t_validate(const phymod_prbs_t* phymod_prbs)
{
        
    if(phymod_prbs == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }

    if(PHYMOD_E_OK != phymod_prbs_poly_t_validate(phymod_prbs->poly)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("poly validation failed")));
    }

    switch(phymod_prbs->invert) {
        case 0:
        case 1:
            break;
        default:
            PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("invert not allowed value")));
            break;
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_prbs_t_init(phymod_prbs_t* phymod_prbs)
{

        
    
    if(phymod_prbs == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_prbs NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_prbs, 0, sizeof(phymod_prbs_t));
    phymod_prbs->poly = phymodPrbsPolyCount;
    phymod_prbs->invert = 0;

        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_prbs_config_set(const phymod_phy_access_t* phy, uint32_t flags , const phymod_prbs_t* prbs)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_prbs_t_validate(prbs)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("prbs validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_config_set) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_config_set(phy, flags , prbs);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_prbs_config_set isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_prbs_config_get(const phymod_phy_access_t* phy, uint32_t flags , phymod_prbs_t* prbs)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(prbs == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("prbs NULL parameter")));
    }
    if(PHYMOD_E_OK != phymod_prbs_t_init(prbs)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("prbs initialization failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_config_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_config_get(phy, flags , prbs);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_prbs_config_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_prbs_enable_set(const phymod_phy_access_t* phy, uint32_t flags , uint32_t enable)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    switch(enable) {
        case 0:
        case 1:
            break;
        default:
            PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("enable not allowed value")));
            break;
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_enable_set) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_enable_set(phy, flags , enable);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_prbs_enable_set isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_prbs_enable_get(const phymod_phy_access_t* phy, uint32_t flags , uint32_t* enable)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(enable == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("enable NULL parameter")));
    }
    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_enable_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_enable_get(phy, flags , enable);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_prbs_enable_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_prbs_status_t_validate(const phymod_prbs_status_t* phymod_prbs_status)
{
        
    if(phymod_prbs_status == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_prbs_status_t_init(phymod_prbs_status_t* phymod_prbs_status)
{

        
    
    if(phymod_prbs_status == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_prbs_status NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_prbs_status, 0, sizeof(phymod_prbs_status_t));
    phymod_prbs_status->prbs_lock = 0;
    phymod_prbs_status->prbs_lock_loss = 0;
    phymod_prbs_status->error_count = 0;

        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_prbs_status_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_prbs_status_t* prbs_status)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(prbs_status == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("prbs_status NULL parameter")));
    }
    if(PHYMOD_E_OK != phymod_prbs_status_t_init(prbs_status)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("prbs_status initialization failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_status_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_prbs_status_get(phy, flags, prbs_status);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_prbs_status_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_pattern_t_validate(const phymod_pattern_t* phymod_pattern)
{
        
    if(phymod_pattern == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_pattern_t_init(phymod_pattern_t* phymod_pattern)
{

        
    
    if(phymod_pattern == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_pattern NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_pattern, 0, sizeof(phymod_pattern_t));

        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_pattern_config_set(const phymod_phy_access_t* phy, const phymod_pattern_t* pattern)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_pattern_t_validate(pattern)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("pattern validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_config_set) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_config_set(phy, pattern);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_pattern_config_set isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_pattern_config_get(const phymod_phy_access_t* phy, phymod_pattern_t* pattern)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(pattern == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("pattern NULL parameter")));
    }
    if(PHYMOD_E_OK != phymod_pattern_t_validate(pattern)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("pattern validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_config_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_config_get(phy, pattern);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_pattern_config_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_pattern_enable_set(const phymod_phy_access_t* phy, uint32_t enable, const phymod_pattern_t* pattern)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_pattern_t_validate(pattern)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("pattern validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_enable_set) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_enable_set(phy, enable, pattern);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_pattern_enable_set isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_pattern_enable_get(const phymod_phy_access_t* phy, uint32_t* enable)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(enable == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("enable NULL parameter")));
    }
    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_enable_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pattern_enable_get(phy, enable);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_pattern_enable_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_core_diagnostics_t_validate(const phymod_core_diagnostics_t* phymod_core_diagnostics)
{
        
    if(phymod_core_diagnostics == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_core_diagnostics_t_init(phymod_core_diagnostics_t* phymod_core_diagnostics)
{

        
    
    if(phymod_core_diagnostics == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_core_diagnostics NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_core_diagnostics, 0, sizeof(phymod_core_diagnostics_t));
    phymod_core_diagnostics->temperature = 0;
    phymod_core_diagnostics->pll_range = 0;

        
    return PHYMOD_E_NONE;
    
}

int phymod_core_diagnostics_get(const phymod_core_access_t* core, phymod_core_diagnostics_t* diag)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(diag == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("diag NULL parameter")));
    }
    if(PHYMOD_E_OK != phymod_core_diagnostics_t_init(diag)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("diag initialization failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(core,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_core_diagnostics_get) {
        PHYMOD_LOCK_TAKE(core);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_core_diagnostics_get(core, diag);
        PHYMOD_LOCK_GIVE(core);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_core_diagnostics_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_diag_slicer_offset_t_validate(const phymod_diag_slicer_offset_t* phymod_diag_slicer_offset)
{
        
    if(phymod_diag_slicer_offset == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_diag_slicer_offset_t_init(phymod_diag_slicer_offset_t* phymod_diag_slicer_offset)
{

        
    
    if(phymod_diag_slicer_offset == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_diag_slicer_offset NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_diag_slicer_offset, 0, sizeof(phymod_diag_slicer_offset_t));
    phymod_diag_slicer_offset->offset_pe = 0;
    phymod_diag_slicer_offset->offset_ze = 0;
    phymod_diag_slicer_offset->offset_me = 0;
    phymod_diag_slicer_offset->offset_po = 0;
    phymod_diag_slicer_offset->offset_zo = 0;
    phymod_diag_slicer_offset->offset_mo = 0;

        
    return PHYMOD_E_NONE;
    
}

int phymod_diag_eyescan_t_validate(const phymod_diag_eyescan_t* phymod_diag_eyescan)
{
        
    if(phymod_diag_eyescan == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_diag_eyescan_t_init(phymod_diag_eyescan_t* phymod_diag_eyescan)
{

        
    
    if(phymod_diag_eyescan == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_diag_eyescan NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_diag_eyescan, 0, sizeof(phymod_diag_eyescan_t));
    phymod_diag_eyescan->heye_left = 0;
    phymod_diag_eyescan->heye_right = 0;
    phymod_diag_eyescan->veye_upper = 0;
    phymod_diag_eyescan->veye_lower = 0;

        
    return PHYMOD_E_NONE;
    
}

int phymod_pmd_mode_t_validate(phymod_pmd_mode_t phymod_pmd_mode)
{
        
    if(phymod_pmd_mode >= phymodPmdModeCount) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Parameter is out of range")));
    }
        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_diagnostics_t_validate(const phymod_phy_diagnostics_t* phymod_phy_diagnostics)
{
        
    if(phymod_phy_diagnostics == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }

    if(PHYMOD_E_OK != phymod_pmd_mode_t_validate(phymod_phy_diagnostics->pmd_mode)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("pmd_mode validation failed")));
    }

    if(PHYMOD_E_OK != phymod_diag_slicer_offset_t_validate(&phymod_phy_diagnostics->slicer_offset)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("slicer_offset validation failed")));
    }

    if(PHYMOD_E_OK != phymod_diag_eyescan_t_validate(&phymod_phy_diagnostics->eyescan)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("eyescan validation failed")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_diagnostics_t_init(phymod_phy_diagnostics_t* phymod_phy_diagnostics)
{

        
    
    if(phymod_phy_diagnostics == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_phy_diagnostics NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_phy_diagnostics, 0, sizeof(phymod_phy_diagnostics_t));
    phymod_phy_diagnostics->signal_detect = 0;
    phymod_phy_diagnostics->vga_bias_reduced = 0;
    phymod_phy_diagnostics->postc_metric = 0;
    phymod_phy_diagnostics->osr_mode = 0;
    phymod_phy_diagnostics->pmd_mode = 0;
    phymod_phy_diagnostics->rx_lock = 0;
    phymod_phy_diagnostics->rx_ppm = 0;
    phymod_phy_diagnostics->tx_ppm = 0;
    phymod_phy_diagnostics->clk90_offset = 0;
    phymod_phy_diagnostics->clkp1_offset = 0;
    phymod_phy_diagnostics->p1_lvl = 0;
    phymod_phy_diagnostics->m1_lvl = 0;
    phymod_phy_diagnostics->dfe1_dcd = 0;
    phymod_phy_diagnostics->dfe2_dcd = 0;
    phymod_phy_diagnostics->slicer_target = 0;
    if(PHYMOD_E_OK != phymod_diag_slicer_offset_t_init(&phymod_phy_diagnostics->slicer_offset)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("slicer_offset initialization failed")));
    }

    if(PHYMOD_E_OK != phymod_diag_eyescan_t_init(&phymod_phy_diagnostics->eyescan)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("eyescan initialization failed")));
    }

    phymod_phy_diagnostics->state_machine_status = 0;
    phymod_phy_diagnostics->link_time = 0;
    phymod_phy_diagnostics->pf_main = 0;
    phymod_phy_diagnostics->pf_hiz = 0;
    phymod_phy_diagnostics->pf_bst = 0;
    phymod_phy_diagnostics->pf_low = 0;
    phymod_phy_diagnostics->pf2_ctrl = 0;
    phymod_phy_diagnostics->vga = 0;
    phymod_phy_diagnostics->dc_offset = 0;
    phymod_phy_diagnostics->p1_lvl_ctrl = 0;
    phymod_phy_diagnostics->dfe1 = 0;
    phymod_phy_diagnostics->dfe2 = 0;
    phymod_phy_diagnostics->dfe3 = 0;
    phymod_phy_diagnostics->dfe4 = 0;
    phymod_phy_diagnostics->dfe5 = 0;
    phymod_phy_diagnostics->dfe6 = 0;
    phymod_phy_diagnostics->txfir_pre = 0;
    phymod_phy_diagnostics->txfir_main = 0;
    phymod_phy_diagnostics->txfir_post1 = 0;
    phymod_phy_diagnostics->txfir_post2 = 0;
    phymod_phy_diagnostics->txfir_post3 = 0;
    phymod_phy_diagnostics->tx_amp_ctrl = 0;
    phymod_phy_diagnostics->br_pd_en = 0;

        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_diagnostics_get(const phymod_phy_access_t* phy, phymod_phy_diagnostics_t* diag)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(diag == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("diag NULL parameter")));
    }
    if(PHYMOD_E_OK != phymod_phy_diagnostics_t_init(diag)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("diag initialization failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_diagnostics_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_diagnostics_get(phy, diag);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_diagnostics_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_pmd_info_dump(const phymod_phy_access_t* phy, char* type)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pmd_info_dump) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pmd_info_dump(phy, type);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_pmd_info_dump isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_pcs_info_dump(const phymod_phy_access_t* phy, char* type)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pcs_info_dump) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_pcs_info_dump(phy, type);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_pcs_info_dump isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_eyescan_options_t_validate(const phymod_phy_eyescan_options_t* phymod_phy_eyescan_options)
{
        
    if(phymod_phy_eyescan_options == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("NULL parameter")));
    }


        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_eyescan_options_t_init(phymod_phy_eyescan_options_t* phymod_phy_eyescan_options)
{

        
    
    if(phymod_phy_eyescan_options == NULL){
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("phymod_phy_eyescan_options NULL parameter")));
    }
    PHYMOD_MEMSET(phymod_phy_eyescan_options, 0, sizeof(phymod_phy_eyescan_options_t));
    phymod_phy_eyescan_options->linerate_in_khz = 0;
    phymod_phy_eyescan_options->timeout_in_milliseconds = 0;
    phymod_phy_eyescan_options->horz_max = 0;
    phymod_phy_eyescan_options->horz_min = 0;
    phymod_phy_eyescan_options->hstep = 0;
    phymod_phy_eyescan_options->vert_max = 0;
    phymod_phy_eyescan_options->vert_min = 0;
    phymod_phy_eyescan_options->vstep = 0;
    phymod_phy_eyescan_options->ber_proj_scan_mode = 0;
    phymod_phy_eyescan_options->ber_proj_timer_cnt = 0;
    phymod_phy_eyescan_options->ber_proj_err_cnt = 0;
    phymod_phy_eyescan_options->mode = 0;

        
    return PHYMOD_E_NONE;
    
}

int phymod_eyescan_mode_t_validate(phymod_eyescan_mode_t phymod_eyescan_mode)
{
        
    if(phymod_eyescan_mode >= phymodEyescanModeCount) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Parameter is out of range")));
    }
        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_eyescan_run(const phymod_phy_access_t* phy, uint32_t flags, phymod_eyescan_mode_t mode, const phymod_phy_eyescan_options_t* eyescan_options)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_eyescan_mode_t_validate(mode)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("mode validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_eyescan_run) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_eyescan_run(phy, flags, mode, eyescan_options);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_eyescan_run isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_link_monitor_mode_t_validate(phymod_link_monitor_mode_t phymod_link_monitor_mode)
{
        
    if(phymod_link_monitor_mode >= phymodLinkMonCount) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Parameter is out of range")));
    }
        
    return PHYMOD_E_NONE;
    
}

int phymod_phy_link_mon_enable_set(const phymod_phy_access_t* phy, phymod_link_monitor_mode_t link_mon_mode, uint32_t enable)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_link_monitor_mode_t_validate(link_mon_mode)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("link_mon_mode validation failed")));
    }

    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_link_mon_enable_set) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_link_mon_enable_set(phy, link_mon_mode, enable);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_link_mon_enable_set isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_link_mon_enable_get(const phymod_phy_access_t* phy, phymod_link_monitor_mode_t link_mon_mode, uint32_t* enable)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(PHYMOD_E_OK != phymod_link_monitor_mode_t_validate(link_mon_mode)) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("link_mon_mode validation failed")));
    }

    if(enable == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("enable NULL parameter")));
    }
    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_link_mon_enable_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_link_mon_enable_get(phy, link_mon_mode, enable);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_link_mon_enable_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


int phymod_phy_link_mon_status_get(const phymod_phy_access_t* phy, uint32_t* lock_status, uint32_t* lock_lost_lh, uint32_t* error_count)
{

    phymod_dispatch_type_t __type__;
    int __rv__;
        
    
    if(lock_status == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("lock_status NULL parameter")));
    }
    if(lock_lost_lh == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("lock_lost_lh NULL parameter")));
    }
    if(error_count == NULL) {
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("error_count NULL parameter")));
    }
    /* Dispatch */
    PHYMOD_DRIVER_TYPE_GET(phy,&__type__);
    if(__type__ >= phymodDispatchTypeCount) { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_PARAM, (_PHYMOD_MSG("Driver is out of range")));
    }

    if(NULL != __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_link_mon_status_get) {
        PHYMOD_LOCK_TAKE(phy);
        __rv__ = __phymod_diagnostics__dispatch__[__type__]->f_phymod_phy_link_mon_status_get(phy, lock_status, lock_lost_lh, error_count);
        PHYMOD_LOCK_GIVE(phy);
        PHYMOD_IF_ERR_RETURN(__rv__);
    } else { 
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL, (_PHYMOD_MSG("phymod_phy_link_mon_status_get isn't implemented for driver type")));
    }
        
    return PHYMOD_E_NONE;
    
}


