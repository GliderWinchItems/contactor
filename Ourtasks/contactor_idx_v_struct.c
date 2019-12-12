/******************************************************************************
* File Name          : contactor_idx_v_struct.c
* Date First Issued  : 06/26/2019
* Board              :
* Description        : Load parameter struct 
*******************************************************************************/

#include "contactor_idx_v_struct.h"

/* Select .c file to load parameters */
#ifdef DEHPARAMS
  #include "contactor_idx_v_struct_deh.c"
#else
  #include "contactor_idx_v_struct_gsm.c"
#endif
