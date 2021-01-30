/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 26th 2021
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,

    IDX_CHAR_CO2, // air CO2 characteristic  
    IDX_CHAR_CO2_VAL, // air CO2 value
    IDX_CHAR_CO2_T_CFG, // Time sensoring configuration
    IDX_CHAR_CO2_ENB, // CO2 enable/disable

    IDX_CHAR_TEMP, // temperature characteristic  
    IDX_CHAR_TEMP_VAL, // temperature value
    IDX_CHAR_TEMP_T_CFG, // Time sensoring configuration
    IDX_CHAR_TEMP_ENB, // temperature enable/disable

    IDX_CHAR_CAP, // capacity characteristic
    IDX_CHAR_CAP_VAL, // capacity value
    IDX_CHAR_CAP_D_CFG, // distance sensoring configuration
    IDX_CHAR_CAP_ENB, // capacity enable/disable

    SEN_IDX_NB, // table elements
};

void gatt_server_task(void *pvparameters);
