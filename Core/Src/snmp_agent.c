/*
 * snmp_agent.c
 *
 *  Created on: Jan 5, 2022
 *      Author: jordanvrtanoski
 */

#include "snmp_agent.h"

/**
 * @brief Initialize SNMP agent
 */
void SNMP_agent_init(void)
{
  snmp_mib2_set_syscontact_readonly((const u8_t*)"root", NULL);
  snmp_mib2_set_syslocation_readonly((const u8_t*)"lwIP development PC", NULL);
  snmp_mib2_set_sysdescr((const u8_t*)"simhost", NULL);
  snmp_init();
}

/**
 * @brief Initialize SNMP agent with extra information
 */
void SNMP_agent_init_with_info(SNMP_Info *info)
{
  snmp_mib2_set_syscontact_readonly(info->syscontact, NULL);
  snmp_mib2_set_syslocation_readonly(info->syslocation, NULL);
  snmp_mib2_set_sysdescr(info->sysdescr, NULL);
  snmp_init();
}

