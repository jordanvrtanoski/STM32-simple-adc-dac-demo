/*
 * snmp_agent.h
 *
 *  Created on: Jan 5, 2022
 *      Author: jordanvrtanoski
 *
 *      See example at https://github.com/particle-iot/lwip-contrib/tree/master/examples/snmp
 */

#ifndef INC_SNMP_AGENT_H_
#define INC_SNMP_AGENT_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/apps/snmp.h"
#include "lwip/apps/snmp_mib2.h"
#include "lwip/apps/snmpv3.h"
#include "lwip/apps/snmp_snmpv2_framework.h"
#include "lwip/apps/snmp_snmpv2_usm.h"

typedef struct
{
  u8_t* syscontact;    /*!< Specifies the Contact Person for the system.
                           This parameter can be a value of a string pointer which is null terminated */
  u8_t* syslocation;   /*!< Specifies the Location for the system.
                           This parameter can be a value of a string pointer which is null terminated */
  u8_t* sysdescr;      /*!< Specifies the name/description for the system.
                           This parameter can be a value of a string pointer which is null terminated */
}SNMP_Info;


void SNMP_agent_init(void);

void SNMP_agent_init_with_info(SNMP_Info *info);


#ifdef __cplusplus
}
#endif

#endif /* INC_SNMP_AGENT_H_ */
