#ifndef _SLAVE_INFO_H_
#define _SLAVE_INFO_H_

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#ifdef __cplusplus
extern "C"{
#endif
char* dtype2string(uint16 dtype);
char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype);
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset);
int si_map_sdo(int slave);
int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);
int si_map_sii(int slave);
void si_sdo(int cnt);
void slaveinfo(char *ifname);
#ifdef __cplusplus
}
#endif
#endif //_SLAVE_INFO_H_
