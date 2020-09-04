/* 
 * SOEM EtherCAT exmaple
 * Ported to raspberry pi by Ho Tam - thanhtam.h[at]gmail.com 
 * Edited by Hyundo Kim - tryguns20@gmail.com
 */
 

#ifndef _PDO_DEF_
#define _PDO_DEF_

#include "servo_def.h"

//ELMO PDO mapping

typedef union _mapping_obj{
	uint32_t obj;
	struct {
		uint16_t index;
		uint8_t subindex;
		uint8_t size;
	};
} mapping_obj;

//0x1605 RxPDO
typedef struct PACKED
{
	INT32	TargetPosition;		//0x607A 00 20	
	INT32	TargetVelocity;		//0x60FF 00 20
	INT16	TargetTorque;		//0x6071 00 10
	UINT16	MaxTorque;		//0x6072 00 10
	UINT16	ControlWord;		//0x6040 00 10
	INT8	ModeOfOperation;	//0x6060 00 08

}ELMO_DRIVE_RxPDO_t;

//0x1A03 TxPDO
typedef struct PACKED
{
	INT32	PositionActualValue;		//0x6064 00 20
	UINT32  DigitalInput;                   //0X60FD 00 20 
	INT32	VelocityActualValue;		//0x606C 00 20
	UINT16	StatusWord;	                //0x6041 00 10
        INT16   TorqueActualValue;              //0x6077 00 10
        INT32   AuxiliaryPositionActualValue;   //0x20a0 00 20

}ELMO_DRIVE_TxPDO_t;


typedef struct _ELMO_ServoDrive
{
	ELMO_DRIVE_RxPDO_t 	OutParam;
	ELMO_DRIVE_TxPDO_t 	InParam;
} ELMO_ServoDrive_t;

typedef struct _ELMO_Drive_pt
{
	ELMO_DRIVE_RxPDO_t 	*ptOutParam;
	ELMO_DRIVE_TxPDO_t 	*ptInParam;
} ELMO_Drive_pt;


#endif //_PDO_DEF_

