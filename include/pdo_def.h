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
	INT32	TargetPosition;		//0x607A	
	INT32	TargetVelocity;		//0x60FF
	INT16	TargetTorque;		//0x6071
	UINT16	MaxTorque;			//0x6072
	UINT16	ControlWord;		//0x6040
//	INT32	PositionOffset;		//0x60B0
//	INT32	VelocityOffset;		//0x60B1
//	INT16	TorqueOffset;		//0x60B2
	INT8	ModeOfOperation;	//0x6060
//	UINT32	PhysicalOutput;		//0x60FE
//	UINT16	TouchProbeFunction;	//0x60B8
}ELMO_DRIVE_RxPDO_t;

//0x1A02 TxPDO
typedef struct PACKED
{
	INT32	PositionActualValue;		//0x6064
    //INT32   AuxiliaryPositionActualValue; //0x60E4
	INT32 DigitalInput;
	//INT16	TorqueActualValue;			//0x6077
	//UINT16	StatusWord;					//0x6041
	//INT8	ModeOfOperationDisplay;		//0x6061
	INT32	VelocityActualValue;		//0x606C
	UINT16	StatusWord;					//0x6041
	//INT8	ModeOfOperationDisplay;		//0x6061
        INT32   AuxiliaryPositionActualValue; //0x60E4
//	UINT32	DigitalInput;				//0x60FD
//	UINT16	TouchProbeStatus;			//0x60B9
//	INT32	TouchProbePosition1;		//0x60BA
//	INT32	TouchProbePosition2;		//0x60BB
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

