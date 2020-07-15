#ifndef _JH_STRUCTURE_H
#define _JH_STRUCTURE_H
#include "../osal/osal.h"

#define LAN9252_1       0
#define LAN9252_2       1
typedef struct OUT_PACKED_1
{
    uint32 outvalue1;
    uint32 outvalue2;
    uint32 outvalue3;
    boolean outvalue4;
} out_lan9252_1;

typedef struct OUT_PACKED_2
{
    uint32 outvalue1;
    uint32 outvalue2;
    uint32 outvalue3;
    boolean outvalue4;
} out_lan9252_2;

typedef struct IN_PACKED_1
{
    uint32  invalue1;
    uint32  invalue2;
    uint32  invalue3;
    uint32  invalue4;
    boolean invalue5;
} in_lan9252_1;

typedef struct IN_PACKED_2
{
    uint32  invalue1;
    uint32  invalue2;
    uint32  invalue3;
    uint32  invalue4;
    boolean invalue5;
} in_lan9252_2;

#endif /* _JH_STRUCTURE_H */
