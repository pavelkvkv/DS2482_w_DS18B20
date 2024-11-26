// wire.h			1-wire related definitions and function prototypes
#ifndef __WIRE_H
#define __WIRE_H

#include "my_types.h"
#include "sw_i2c.h"





// DS2480 config structure
typedef struct __attribute__((packed, aligned(1))) 
{
    bool APU    : 1; // Active pull up
    bool res_0  : 1; // always 0
    bool SPU    : 1; // Strong pull up
    bool WS     : 1; // Wire speed (0 - std, 1 - fast)
    bool A͟P͟U͟    : 1; // Active pull up (inverted)
    bool res_1  : 1; // always 1
    bool S͟P͟U͟    : 1; // Strong pull up (inverted)
    bool W͟S͟     : 1; // Wire speed (inverted)
} DS2480Config;

// DS18B20 config structure
typedef struct __attribute__((packed, aligned(1))) 
{
    u8 user_0   ;    // user byte
    u8 user_1   ;    // user byte
    union 
    {
    u8 config_byte;    
    struct              // config byte  00 - 100 ms, 01 - 200 ms, 10 - 400 ms, 11 - 800 ms
    {
    u8 ones     : 5;    // always 1
    bool r0     : 1;    // cfg resolution (delay)
    bool r1     : 1;    // cfg resolution (delay)
    bool zero   : 1;    // always 0
    }config;
    }config;
} DS18B20Config;




// Functions
int OWInit();
void OWSetBus(sw_i2c_t * bus, u8 deviceaddr);
int OWGetIDs(u64 *IDs, size_t len);
u8 OWWriteByte(u8 wrByte);
u8 OWReadByte(u8 rdByte);

u8 OWDReset(void);
u8 OWDWriteConfig(DS2480Config *pDSconfig);

u16 DS18B20_readTemp(int deviceNumber);
int DS18B20_init(int deviceNumber);
int OWDeviceSearch();
int OWSearch(void);
unsigned char DS2482_search_triplet(int search_direction);
int OWAddr2Num(u64 OWID);

#endif
