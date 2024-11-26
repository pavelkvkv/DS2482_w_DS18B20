/**** 
 *  wire.c:		
 * Библиотека, которая реализует протокол 1-Wire через I2C-адаптер DS2482.
 * Поддержка опроса и сканирования термодатчиков DS18B20
 * 
 * Автор идеи: bayesiandog
 * Режиссёр: Павел Карягин
 * В ролях: DS2482-100 (и -800), SW_I2C_Lib
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>


#include "wire.h"

#include "sw_i2c.h"

#define TAG "i2c_1wire"
#include "log.h"

// Valid pointer codes for read register selection
#define	StatReg							(0xF0)
#define	DataReg							(0xE1)
#define	ChanReg							(0xD2)
#define	ConfigReg						(0xC3)

// DS2480 commands
#define OWDSConfig                      (0xD2)
#define OWDSDeviceResetCommand	   		(0xF0)
#define	OWDSSetReadPtrCommand			(0xE1)
#define	OWDSChannelSelect				(0xC3)
#define OWDSResetCommand				(0xB4)
#define OWDSReadByteCommand				(0x96)
#define OWDSTriplet						(0x78)
#define OWDSWriteByteCommand			(0xA5)

// 1-wire commands
#define OWSearchCmd                     (0xF0)
#define OWMatchROMCmd                   (0x55)
#define OWConvertTCmd                   (0x44)
#define OWReadPad                       (0xBE)
#define OWWritePad                      (0x4E)
#define OWCopyPad                       (0x48)

// Status register bit assesment
#define STATUS_SBR	(1 << 5)
#define STATUS_TSB	(1 << 6)
#define STATUS_DIR	(1 << 7)
#define STATUS_BUSY (1 << 0)
#define STATUS_RST  (1 << 4)
#define STATUS_PPD  (1 << 1)

#define OWID_MAX_ELEMENTS 16
#define RST_TIMEOUT 5
#define BSY_TIMEOUT 10

static unsigned char ROM_NO[8]  = {0};
//static unsigned char OWID[OWID_MAX_ELEMENTS][8] = {0};

union
{
	u64 ID;
	u8 ID8[8];
} OWIDU[OWID_MAX_ELEMENTS] = {0};

static unsigned char crc8 = 0;
static int LastDiscrepancy = 0;
static int LastFamilyDiscrepancy = 0;
static int LastDeviceFlag = 0;

static sw_i2c_t * i2c_bus = NULL;
static u8 OWDAddress = 0x18 << 1;

static int OWNext();
static int OWFirst();
static u8 OWBusyWait(u8 * stat_register);
static u8 OWResetWait(void);
static bool OWBusy(void);
static bool OWReset(void);

/**
 * Установка дескриптора используемой шины и адреса DS2482
 */
void OWSetBus(sw_i2c_t * bus, u8 deviceaddr)
{
	logI("");
	i2c_bus = bus;
	OWDAddress = deviceaddr;
}

/**
 * Копирует все имеющиеся датчики (ID) в предложенный массив
 * @return количество копируемых датчиков
 */
int OWGetIDs(u64 *IDs, size_t len)
{
	memset(IDs, 0, sizeof(u64) * len);
	for(int i = 0; i < len; i++) 
	{
		IDs[i] = OWIDU[i].ID;
	}

	return len;
}

/**
 * Инициализирует контроллер 1-Wire, сканирует датчики и прописывает в них настройки
 * @return количество найденных датчиков
 */
int OWInit()
{
	logI("");
	DS2480Config DSconfig_default = 
	{
		.APU = 1,
		.SPU = 0,
		.WS = 0
	};
	if(OWDWriteConfig(&DSconfig_default) == 1)
	{
		logE("Failed to write config");
		return 0;
	}

	if(OWResetWait() == 1) 
	{
		logW("Reset bus shows empty line");
		return 0;
	}

	int count = OWDeviceSearch();

	for (int i = 0; i < count; i++)
	{
		DS18B20_init(i);
	}

	return count;
}

// writeOneWireByte
//
// Writes a 1-Wire byte using I2C commands sent to the DS2482.
//
// wrByte - The byte to be written to the 1-Wire.
// returns - Nothing
u8 OWWriteByte(u8 wrByte)
{
	if(OWBusyWait(NULL) == 1) { return 1; }
	SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSWriteByteCommand, &wrByte, 1);
	//if(OWBusyWait(NULL) == 1) { return 1; }

	return 1;
} // OWWriteByte


// readOneWireByte
//
// Reads a 1-Wire byte using I2C commands to the DS2482.
//
// returns the byte read
//
u8 OWReadByte(u8 readRegister)
{
	u8 result;

	if(readRegister != StatReg)
		if(OWBusyWait(NULL) == 1) { return 1; }

	SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSSetReadPtrCommand, &readRegister, 1);
	SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, &result, 1);

	if(readRegister != StatReg)
		if(OWBusyWait(NULL) == 1) { return 1; }

	return result;
} // OWReadByte


/**
 * Сброс шины 1-Wire
 * @return 1 если на шине обнаружен пульс слейва
 */
static bool OWReset(void)
{
	//logI("");
	u8 reset;
	u8 status;	
	u32 start_time = xTaskGetTickCount(); // Получаем текущее время

	//I2C_Read(OWDAddress, OWDSResetCommand, 0, 1, status);	
	SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSResetCommand, 0, 0);

	do {
		SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, &status, 1);
	} while (!(status & STATUS_PPD) && (xTaskGetTickCount() - start_time < RST_TIMEOUT)); // Wait for Presense Pulse to be detected
	do { 
		SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, &status, 1);
	} while ((status & STATUS_BUSY) && (xTaskGetTickCount() - start_time < BSY_TIMEOUT)); // Wait for busy bit to clear, similar to OWBusy()

	reset = (status & STATUS_PPD);	
	//logI("Status: %02X", status);

	return (bool)reset;
}

// DS2480 device reset			
//
// Resets the DS2482.
//
// returns result
//
u8 OWDReset(void)
{
	//logI("");
	//unsigned char buff[1];
	u8 status;	

	// checking if 1-Wire busy
	//if(OWBusyWait() == 1) { return 1; }

	SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSDeviceResetCommand, 0, 0);
	//SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, &status, 1);

	if(OWBusyWait(&status) == 1) { return 1; }

	return status & STATUS_RST;
}

u8 OWDWriteConfig(DS2480Config *pDSconfig) 
{
	//logI("");
  	ASSERT(sizeof(DS2480Config) == 1, "sizeof(DS2480Config) == 1");

	pDSconfig->A͟P͟U͟ = !pDSconfig->APU;
	pDSconfig->S͟P͟U͟ = !pDSconfig->SPU;
	pDSconfig->W͟S͟  = !pDSconfig->WS;
	pDSconfig->res_0  = 0;
	pDSconfig->res_1  = 1;

	// checking if 1-Wire busy
	if(OWBusyWait(NULL) == 1) { return 1; }
	SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSConfig, (u8*)pDSconfig, 1);
	DS2480Config tmp = {0};
	SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, (u8*)&tmp, 1);
	if(tmp.APU == pDSconfig->APU && tmp.SPU == pDSconfig->SPU && tmp.WS == pDSconfig->WS && tmp.res_0 == 0) return 0;
	return 1;
}




/**
 * @brief Проверяет занятость 1-Wire интерфейса.
 *
 * @return 1 если устройство занято, 0 если нет.
 */
static bool OWBusy(void)
{
	u8 buff;
	u8 reg = StatReg;
	SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSSetReadPtrCommand, &reg, 1);
	SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, &buff, 1);

	return (bool)(buff & (STATUS_BUSY));
}

/**
 * @brief Проверяет занятость 1-Wire интерфейса с таймаутом.
 *
 * Функция отправляет команду для проверки статуса устройства, и если устройство занято,
 * ожидает его освобождения с ограничением по времени.
 * @param stat_register - указатель на байт, в который будет записан статус устройства
 *
 * @return 0 если устройство готово, 1 если истек таймаут.
 */
static u8 OWBusyWait(u8 * stat_register)
{
    u32 start_time = xTaskGetTickCount(); // Получаем текущее время

	u8 * stat_register_buff = NULL;
	u8 stat_register_tmp = 0;
	if(stat_register != NULL) stat_register_buff = stat_register;
	else stat_register_buff = &stat_register_tmp;

	bool i2c_ret = true;
	u8 StatReg_addr = StatReg;
	i2c_ret = SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSSetReadPtrCommand, &StatReg_addr, 1);
	if(i2c_ret == false) return 1;

    // Ожидаем, пока устройство занято, или пока не истечет таймаут
    while (1)
    {
		i2c_ret = SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, stat_register_buff, 1);
        // Если устройство не занято, выходим
        if ((*stat_register_buff & (STATUS_BUSY)) == 0 && i2c_ret == true)
        {
			//logI("Busy OK");
            return 0; // Устройство готово
        }
        // Проверяем таймаут
        if ((xTaskGetTickCount() - start_time) > BSY_TIMEOUT)
        {
			logW("Busy Timeout");
            return 1; // Таймаут истек
        }
    }
}

static u8 OWResetWait(void)
{
    u32 start_time = xTaskGetTickCount(); // Получаем текущее время

    // Ожидаем, пока устройство занято, или пока не истечет таймаут
    while (1)
    {
        // 
        if (OWReset() == 1)
        {
			//logI("Reset Bus OK");
            return 0; // Устройство готово
        }
        // Проверяем таймаут
        if ((xTaskGetTickCount() - start_time) > RST_TIMEOUT)
        {
			logW("Reset Bus Timeout");
            return 1; // Таймаут истек
        }
    }
}


int DS18B20_init(int deviceNumber)
{	
	if(deviceNumber >= OWID_MAX_ELEMENTS || OWIDU[deviceNumber].ID < 0) return 0;

	if(OWResetWait() == 1) return 1;
    OWWriteByte(OWMatchROMCmd); // Match ROM
   
    for (int i=7;i>=0;i--) 
	{
        OWWriteByte(OWIDU[deviceNumber].ID8[i]);
    }
	OWWriteByte(OWWritePad); // Write Scratchpad
	DS18B20Config Tconfig = 
	{
		.config.config.ones = 0b11111,
		.config.config.zero = 0,
		.config.config.r0 = 0,
		.config.config.r1 = 0,
		.user_0 = 0xFF, 
		.user_1 = 0xFF
	};
	OWWriteByte(Tconfig.user_0);
	OWWriteByte(Tconfig.user_1);
	OWWriteByte(Tconfig.config.config_byte);

	portENTER_CRITICAL();
	OWWriteByte(OWCopyPad); // Copy Scratchpad
	DS2480Config DSconfig_spu = 
	{
		.APU = 1,
		.SPU = 1,
		.WS = 0
	};
    OWDWriteConfig(&DSconfig_spu); // enable strong pull up and std speed
	portEXIT_CRITICAL();

	vTaskDelay(10);

	return 0;	
}

u16 DS18B20_readTemp(int deviceNumber)
{
	if(deviceNumber >= OWID_MAX_ELEMENTS || OWIDU[deviceNumber].ID < 0) return 0;

    if(OWResetWait() == 1) return 0;
    OWWriteByte(OWMatchROMCmd); // Match ROM
   
    for (int i=7;i>=0;i--) {
        OWWriteByte(OWIDU[deviceNumber].ID8[i]);
    }

	DS2480Config DSconfig = 
	{
		.APU = 1,
		.SPU = 1,
		.WS = 0
	};
    
	portENTER_CRITICAL();
    OWWriteByte(OWConvertTCmd); // Convert T
	OWDWriteConfig(&DSconfig); // SPU enable
	portEXIT_CRITICAL();

    vTaskDelay(100); // Wait for measurement
    if(OWResetWait() == 1) return 0;
    OWWriteByte(OWMatchROMCmd); // Match ROM
   
    for (int i=7;i>=0;i--) {
        OWWriteByte(OWIDU[deviceNumber].ID8[i]);
    }


    OWWriteByte(OWReadPad); // Read Scratchpad
	OWBusyWait(NULL);
    u8 ptr = DataReg;
	u8 ret[2];
    for (int i=0;i<2;i++) {
        SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSReadByteCommand, 0, 0);
        vTaskDelay(2);	  	
        SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSSetReadPtrCommand, &ptr, 1);		// set pointer to read
        //vTaskDelay(10);
        SW_I2C_Read_Noaddr(i2c_bus, OWDAddress, &ret[i], 1);
    }
    u16 temp = (ret[1]<<8 | ret[0]);
	//logI("Got temperature: %.01f", temp*0.0625);

    return (int)(temp*0.0625);

}


//--------------------------------------------------------------------------
// Perform a search for all devices on the 1-Wire network  	  
int OWDeviceSearch()
{
	u8 result = 0;

	if (OWFirst() != 0)
	{
		while (OWNext() != 0);

		// scan array
		for (result = 0; result < OWID_MAX_ELEMENTS; result++)
		{
			if (OWIDU[result].ID == 0) break;
		}
	}
	logI("Found %d devices", result);
	return result;
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire network
// Return 1: device found, ROM number in ROM_NO buffer
//  	  0: no device present
//

int OWFirst()
{
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = 0;
	LastFamilyDiscrepancy = 0;
   
	return OWSearch();
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire network
// Return 1: device found, ROM number in ROM_NO buffer
// 		  0: device not found, end of search
//
int OWNext()
{
  
// leave the search state alone
    return OWSearch();
}

//--------------------------------------------------------------------------
// The 'OWSearch' function does a general search. This function
// continues from the previous search state. The search state
// can be reset by using the 'OWFirst' function.
// This function contains one parameter 'alarm_only'.
// When 'alarm_only' is true (1) the find alarm command
// 0xEC is sent instead of the normal search command 0xF0.
// Using the find alarm command 0xEC will limit the search to only
// 1-Wire devices that are in an 'alarm' state.
//
// Returns: 1: when a 1-Wire device was found and its
// Serial Number placed in the global ROM
//    		0: when no new device was found. Either the
// last search was the last device or there
// are no devices on the 1-Wire Net.
//
int OWSearch()
{
	char buf[200];
	int id_bit_number;
	int last_zero, rom_byte_number, search_result;
	int id_bit, cmp_id_bit;
	unsigned char rom_byte_mask, search_direction, status;
  	static u8 iter = 0;
	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;
	crc8 = 0;
	// if the last call was not the last one
  
	if (!LastDeviceFlag)
		{ 
            if (!OWReset()) { // if reset fails
                // reset the search
                LastDiscrepancy = 0;
                LastDeviceFlag = 0;
                LastFamilyDiscrepancy = 0;
                return 0;
            }
            
		// issue the search command
		OWWriteByte(OWSearchCmd);
		
		// loop to do the search
		do
			{ // if this discrepancy if before the Last Discrepancy
			// on a previous next then pick the same as last time
			if (id_bit_number < LastDiscrepancy)
				{
				if ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0)
					search_direction = 1;
				else
					search_direction = 0;
				}
			else
				{
				// if equal to last pick 1, if not then pick 0
				if (id_bit_number == LastDiscrepancy)
					search_direction = 1;
				else
					search_direction = 0;
				}
			// Perform a triple operation on the DS2482 which will perform
			// 2 read bits and 1 write bit
			status = DS2482_search_triplet(search_direction);
           
			// check bit results in status byte
            id_bit = ((status & STATUS_SBR) == STATUS_SBR);
            cmp_id_bit = ((status & STATUS_TSB) == STATUS_TSB);
            
            search_direction = ((status & STATUS_DIR) == STATUS_DIR) ? 1 : 0;    
                   
			// check for no devices on 1-Wire
			if ((id_bit) && (cmp_id_bit))
				break;
			else
				{
				if ((!id_bit) && (!cmp_id_bit)  && (search_direction == 0))
					{                      
					last_zero = id_bit_number;
					// check for Last discrepancy in family
					if (last_zero < 9)
						LastFamilyDiscrepancy = last_zero;
					}
				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;
				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;
				// if the mask is 0 then go to new SerialNum byte rom_byte_number
				// and reset mask
				if (rom_byte_mask == 0)
					{
					//calc_crc8(ROM_NO[rom_byte_number]); // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
					}
				}            
			}
		while(rom_byte_number < 8); // loop until through all ROM bytes 0-7
		// if the search was successful then
		if (!((id_bit_number < 65)))//if (!((id_bit_number < 65) || (crc8 != 0)))
			{
			// search successful so set LastDiscrepancy,LastDeviceFlag
			// search_result
			LastDiscrepancy = last_zero;           
			// check for last device
			if (LastDiscrepancy == 0)
			    LastDeviceFlag = true;
            search_result = true;
			}
		}
	// if no device found then reset counters so next
	// 'search' will be like a first
	if (!search_result || (ROM_NO[0] == 0))
		{
		LastDiscrepancy = 0;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0;
		search_result = false;       
		}
        int i,j;
    if (search_result) {
        for (i=0, j=7;i<8;i++, j--) {
            OWIDU[iter].ID8[j] = ROM_NO[i]; // save ID ROM in 2D array OWID
        }     
        logI("ID = %"PRIX64, OWIDU[iter].ID);
        

        ++iter;
        if (LastDeviceFlag) {
            logI("LAST DEVICE");   
            iter = 0;
            return 2;
        }
    }
    else {
        logI("NO DEVICES FOUND");
        iter = 0;
    }
	return search_result;
}


//--------------------------------------------------------------------------
// Use the DS2482 help command '1-Wire triplet' to perform one bit of a
//1-Wire search.
//This command does two read bits and one write bit. The write bit
// is either the default direction (all device have same bit) or in case of
// a discrepancy, the 'search_direction' parameter is used.
//
// Returns � The DS2482 status byte result from the triplet command
//
unsigned char DS2482_search_triplet(int search_direction)
{
	unsigned char status;
	int poll_count = 0;
  	char hak[2];
	// 1-Wire Triplet (Case B)
	// S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
	// \--------/
	// Repeat until 1WB bit has changed to 0
	// [] indicates from slave
	// SS indicates byte containing search direction bit value in msbit

	// checking if 1-Wire busy
	if(OWBusyWait(NULL) == 1) { return 1; }
    hak[0] = search_direction ? 0x80 : 0x00;
	//I2C_Write(OWDAddress, OWDSTriplet, 0, 1, hak);
	SW_I2C_Write_8addr(i2c_bus, OWDAddress, OWDSTriplet, 0, 0);
	if(OWBusyWait(&status) == 1) { return 1; }
	//status = OWReadByte(StatReg);
	// return status byte
	return status;
}

int OWAddr2Num(u64 OWID)
{
	for(int i = 0; i < OWID_MAX_ELEMENTS; i++)
	{
		if(OWIDU[i].ID == OWID)
			return i;
	}
	return -1;
}