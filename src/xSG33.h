/*
	This is a library for the SG33 
	Air Quality sensor

	The board uses I2C for communication.
	
	The board communicates with the following I2C device:
	-	CSS811
	
	Data Sheets:
	CSS811 - http://ams.com/eng/content/download/951091/2269479/471718
*/

#ifndef xSG33_h
#define xSG33_h

// System Includes
#include <inttypes.h>
#include <Arduino.h>
#include "xCore.h"
#include "math.h"

// Register Defines
#define CSS811_REG_STATUS 0x00
#define CSS811_REG_MEAS_MODE 0x01
#define CSS811_REG_ALG_RST_DATA 0X02
#define CSS811_REG_RAW_DATA 0x03
#define CSS811_REG_ENV_DATA 0x05
#define CSS811_REG_THRESHOLDS 0x10
#define CSS811_REG_BASELINE 0x11
#define CSS811_REG_HW_VERSION 0x21
#define CSS811_REG_FW_BOOT_V 0x23
#define CSS811_REG_FW_APP_V 0x24
#define CSS811_REG_FW_ERROR_ID 0xE0
#define CSS811_REG_SW_RESET 0xFF
#define CSS811_DATA_READY 0x08

// Device ID
#define CSS811_REG_HW_ID 0x20
#define CSS811_HW_CODE 0x81

// Bootloader Registers
#define CCS811_BOOTLOADER_APP_ERASE 0xF1
#define CCS811_BOOTLOADER_APP_DATA 0xF2
#define CCS811_BOOTLOADER_APP_VERIFY 0xF3
#define CCS811_BOOTLOADER_APP_START 0xF4

// Drive Modes
#define CCS811_DRIVE_MODE_IDLE 0x00
#define CCS811_DRIVE_MODE_1SEC 0x10
#define CCS811_DRIVE_MODE_10SEC 0x20
#define CCS811_DRIVE_MODE_60SEC 0x30
#define CCS811_DRIVE_MODE_250MS 0x40

#define BYTE_TO_BINARY(byte)       \
	(byte & 0x80 ? '1' : '0'),     \
		(byte & 0x40 ? '1' : '0'), \
		(byte & 0x20 ? '1' : '0'), \
		(byte & 0x10 ? '1' : '0'), \
		(byte & 0x08 ? '1' : '0'), \
		(byte & 0x04 ? '1' : '0'), \
		(byte & 0x02 ? '1' : '0'), \
		(byte & 0x01 ? '1' : '0')

/*=========================================================================*/

class xSG33 : public xCoreClass
{
  public:
	/*
		* Constructor
		* Creates a new instance of Sensor class.
		*/
	xSG33();
	xSG33(uint8_t addr);

	/*
		* Runs the setup of the sensor. 
		* Call this in setup(), before reading any sensor data.
		*
		* @return true if setup was successful.
		*/
	bool begin();

	/*
		* Used to get the TVOC value in ppb.
		* Call this in loop(). Used to get sensor temperature.
		*
		* @return TVOC, returns float value of TVOC.
		*/
	uint16_t getTVOC(void);

	/*
		* Used to get the eC02 value in ppm.
		* Call this in loop(). Used to get sensor temperature.
		*
		* @return C02, returns float value of C02.
		*/
	uint16_t getCO2(void);

	/*
		* Used to get the temperature value in degress farenhied.
		* Call this in loop(). Used to get sensor temperature.
		*
		* @input humidity, the ambient humidity.
		* @input tempC, the ambient temperature in degress C.
		*
		* @return none
		*/
	void setEnvironmentData(float humidity, float tempC);

	void enableInterrupt();
	void disableInterrupt();

	uint8_t getErrorCode(void);

	void setDriveMode(uint8_t mode);

	/*
		* Check for the availability of new data
		*
		*@return boolean, true for data available
		*/
	bool dataAvailable(void);

	bool getAlgorithmResults(void);

  private:
	/*
		* Check if any errors have occured
		*
		* @return error number
		*/
	bool checkForStatusError(void);

	void sw_reset(void);

	void multiRead(uint8_t reg, uint8_t *buf, uint8_t num);
	void multiWrite(uint8_t reg, uint8_t *buf, uint8_t num);

	uint8_t SG33_I2C_ADDR;
	uint16_t _eCO2, _TVOC;
};

#endif
