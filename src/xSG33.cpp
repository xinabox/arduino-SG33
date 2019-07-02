/*
	This is a library for the SG33 
	Air Quality sensor

	The board uses I2C for communication.
	
	The board communicates with the following I2C device:
	-	CSS811
	
	Data Sheets:
	CSS811 - http://ams.com/eng/content/download/951091/2269479/471718
*/

#include "xSG33.h"

/*--Public Class Function--*/

/********************************************************
 	Constructor
*********************************************************/
xSG33::xSG33(void)
{
	SG33_I2C_ADDR = 0x5A;
}

xSG33::xSG33(uint8_t addr)
{
	SG33_I2C_ADDR = addr;
}

/********************************************************
 	Configure Sensor
*********************************************************/
bool xSG33::begin(void)
{
	uint8_t ID;

	ID = xCore.read8(SG33_I2C_ADDR, CSS811_REG_HW_ID);

	if (ID == CSS811_HW_CODE)
	{
		sw_reset();
		delay(10);
		xCore.write1(SG33_I2C_ADDR, CCS811_BOOTLOADER_APP_START);
		delay(10);

		if (checkForStatusError())
		{
			return false;
		}

		disableInterrupt();
		setDriveMode(CCS811_DRIVE_MODE_1SEC);
		return true;
	}
	else
	{
		return false;
	}
}

/********************************************************
 	Read Data from CCS811 Sensor
*********************************************************/
bool xSG33::getAlgorithmResults(void)
{
	uint8_t buf[8];
	multiRead(CSS811_REG_ALG_RST_DATA, buf, 8);

	_eCO2 = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
	_TVOC = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);

	if (buf[5] & 0x01)
	{
		return false;
	}
	return true;
}

/********************************************************
 	Check is new data is available
*********************************************************/
bool xSG33::dataAvailable(void)
{
	uint8_t status = xCore.read8(SG33_I2C_ADDR, CSS811_REG_STATUS);
	uint8_t ready = (status & 1 << 3);
	if (!ready)
	{
		return false;
	}
	return true;
}
/********************************************************
 	Enable CCS811 Sensor Interrupt
*********************************************************/
void xSG33::enableInterrupt(void)
{
	uint8_t meas_mode = xCore.read8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE);
	meas_mode ^= (-1 ^ meas_mode) & (1 << 3);
	xCore.write8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE, meas_mode);
}

/********************************************************
 	Disable CCS811 Sensor Interrupt
*********************************************************/
void xSG33::disableInterrupt(void)
{
	uint8_t meas_mode = xCore.read8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE);
	meas_mode &= ~(1 << 3);
	xCore.write8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE, meas_mode);
}

/********************************************************
 	Read TVOC from CCS811 Sensor
*********************************************************/
uint16_t xSG33::getTVOC(void)
{
	return _TVOC;
}

/********************************************************
 	Read CO2 from CCS811 Sensor
*********************************************************/
uint16_t xSG33::getCO2(void)
{
	return _eCO2;
}

/********************************************************
 	Set the mode for IAQ measurements
*********************************************************/
void xSG33::setDriveMode(uint8_t mode)
{
	uint8_t meas_mode = xCore.read8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE);
	meas_mode &= 0x0C; // clear old meas_mode settings
	xCore.write8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE, meas_mode | mode);
}

/*--Private Class Function--*/

/********************************************************
 	Perfrom a Software Reset of CCS811
*********************************************************/
void xSG33::sw_reset(void)
{
	Wire.beginTransmission(SG33_I2C_ADDR);
	Wire.write(CSS811_REG_SW_RESET);
	Wire.write(0x11);
	Wire.write(0xE5);
	Wire.write(0x72);
	Wire.write(0x8A);
	Wire.endTransmission();
}

/********************************************************
 	Check if error has occured on CCS811
*********************************************************/
bool xSG33::checkForStatusError(void)
{
	uint8_t error = xCore.read8(SG33_I2C_ADDR, CSS811_REG_STATUS);

	if (error & 0x01)
	{
		return true;
	}
	return false;
}

/********************************************************
 	Retrieve Error Code from CCS811
*********************************************************/
uint8_t xSG33::getErrorCode(void)
{
	uint8_t error_code = xCore.read8(SG33_I2C_ADDR, CSS811_REG_FW_ERROR_ID);
	return error_code;
}

/********************************************************
 	Read/Write Data from CCS811
*********************************************************/
void xSG33::multiRead(uint8_t reg, uint8_t *buf, uint8_t num)
{
	uint8_t value;
	uint8_t pos = 0;

	//on arduino we need to read in 32 byte chunks
	while (pos < num)
	{

		uint8_t read_now = min((uint8_t)32, (uint8_t)(num - pos));
		Wire.beginTransmission(SG33_I2C_ADDR);
		Wire.write((uint8_t)reg + pos);
		Wire.endTransmission();
		Wire.requestFrom(SG33_I2C_ADDR, read_now);

		for (int i = 0; i < read_now; i++)
		{
			buf[pos] = Wire.read();
			pos++;
		}
	}
}

void xSG33::multiWrite(uint8_t reg, uint8_t *buf, uint8_t num)
{
	Wire.beginTransmission(SG33_I2C_ADDR);
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t *)buf, num);
	Wire.endTransmission();
}

/********************************************************
 	Set the environmemtal data
*********************************************************/
void xSG33::setEnvironmentData(float humidity, float tempC)
{
	if ((tempC < -25) || (tempC > 50))
		return;
	if ((humidity > 100) || humidity > 0)
		return;

	uint32_t var1 = humidity * 1000;

	uint32_t var2 = tempC * 1000;
	var2 += 25000;

	uint8_t var3[4];

	var3[0] = (var1 + 250) / 500;
	var3[1] = 0;
	var3[2] = (var2 + 250) / 500;
	var3[3] = 0;
	
	multiWrite(CSS811_REG_ENV_DATA, var3, 4);
}