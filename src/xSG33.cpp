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
sg33v1::sg33v1(void)
{
	SG33_I2C_ADDR = 0x5A;
}

sg33v1::sg33v1(uint8_t addr)
{
	SG33_I2C_ADDR = addr;
}

/********************************************************
 	Configure Sensor
*********************************************************/
bool sg33v1::begin(void)
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
bool sg33v1::getAlgorithmResults(void)
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
bool sg33v1::dataAvailable(void)
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
void sg33v1::enableInterrupt(void)
{
	uint8_t meas_mode = xCore.read8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE);
	meas_mode ^= (-1 ^ meas_mode) & (1 << 3);
	xCore.write8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE, meas_mode);
}

/********************************************************
 	Disable CCS811 Sensor Interrupt
*********************************************************/
void sg33v1::disableInterrupt(void)
{
	uint8_t meas_mode = xCore.read8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE);
	meas_mode &= ~(1 << 3);
	xCore.write8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE, meas_mode);
}

/********************************************************
 	Read TVOC from CCS811 Sensor
*********************************************************/
uint16_t sg33v1::getTVOC(void)
{
	return _TVOC;
}

/********************************************************
 	Read CO2 from CCS811 Sensor
*********************************************************/
uint16_t sg33v1::getCO2(void)
{
	return _eCO2;
}

/********************************************************
 	Set the mode for IAQ measurements
*********************************************************/
void sg33v1::setDriveMode(uint8_t mode)
{
	uint8_t meas_mode = xCore.read8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE);
	meas_mode &= 0x0C; // clear old meas_mode settings
	xCore.write8(SG33_I2C_ADDR, CSS811_REG_MEAS_MODE, meas_mode | mode);
}

/*--Private Class Function--*/

/********************************************************
 	Perfrom a Software Reset of CCS811
*********************************************************/
void sg33v1::sw_reset(void)
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
bool sg33v1::checkForStatusError(void)
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
uint8_t sg33v1::getErrorCode(void)
{
	uint8_t error_code = xCore.read8(SG33_I2C_ADDR, CSS811_REG_FW_ERROR_ID);
	return error_code;
}

/********************************************************
 	Read/Write Data from CCS811
*********************************************************/
void sg33v1::multiRead(uint8_t reg, uint8_t *buf, uint8_t num)
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

void sg33v1::multiWrite(uint8_t reg, uint8_t *buf, uint8_t num)
{
	Wire.beginTransmission(SG33_I2C_ADDR);
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t *)buf, num);
	Wire.endTransmission();
}

/********************************************************
 	Set the environmemtal data
*********************************************************/
void sg33v1::setEnvironmentData(float humidity, float tempC)
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

/*!
 *  @brief  Instantiates a new SGP30 class
 */
sg33v2::sg33v2() {}

/*!
 *  @brief  Setups the hardware and detects a valid SGP30. Initializes I2C
 *          then reads the serialnumber and checks that we are talking to an
 * SGP30
 *  @param  theWire
 *          Optional pointer to I2C interface, otherwise use Wire
 *  @param  initSensor
 *          Optional pointer to prevent IAQinit to be called. Used for Deep
 *          Sleep.
 *  @return True if SGP30 found on I2C, False if something went wrong!
 */
boolean sg33v2::begin(boolean initSensor)
{
	//   if (i2c_dev) {
	//     delete i2c_dev; // remove old interface
	//   }

	//   i2c_dev = new Adafruit_I2CDevice(SGP30_I2CADDR_DEFAULT, theWire);

	//   if (!i2c_dev->begin()) {
	//     return false;
	//   }

	uint8_t command[2];
	command[0] = 0x36;
	command[1] = 0x82;
	if (!readWordFromCommand(command, 2, 10, serialnumber, 3))
		return false;

	uint16_t featureset;
	command[0] = 0x20;
	command[1] = 0x2F;
	if (!readWordFromCommand(command, 2, 10, &featureset, 1))
		return false;
	// Serial.print("Featureset 0x"); Serial.println(featureset, HEX);
	if ((featureset & 0xF0) != SGP30_FEATURESET)
		return false;
	if (initSensor)
	{
		if (!IAQinit())
			return false;
	}

	return true;
}

/*!
 *   @brief Commands the sensor to perform a soft reset using the "General
 * Call" mode. Take note that this is not sensor specific and all devices that
 * support the General Call mode on the on the same I2C bus will perform this.
 *
 *   @return True if command completed successfully, false if something went
 *           wrong!
 */
boolean sg33v2::softReset(void)
{
	uint8_t command[2];
	command[0] = 0x00;
	command[1] = 0x06;
	return readWordFromCommand(command, 2, 10);
}

/*!
 *   @brief  Commands the sensor to begin the IAQ algorithm. Must be called
 * after startup.
 *   @returns True if command completed successfully, false if something went
 *            wrong!
 */
boolean sg33v2::IAQinit(void)
{
	uint8_t command[2];
	command[0] = 0x20;
	command[1] = 0x03;
	return readWordFromCommand(command, 2, 10);
}

/*!
 *  @brief  Commands the sensor to take a single eCO2/VOC measurement. Places
 *          results in {@link TVOC} and {@link eCO2}
 *  @return True if command completed successfully, false if something went
 *          wrong!
 */
boolean sg33v2::IAQmeasure(void)
{
	uint8_t command[2];
	command[0] = 0x20;
	command[1] = 0x08;
	uint16_t reply[2];
	if (!readWordFromCommand(command, 2, 12, reply, 2))
		return false;
	TVOC = reply[1];
	eCO2 = reply[0];
	return true;
}

/*!
 *  @brief  Commands the sensor to take a single H2/ethanol raw measurement.
 * Places results in {@link rawH2} and {@link rawEthanol}
 *  @returns True if command completed successfully, false if something went
 * wrong!
 */
boolean sg33v2::IAQmeasureRaw(void)
{
	uint8_t command[2];
	command[0] = 0x20;
	command[1] = 0x50;
	uint16_t reply[2];
	if (!readWordFromCommand(command, 2, 25, reply, 2))
		return false;
	rawEthanol = reply[1];
	rawH2 = reply[0];
	return true;
}

/*!
 *   @brief  Request baseline calibration values for both CO2 and TVOC IAQ
 *           calculations. Places results in parameter memory locaitons.
 *   @param  eco2_base
 *           A pointer to a uint16_t which we will save the calibration
 *           value to
 *   @param  tvoc_base
 *           A pointer to a uint16_t which we will save the calibration value to
 *   @return True if command completed successfully, false if something went
 *           wrong!
 */
boolean sg33v2::getIAQBaseline(uint16_t *eco2_base,
							   uint16_t *tvoc_base)
{
	uint8_t command[2];
	command[0] = 0x20;
	command[1] = 0x15;
	uint16_t reply[2];
	if (!readWordFromCommand(command, 2, 10, reply, 2))
		return false;
	*eco2_base = reply[0];
	*tvoc_base = reply[1];
	return true;
}

/*!
 *  @brief  Assign baseline calibration values for both CO2 and TVOC IAQ
 *          calculations.
 *  @param  eco2_base
 *          A uint16_t which we will save the calibration value from
 *  @param  tvoc_base
 *          A uint16_t which we will save the calibration value from
 *  @return True if command completed successfully, false if something went
 *          wrong!
 */
boolean sg33v2::setIAQBaseline(uint16_t eco2_base, uint16_t tvoc_base)
{
	uint8_t command[8];
	command[0] = 0x20;
	command[1] = 0x1e;
	command[2] = tvoc_base >> 8;
	command[3] = tvoc_base & 0xFF;
	command[4] = generateCRC(command + 2, 2);
	command[5] = eco2_base >> 8;
	command[6] = eco2_base & 0xFF;
	command[7] = generateCRC(command + 5, 2);

	return readWordFromCommand(command, 8, 10);
}

/*!
 *  @brief  Set the absolute humidity value [mg/m^3] for compensation to
 * increase precision of TVOC and eCO2.
 *  @param  absolute_humidity
 *          A uint32_t [mg/m^3] which we will be used for compensation.
 *          If the absolute humidity is set to zero, humidity compensation
 *          will be disabled.
 *  @return True if command completed successfully, false if something went
 *          wrong!
 */
boolean sg33v2::setHumidity(uint32_t absolute_humidity)
{
	if (absolute_humidity > 256000)
	{
		return false;
	}

	uint16_t ah_scaled =
		(uint16_t)(((uint64_t)absolute_humidity * 256 * 16777) >> 24);
	uint8_t command[5];
	command[0] = 0x20;
	command[1] = 0x61;
	command[2] = ah_scaled >> 8;
	command[3] = ah_scaled & 0xFF;
	command[4] = generateCRC(command + 2, 2);

	return readWordFromCommand(command, 5, 10);
}

/*!
 *  @brief  I2C low level interfacing
 */

bool sg33v2::readWordFromCommand(uint8_t command[],
								 uint8_t commandLength,
								 uint16_t delayms, uint16_t *readdata,
								 uint8_t readlen)
{
	Wire.beginTransmission(SGP30_I2CADDR_DEFAULT);
	for (uint8_t i = 0; i < commandLength; i++)
	{
		Wire.write(command[i]);
	}
	Wire.endTransmission(true);

	delay(delayms);

	if (readlen == 0)
		return true;

	uint8_t replylen = readlen * (SGP30_WORD_LEN + 1);
	uint8_t replybuffer[replylen];
	if (!_read(replybuffer, replylen))
	{
		return false;
	};
		// if (!i2c_dev->read(replybuffer, replylen))
		// {
		// 	return false;
		// }

//#define I2C_DEBUG 1
	for (uint8_t i = 0; i < readlen; i++)
	{
		uint8_t crc = generateCRC(replybuffer + i * 3, 2);
#ifdef I2C_DEBUG
		SerialUSB.print("\t\tCRC calced: 0x");
		SerialUSB.print(crc, HEX);
		SerialUSB.print(" vs. 0x");
		SerialUSB.println(replybuffer[i * 3 + 2], HEX);
#endif
		if (crc != replybuffer[i * 3 + 2])
			return false;
		// success! store it
		readdata[i] = replybuffer[i * 3];
		readdata[i] <<= 8;
		readdata[i] |= replybuffer[i * 3 + 1];
#ifdef I2C_DEBUG
		SerialUSB.print("\t\tRead: 0x");
		SerialUSB.println(readdata[i], HEX);
#endif
	}
	return true;
}

uint8_t sg33v2::generateCRC(uint8_t *data, uint8_t datalen)
{
	// calculates 8-Bit checksum with given polynomial
	uint8_t crc = SGP30_CRC8_INIT;

	for (uint8_t i = 0; i < datalen; i++)
	{
		crc ^= data[i];
		for (uint8_t b = 0; b < 8; b++)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ SGP30_CRC8_POLYNOMIAL;
			else
				crc <<= 1;
		}
	}
	return crc;
}

bool sg33v2::_read(uint8_t *buffer, size_t len)
{
  size_t recv = Wire.requestFrom((uint8_t)SGP30_I2CADDR_DEFAULT, len, true); //remove 2nd cast, add 1st cast
  if (recv != len)
  {
  	return false;
  }
  for (uint8_t i = 0; i < len; i++)
  {
    buffer[i] = Wire.read();
  }
  return true;
}

xSG33::xSG33(void)
{
	//SG33_I2C_ADDR = 0x5A;
}

xSG33::xSG33(uint8_t addr)
{
	//SG33_I2C_ADDR = addr;
}

bool xSG33::begin(void)
{
	if (xCore.ping(0x58))
	{
		version = 2;
	}
	else
	{
		version = 1;
	}

	if (version == 1)
		return v1.begin();
	if (version == 2)
		return v2.begin(true);
		
	return false; //should never be reached
}

bool xSG33::getAlgorithmResults(void)
{
	if (version == 1)
		return v1.getAlgorithmResults();
	if (version == 2)
		return v2.IAQmeasure();
}

bool xSG33::dataAvailable(void)
{
	if (version == 1)
		return v1.dataAvailable();
	return true;
}

uint16_t xSG33::getTVOC(void)
{
	if (version == 1)
		return v1.getTVOC();
	if (version == 2)
		return v2.TVOC;
}

uint16_t xSG33::getCO2(void)
{
	if (version == 1)
		return v1.getCO2();
	if (version == 2)
		return v2.eCO2;
}

void xSG33::setEnvironmentData(float humidity, float tempC)
{
	if (version == 1)
		v1.setEnvironmentData(humidity, tempC);
	if (version == 2)
		v2.setHumidity(getAbsoluteHumidity(tempC, humidity));
}

uint32_t xSG33::getAbsoluteHumidity(float temperature, float humidity)
{
	// approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
	const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
	const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);																  // [mg/m^3]
	return absoluteHumidityScaled;
}
