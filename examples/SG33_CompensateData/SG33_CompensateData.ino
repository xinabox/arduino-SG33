/*************************************************************
	This is an examples for the SG33
	Digital Environment Sensor
	
	You can buy one on our store!
	-----> https://xinabox.cc/products/SG33
	
	Supported on the all ☒CHIPs
	
	The sensor communicates over the I2C Bus.
	
*************************************************************/

#include <xCore.h>
#include <xSG33.h>
#include <xSW01.h>

xSW01 SW01;
xSG33 SG33;

const int DELAY_TIME = 1000;

void setup()
{
	// Start the Serial Monitor
	Serial.begin(115200);

	// Start the I2C Comunication
	Wire.begin();

	// Start the  SG33 Sensor
	SG33.begin();

    // Start the SW01 Sensor
    SW01.begin();

	//Delay for sensor to normalise
	delay(2000);

	Serial.println("===============================");
	Serial.println("\tXINABOX SG33 Data      ");
	Serial.println("===============================");
}

void loop()
{
	// Create a variable to store the data read from SG33
	int eCO2, TVOC;

	// Read and calculate data from SG33 sensor
	if (SG33.dataAvailable())
	{
		SG33.getAlgorithmResults();
        SG33.setEnvironmentData(SW01.getHumidity(), SW01.getTempC());
		
        // Read the data from the SG33
		eCO2 = SG33.getCO2();
		TVOC = SG33.getTVOC();

		// Display the recoreded data over the Serial Monitor
		Serial.print("\neCO2: ");
		Serial.println(eCO2);
		Serial.print("TVOC: ");
		Serial.println(TVOC);
	}

	// Small delay between sensor reads
	delay(DELAY_TIME);
}