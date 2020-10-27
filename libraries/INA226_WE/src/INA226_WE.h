/******************************************************************************
 *
 * This is a library for the INA226 Current and Power Sensor Module
 *
 * You'll find several example sketches which should enable you to use the library. 
 *
 * You are free to use it, change it or build on it. In case you like it, it would
 * be cool if you give it a star.
 *
 * If you find bugs, please inform me!
 * 
 * Written by Wolfgang (Wolle) Ewald
 * https://wolles-elektronikkiste.de
 *
 * 
 ******************************************************************************/

#ifndef INA226_WE_H_
#define INA226_WE_H_

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

/* registers */
#define INA226_ADDRESS  		0x40
#define INA226_CONF_REG 		0x00 //Configuration Register
#define INA226_SHUNT_REG    	0x01 //Shunt Voltage Register
#define INA226_BUS_REG    		0x02 //Bus Voltage Register
#define INA226_PWR_REG     		0x03 //Power Register 
#define INA226_CURRENT_REG 		0x04 //Current flowing through Shunt
#define INA226_CAL_REG     		0x05 //Calibration Register 
#define INA226_MASK_EN_REG		0x06 //Mask/Enable Register 
#define INA226_ALERT_LIMIT_REG	0x07 //Alert Limit Register
#define INA226_MAN_ID_REG		0xFE //Contains Unique Manbufacturer Identification Number
#define INA226_ID_REG			0xFF //Contains unique ID

/* parameters, flag bits */
#define INA226_RST            	0x8000 //Reset 
#define INA226_AFF			   	0x0010 //Alert function flag
#define INA226_CVRF				0x0008 //Conversion ready flag
#define INA226_OVF				0x0004 //Overflow flags
#define INA226_ALERT_POL		0x0002 //Alert pin polarity - if set then active-high
//Latch enable - if set then alert flag remains until mask/enable register is read
//if not set then flag is cleared after next conversion within limits
#define INA226_LATCH_EN			0x0001 

typedef enum INA226_AVERAGES{
	AVERAGE_1 		= 0x0000, 
	AVERAGE_4 		= 0x0200,
	AVERAGE_16 		= 0x0400,
	AVERAGE_64		= 0x0600,
	AVERAGE_128		= 0x0800,
	AVERAGE_256		= 0x0A00,
	AVERAGE_512		= 0x0C00,
	AVERAGE_1024	= 0x0E00
} averageMode;

typedef enum INA226_CONV_TIME{ // Conversion time in microseconds
	CONV_TIME_140 	= 0b00000000,
	CONV_TIME_204 	= 0b00000001,
	CONV_TIME_332 	= 0b00000010,
	CONV_TIME_588 	= 0b00000011,
	CONV_TIME_1100 	= 0b00000100,
	CONV_TIME_2116 	= 0b00000101,
	CONV_TIME_4156	= 0b00000110,
	CONV_TIME_8244	= 0b00000111
} convTime;

typedef enum INA226_MEASURE_MODE{
	POWER_DOWN 		= 0b00000000,
	TRIGGERED		= 0b00000011,
	CONTINUOUS		= 0b00000111
} measureMode;

typedef enum INA226_ALERT_TYPE{
	SHUNT_OVER 	  = 0x8000,
	SHUNT_UNDER	  = 0x4000,
	BUS_OVER	  = 0x2000,
	BUS_UNDER	  = 0x1000,
	POWER_OVER	  = 0x0800,
	CURRENT_OVER  = 0xFFFE,
	CURRENT_UNDER = 0xFFFF,
	//CONV_READY	  = 0x0400   not implemented! Use enableConvReadyAlert() 
} alertType;

typedef enum INA226_CURRENT_RANGE{
	MA_400,
	MA_800
} currentRange;

class INA226_WE
{
public:	
	INA226_WE(int addr);
	INA226_WE();			//sets default I2C Address 0x40
  
	void init();
	void reset_INA226();
	void setCorrectionFactor(float corr);
	void setAverage(INA226_AVERAGES averages);
	void setConversionTime(INA226_CONV_TIME convTime);
	void setMeasureMode(INA226_MEASURE_MODE mode);
	void setCurrentRange(INA226_CURRENT_RANGE range);
	void setResistorRange(float resistor, float range);
	float getShuntVoltage_mV();
	float getBusVoltage_V();
	float getCurrent_mA();
	float getBusPower();
	void startSingleMeasurement();
	void powerDown();
	void powerUp(); 
	void waitUntilConversionCompleted();
	void setAlertPinActiveHigh();
	void enableAlertLatch();
	void enableConvReadyAlert();
	void setAlertType(INA226_ALERT_TYPE type, float limit);
	void readAndClearFlags();
	bool overflow;
	bool convAlert;
	bool limitAlert;
	
	
private:
	INA226_AVERAGES deviceAverages;
	INA226_CONV_TIME deviceConvTime;
	INA226_MEASURE_MODE deviceMeasureMode;
	INA226_CURRENT_RANGE deviceCurrentRange;
	INA226_ALERT_TYPE deviceAlertType; 
	int i2cAddress;
	uint16_t calVal;
	uint16_t confRegCopy;
	float currentDivider_mA;
	float pwrMultiplier_mW;
	void writeRegister(uint8_t reg, uint16_t val);
	uint16_t readRegister(uint8_t reg);
};

#endif

