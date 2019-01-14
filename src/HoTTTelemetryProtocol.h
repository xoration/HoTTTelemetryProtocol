/*
    Copyright Stefan Seifert 2018-2019

    This file is part of HoTTTelemetryProtocol.

    HoTTTelemetryProtocol is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    HoTTTelemetryProtocol is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with HoTTTelemetryProtocol.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HOTTTELEMETRYPROTOCOL_H
#define HOTTTELEMETRYPROTOCOL_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "ElapsedMillis.h"

#define HOTTV4_TX_DELAY 1000

#define HOTT_BINARY_INTERVAL 200
#define HOTT_TEXT_INTERVAL 800

#define HOTT_PACKET_SIZE_BINARY 45
#define HOTT_PACKET_SIZE_TEXTMODE 173
#define HOTT_ROW 8
#define HOTT_COLUMN 21

/*
S: upper half-byte == Sensor ID:
D GAM (General Air Module) Neu!
E EAM (Electric Air Module) Neu!
A GPS Neu!
9 VARIO Neu!
K: lower half-byte:
Empfaenger: F (keine Bedeutung)
SmartBox: Tastencode (rechtes Touch Pad am Sender, teilweise auch links)
7 ESC Seite Zurueck
D INC Cursor Runter
B DEC Cursor Rauf
E ENTER Seite vorwaerts
9 INC+DEC (SET-Taste am Sender)
Alle anderen Werte haben keine Bedeutung, also auch F vom Rx nicht
*/

enum class HoTTSensor : uint8_t
{
	None = 0x00,
	GAM = 0xD0,
	EAM = 0xE0,
	GPS = 0xA0,
	VARIO = 0x90,
	AIRESC = 0xC0
};

enum class HoTTCommand : uint8_t
{
	NOTHING = 0x0F,
	PREVIOUS_PAGE = 0x07,
	UP = 0x0B,
	DOWN = 0x0D,
	NEXT_PAGE = 0x0E,
	SET = 0x09
};

enum class HoTTMode : uint8_t
{
	Unknown = 0,
	Binary = 0x7C,
	Text = 0x7B,
	EOT = 0x7D
};

class HoTTTelemetryProtocol
{
  public:
	virtual void Init(int rxtx);
	virtual void Init(int rx, int rxtx);

	virtual void PollSensorTextMode(HoTTSensor sensor, HoTTCommand command);
	virtual bool PollSensorBinaryMode(HoTTSensor sensor);

	struct HoTTPacket
	{
		uint8_t startByte; // Byte 0
		uint8_t sensorID;  // Byte 1
		uint8_t alarmTone; // Byte 2
		union {
			struct
			{
				char display[168];
				uint8_t endByte;
				uint8_t checksum;
			} TextMode;
			struct
			{
				uint8_t sensorTextID;
				uint8_t alarmInverse;
				int16_t altitude;
				int16_t maxAltitude;
				int16_t minAltitude;

				uint16_t m1s;
				uint16_t m3s;
				uint16_t m10s;

				uint8_t text[24];
				uint8_t empty;

				uint8_t version;
				uint8_t endByte;
				uint8_t checksum;
			} Vario;

			struct
			{
				uint8_t sensorTextID;			   // Byte 3
				uint8_t Inverse;				   // Byte 4
				uint8_t InverseStatusI;			   // Byte 5
				uint16_t InputVolt;				   // Byte 6
				uint16_t MinInputVolt;			   // Byte 8
				uint16_t Capacity;				   // Byte 10
				uint8_t EscTemperature;			   // Byte 12
				uint8_t MaxEscTemperature;		   // Byte 13
				uint16_t Current;				   // Byte 14
				uint16_t MaxCurrent;			   // Byte 16
				uint16_t RPM;					   // Byte 18
				uint16_t MaxRPM;				   // Byte 20
				uint8_t ThrottlePercent;		   // Byte 22
				uint16_t Speed;					   // Byte 23
				uint16_t MaxSpeed;				   // Byte 25
				uint8_t BECVoltage;				   // Byte 27
				uint8_t MinBECVoltage;			   // Byte 28
				uint8_t BECCurrent;				   // Byte 29
				uint8_t MinBECCurrent;			   // Byte 30
				uint8_t MaxBECCurrent;			   // Byte 31
				uint8_t PWM;					   // Byte 32
				uint8_t BECTemperature;			   // Byte 33
				uint8_t MaxBECTemperature;		   // Byte 34
				uint8_t MotorOrExtTemperature;	 // Byte 35
				uint8_t MaxMotorOrExtTemperature;  // Byte 36
				uint16_t RPMWithoutGearOrExt;	  // Byte 37
				uint8_t Timing;					   // Byte 39
				uint8_t AdvancedTiming;			   // Byte 40
				uint8_t HighestCurrentMotorNumber; // Byte 41
				uint8_t VersionNumber;			   // Byte 42
				uint8_t version;				   /* Byte 43: 00 version number */
				uint8_t endByte;				   /* Byte 44: 0x7D Ende byte */
				uint8_t checksum;				   /* Byte 45: Parity Byte */
			} AirEsc;

			struct
			{
				uint8_t sensorTextID;
				uint8_t alarmInverse;
				uint8_t cell1Low; /* Low Voltage Cell 1-7 in 2mV steps */
				uint8_t cell2Low;
				uint8_t cell3Low;
				uint8_t cell4Low;
				uint8_t cell5Low;
				uint8_t cell6Low;
				uint8_t cell7Low;
				uint8_t cell1High; /* High Voltage Cell 1-7 in 2mV steps */
				uint8_t cell2High;
				uint8_t cell3High;
				uint8_t cell4High;
				uint8_t cell5High;
				uint8_t cell6High;
				uint8_t cell7High;

				uint16_t battery1; /* Battetry 1 LSB/MSB in 100mv steps; 50 == 5V */
				uint16_t battery2; /* Battetry 2 LSB/MSB in 100mv steps; 50 == 5V */

				uint8_t temp1; /* Temp 1; Offset of 20. 20 == 0C */
				uint8_t temp2; /* Temp 2; Offset of 20. 20 == 0C */

				uint16_t altitude; /* Offset -500. 500 == 0 */
				uint16_t current;  /* 1 = 0.1A */
				uint16_t driveVoltage;
				uint16_t capacity; /* mAh */
				uint16_t m2s;	  /* m2s; 0x48 == 0 */
				uint8_t m3s;	   /* m3s; 0x78 == 0 */

				uint16_t rpm; /* RPM. 10er steps; 300 == 3000rpm */
				uint8_t minutes;
				uint8_t seconds;
				uint8_t speed;

				uint8_t version;
				uint8_t endByte;
				uint8_t checksum;
			} ElectricAir;

			struct
			{
				uint8_t sensorTextID;
				uint8_t alarmInverse;
				uint8_t alarmInverse2;   /* Byte 6: 00 inverse status status 1 = kein GPS Signal */
				uint8_t flightDirection; /* Byte 7: 119 = Flightdir./dir. 1 = 2�; 0� (North), 9 0� (East), 180� (South), 270� (West) */
				uint8_t GPSSpeedLow;	 /* Byte 8: 8 = /GPS speed low byte 8km/h */
				uint8_t GPSSpeedHigh;	/* Byte 9: 0 = /GPS speed high byte */

				uint8_t LatitudeNS;		 /* Byte 10: 000 = N = 48�39�988 */
				uint8_t LatitudeMinLow;  /* Byte 11: 231 0xE7 = 0x12E7 = 4839 */
				uint8_t LatitudeMinHigh; /* Byte 12: 018 18 = 0x12 */
				uint8_t LatitudeSecLow;  /* Byte 13: 171 220 = 0xDC = 0x03DC =0988 */
				uint8_t LatitudeSecHigh; /* Byte 14: 016 3 = 0x03 */

				uint8_t longitudeEW;	  /* Byte 15: 000  = E= 9� 25�9360 */
				uint8_t longitudeMinLow;  /* Byte 16: 150 157 = 0x9D = 0x039D = 0925 */
				uint8_t longitudeMinHigh; /* Byte 17: 003 3 = 0x03 */
				uint8_t longitudeSecLow;  /* Byte 18: 056 144 = 0x90 0x2490 = 9360*/
				uint8_t longitudeSecHigh; /* Byte 19: 004 36 = 0x24 */

				uint8_t distanceLow;	 /* Byte 20: 027 123 = /distance low byte 6 = 6 m */
				uint8_t distanceHigh;	/* Byte 21: 036 35 = /distance high byte */
				uint8_t altitudeLow;	 /* Byte 22: 243 244 = /Altitude low byte 500 = 0m */
				uint8_t altitudeHigh;	/* Byte 23: 001 1 = /Altitude high byte */
				uint8_t resolutionLow;   /* Byte 24: 48 = Low Byte m/s resolution 0.01m 30000 = 0.00m/s (1=0.01m/s) */
				uint8_t resolutionHigh;  /* Byte 25: 117 = High Byte m/s resolution 0.01m */
				uint8_t vario3;			 /* Byte 26: 120 = 0m/3s */
				uint8_t GPSNumSat;		 /* Byte 27: GPS.Satelites (number of satelites) (1 byte) */
				uint8_t GPSFixChar;		 /* Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
				uint8_t HomeDirection;   /* Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) */
				uint8_t angleXdirection; /* Byte 30: angle x-direction (1 byte) */
				uint8_t angleYdirection; /* Byte 31: angle y-direction (1 byte) */
				uint8_t angleZdirection; /* Byte 32: angle z-direction (1 byte) */
				uint8_t gyroXLow;		 /* Byte 33: gyro x low byte (2 bytes) */
				uint8_t gyroXHigh;		 /* Byte 34: gyro x high byte */
				uint8_t gyroYLow;		 /* Byte 35: gyro y low byte (2 bytes) */
				uint8_t gyroYHigh;		 /* Byte 36: gyro y high byte */
				uint8_t gyroZLow;		 /* Byte 37: gyro z low byte (2 bytes) */
				uint8_t gyroZHigh;		 /* Byte 38: gyro z high byte */
				uint8_t vibration;		 /* Byte 39: vibration (1 bytes) */
				uint8_t Ascii4;			 /* Byte 40: 00 ASCII Free Character [4] */
				uint8_t Ascii5;			 /* Byte 41: 00 ASCII Free Character [5] */
				uint8_t GPS_fix;		 /* Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX */
				uint8_t version;		 /* Byte 43: 00 version number */
				uint8_t endByte;		 /* Byte 44: 0x7D Ende byte */
				uint8_t checksum;		 /* Byte 45: Parity Byte */
			} GPS;
		};

	} HoTTPacketBinary, HoTTPacketText;

  private:
	virtual void EnableReceiverMode();
	virtual void EnableTransmitterMode();

	virtual bool ReceiveHoTTBinary();
	virtual void ReceiveHoTTTextMode();
	virtual bool ReceiveHoTT(int packetSize);

	int hottRx;
	int hottTx;
	elapsedMillis receiveTimeout;

	SoftwareSerial *hottSerial;
	uint8_t receivingBuffer[HOTT_PACKET_SIZE_TEXTMODE];
};

#endif