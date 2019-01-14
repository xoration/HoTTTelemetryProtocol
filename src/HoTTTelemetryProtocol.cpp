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

#include "HoTTTelemetryProtocol.h"
#include "ElapsedMillis.h"

void HoTTTelemetryProtocol::Init(int rxtx)
{
	Init(rxtx, rxtx);
}

void HoTTTelemetryProtocol::Init(int rx, int tx)
{
	memset(receivingBuffer, 0x00, sizeof(receivingBuffer));
	this->hottRx = rx;
	this->hottTx = tx;

	hottSerial = new SoftwareSerial(hottTx, hottRx);
	hottSerial->begin(19200);
}

void HoTTTelemetryProtocol::PollSensorTextMode(HoTTSensor sensor, HoTTCommand command)
{
	EnableTransmitterMode();

	hottSerial->write(0x7F);
	delayMicroseconds(HOTTV4_TX_DELAY);
	hottSerial->write(static_cast<int>(sensor) | static_cast<int>(command));
	EnableReceiverMode();

	ReceiveHoTTTextMode();
}

bool HoTTTelemetryProtocol::PollSensorBinaryMode(HoTTSensor sensor)
{
	EnableTransmitterMode();

	hottSerial->write(0x80);
	delayMicroseconds(HOTTV4_TX_DELAY);
	hottSerial->write(0x80 | (static_cast<int>(sensor) >> 4));
	hottSerial->write(0x80 | (static_cast<int>(sensor) >> 4));
	EnableReceiverMode();

	return ReceiveHoTTBinary();
}

bool HoTTTelemetryProtocol::ReceiveHoTT(int packetSize)
{
	HoTTMode mode = HoTTMode::Unknown;
	bool packetComplete = false;
	int position = 0;
	int timeout = 200;
	receiveTimeout = 0;

	uint8_t buffer[200];
	int bufferPosition = 0;
	int maxAvail = 0;

	while (!packetComplete)
	{

		if (mode == HoTTMode::Text)
		{
			timeout = 3000;
		}

		if (receiveTimeout > timeout)
		{
			return false;
		}

		int avail = hottSerial->available();

		if (!avail)
		{
			continue;
		}

		int bytesReceived = hottSerial->readBytes(buffer, avail);

		if (mode == HoTTMode::Unknown)
		{
			for (int i = 0; i < bytesReceived; i++)
			{
				if (buffer[i] == (char)HoTTMode::Text)
				{
					mode = HoTTMode::Text;
					receiveTimeout = 0;
					bufferPosition = i;
					break;
				}
				else if (buffer[i] == (char)HoTTMode::Binary)
				{
					mode = HoTTMode::Binary;
					receiveTimeout = 0;
					bufferPosition = i;
					break;
				}
			}
		}
		else
		{
			bufferPosition = 0;
		}

		memcpy(receivingBuffer + maxAvail, buffer + bufferPosition, bytesReceived);
		maxAvail += avail;

		if (maxAvail == packetSize)
		{
			packetComplete = true;
		}
	}

	if (!packetComplete)
	{
		return false;
	}

	if (mode == HoTTMode::Binary)
	{
		memcpy(&HoTTPacketBinary, receivingBuffer, sizeof(receivingBuffer));
	}
	else
	{
		memcpy(&HoTTPacketText, receivingBuffer, sizeof(receivingBuffer));
	}

	return true;
}

bool HoTTTelemetryProtocol::ReceiveHoTTBinary()
{
	return ReceiveHoTT(HOTT_PACKET_SIZE_BINARY);
}

void HoTTTelemetryProtocol::ReceiveHoTTTextMode()
{
	ReceiveHoTT(HOTT_PACKET_SIZE_TEXTMODE);
}

inline void HoTTTelemetryProtocol::EnableReceiverMode()
{
	DDRD &= ~(1 << hottTx);
	PORTD |= (1 << hottTx);
}

inline void HoTTTelemetryProtocol::EnableTransmitterMode()
{
	DDRD |= (1 << hottRx);
}