/*  PSXLIB Controller Decoder Simple & Compact Library (PsxLib.cpp)
	To Use with PS1 and PS2 GamePad controllers with Arduino & c., Esp8266, Esp32 etc.
	
	Original written by: Kevin Ahrendt June 22nd, 2008
	Modified by Giuseppe Porcheddu November 5ft, 2018 to be used with Espressif ESP micro series
	Following also libs of Bill Porter and Kompanets Konstantin (aka I2M)

	*  Connections: Example with Digital Pins 2, 4, 0, 15 and Ground
	*  GamePad Cable connected to micro by five pins. 
	*  Green   		ACK  -----------------|
	*  Blue    		Clock   ---- 2     Res 10K
	*  Yellow  		Attn    ---- 4        |
	*  Red     		VCC+ ------------------
	*  Black   		Ground  ---- Ground   |
	*  White   		Vibration    NC       |
	*  Orange  		Command ---- 0     Res 10K
	*  Brown   		Data    ---- 15 ------|
	*  Note: VCC Psx side connected with two R10K with Clock and Data Pins.
	
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "PsxLib.h"

static byte enter_config[]={0x01,0x43,0x00,0x01,0x00};
static byte set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
static byte set_bytes_large[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
static byte exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
static byte enable_rumble[]={0x01,0x4D,0x00,0x00,0x01};
static byte type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};

Psx::Psx() { };	// Class Psx

byte Psx::shift(byte _dataOut)	{	// Does the actual shifting, both in and out simultaneously
	_dataIn = 0;

	for (byte i = 0; i < 8; i++)	{
		
		if ( _dataOut & (1 << i) ) 
			digitalWrite(_cmndPin, HIGH);	// Writes out the _dataOut bits
		else 
			digitalWrite(_cmndPin, LOW);

		digitalWrite(_clockPin, LOW);
		delayMicroseconds(_delay);

		if ( digitalRead(_dataPin) ) {	// Reads the data pin
			_dataIn |= _BV(i);			// Giuseppe Shift the read into _dataIn (align to the others libs)
		}
		
		digitalWrite(_clockPin, HIGH);
		delayMicroseconds(_delay);
	}
	return _dataIn;
}


void Psx::setupPins(byte dataPin, byte cmndPin, byte attPin, byte clockPin, byte delay) {
	pinMode(dataPin, INPUT_PULLUP);
	//digitalWrite(dataPin, HIGH);	// Turn on internal pull-up
	pinMode(attPin, OUTPUT);
	digitalWrite(_attPin, HIGH);
	pinMode(cmndPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	digitalWrite(_clockPin, HIGH);

	_dataPin = dataPin;
	_cmndPin = cmndPin;
	_attPin = attPin;
	_clockPin = clockPin;
	_delay = delay;

  delayMicroseconds(1000000);

  delayMicroseconds(1000000);
  
  
  
  do
  {
    sendCommandString(enter_config, sizeof(enter_config)); //start config run
    sendCommandString(set_mode, sizeof(set_mode));
    sendCommandString(exit_config, sizeof(exit_config));
  
    read();
    Serial.println(_mode, HEX); // Shoujld be -x73
  }
  while(_mode & 0xf0 != 0x70);
}


uint32_t Psx::read() {
	digitalWrite(_attPin, LOW);

	shift(0x01);
	_mode = shift(0x42);
	shift(0xFF);

	_data1 = ~shift(0xFF);
	_data2 = ~shift(0xFF);
  byte _data3 = shift(0xFF);
  byte _data4 = shift(0xFF);
  byte _data5 = shift(0xFF);
  byte _data6 = shift(0xFF);

	digitalWrite(_attPin, HIGH);
	
	_last_buttons = _buttonsData;	// Giuseppe: Save Old Value

	_buttonsData = (_data2 << 8) | _data1;
  _sticksData = (_data6 << 24) | (_data5 << 16) | (_data4 << 8) | _data3;

	return _buttonsData;
}

bool Psx::button(unsigned int button) {
  return ((_buttonsData & button) > 0);
}

bool Psx::newButtonState() {
  return ((_last_buttons ^ _buttonsData) > 0);
}

bool Psx::buttonNewState(unsigned int button) {
  return (((_last_buttons ^ _buttonsData) & button) > 0);
}

byte Psx::analog(byte button) {
   return (uint8_t) (_sticksData >> ((button)*8));
}

void Psx::sendCommandString(byte string[], byte len) {

  digitalWrite(_attPin, LOW);
  
  for (int y=0; y < len; y++)
  {
    shift(string[y]);
  } 
  digitalWrite(_attPin, HIGH);
}
