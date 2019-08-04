/*****************************************************************************

  2019-08-03 -> New modbus library from https://github.com/yaacov/ArduinoModbusSlave 
             -> Input Registers (FC=04) 0 to 7 are analog inputs a/d values from 1 to 8 (read only 10 bits 0-1023)
             -> Input Registers (FC=04) 8 to 15 are analog output states from 1 to 8 (read only 0-255)
             -> Holding Registers (FC=06) 0 to 7 are analog output drive from 1 to 8 (write only 0-255)

  2017-04-05 -> Add INPUTS average calculations
******************************************************************************/

// include libraries:
#include <ModbusSlave.h>
#include <SPI.h>

#define SLAVE_ID 1
#define CTRL_PIN A4
#define BAUDRATE 19200

SPISettings MCP3008(500000, MSBFIRST, SPI_MODE0);
SPISettings TLC5620(500000, MSBFIRST, SPI_MODE1);

// set slave select pins
const int adcChipSelect = 8;
const int dacLoadPin[] = {10, 9}; //LOAD pin. 2 DACs TLC5620

long previousMillis = 0;

int analogInput[8];
byte analogInputBytes[16]; //analog input 0@7 separate in two bytes MSB,LSB {MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB}
int idx = 0;
int aiIdx = 0;

byte analogOutput[8];
const byte dacAddress[] = {0x03, 0x01, 0x05, 0x07}; //DAC addr and range bit. first bytes to send to TLC5620

int analogInputAvg[8];
int readingsAI1[10];
int readingsAI2[10];
int readingsAI3[10];
int readingsAI4[10];
int readingsAI5[10];
int readingsAI6[10];
int readingsAI7[10];
int readingsAI8[10];
byte count = 0;

Modbus slave(SLAVE_ID, CTRL_PIN);

void setup() {

  // set the ChipSelect as an output:
  pinMode(adcChipSelect, OUTPUT);
  pinMode(dacLoadPin[0], OUTPUT);
  pinMode(dacLoadPin[1], OUTPUT);

  // initialize SPI:
  SPI.begin();

  digitalWrite(adcChipSelect, LOW);
  digitalWrite(adcChipSelect, HIGH);

  digitalWrite(dacLoadPin[0], HIGH);
  digitalWrite(dacLoadPin[1], HIGH);

  //slave.cbVector[CB_READ_INPUT_REGISTERS] = readAnalogIn;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = readAnalogInOut;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeAnalogOut;

  Serial.begin( BAUDRATE );
  slave.begin( BAUDRATE );

}


void loop() {

  // poll for Modbus RTU requests
  slave.poll();

  for (int i = 0; i <= 7; i++) {
    analogInput[i] = adcChannelRead(i);
    analogInTwoBytes(i);
  }

  readAvgAnalogIn();
  analogOutputWrite();

}


//Handle Read Input Registers (0 To 15) (FC=04) and write back the values from analog inputs and analog output status
uint8_t readAnalogInOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint16_t registerIndex;

  // read analog input
  for (int i = 0; i < length; i++) {
    registerIndex = address + i;

    if (registerIndex >= 0 && registerIndex <= 7) { slave.writeRegisterToBuffer(i, analogInputAvg[registerIndex]); }
    if (registerIndex >= 8 && registerIndex <= 15) { slave.writeRegisterToBuffer(i, analogOutput[registerIndex-8]); }
  }

  return STATUS_OK;
}


/*
//Handle Read Input Registers (0 To 7) (FC=04) and write back the values from analog inputs
uint8_t readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read analog input
    for (int i = 0; i < length; i++) {
        slave.writeRegisterToBuffer(i, analogInputAvg[address + i]);
    }

    return STATUS_OK;
}
*/

//Handle Write Holding Registers (0 To 7) (FC=06, FC=16) and write data into analog outputs
uint8_t writeAnalogOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint16_t value;
  uint16_t registerIndex;

  // set analog output state(s)
  for (int i = 0; i < length; i++) {
    registerIndex = address + i;

    value = slave.readRegisterFromBuffer(i);
    if (registerIndex < 8 && value < 256) { analogOutput[registerIndex] = value; }
  }

  return STATUS_OK;
}


int adcChannelRead(byte readAddress) {

  byte dataMSB = 0;
  byte dataLSB = 0;
  byte JUNK = 0x00;

  SPI.beginTransaction(MCP3008);
  digitalWrite(adcChipSelect, LOW);
  SPI.transfer(0x01); // Start Bit
  dataMSB = SPI.transfer(((0x01 <<  3) | readAddress) << 4) & 0x03; // Send readAddress and receive MSB data, masked to two bits
  dataLSB = SPI.transfer(JUNK); // Push junk data and get LSB byte return
  digitalWrite(adcChipSelect, HIGH);
  SPI.endTransaction();

  return dataMSB << 8 | dataLSB;
}


void readAvgAnalogIn() {
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis  > 5) {
    previousMillis = currentMillis;

    if (count > 9) count = 0;
    readingsAI1[count] = analogInput[0];
    readingsAI2[count] = analogInput[1];
    readingsAI3[count] = analogInput[2];
    readingsAI4[count] = analogInput[3];
    readingsAI5[count] = analogInput[4];
    readingsAI6[count] = analogInput[5];
    readingsAI7[count] = analogInput[6];
    readingsAI8[count] = analogInput[7];
    count++;

    analogInputAvg[0] = getAgerage(readingsAI1);
    analogInputAvg[1] = getAgerage(readingsAI2);
    analogInputAvg[2] = getAgerage(readingsAI3);
    analogInputAvg[3] = getAgerage(readingsAI4);
    analogInputAvg[4] = getAgerage(readingsAI5);
    analogInputAvg[5] = getAgerage(readingsAI6);
    analogInputAvg[6] = getAgerage(readingsAI7);
    analogInputAvg[7] = getAgerage(readingsAI8);

  }
}


int getAgerage(int readings[10]) {
    int sum = 0;
    int avg = 0;

    for (int i = 0; i < 10; i++) {
       sum += readings[i];
    }
    avg = sum / 10;

    return avg;
}


void analogInTwoBytes(int id) { //Put 10 bits analog input in two separate bytes. For sending through I2C.
  //extract high and low from analog input readings. Stores in array after....
  int AnlgIn = analogInputAvg[id];
  byte high = (AnlgIn >> 8) & 0xFF;
  byte low = AnlgIn & 0xFF;

  if (id == 0) {
    aiIdx = 0;
  }
  analogInputBytes[aiIdx] = high;
  analogInputBytes[aiIdx + 1] = low;
  aiIdx += 2;
}


void dacChannelWrite(byte high, byte low, int loadPin) {

  SPI.beginTransaction(TLC5620);
  digitalWrite(loadPin, HIGH);
  SPI.transfer(high);
  SPI.transfer(low);
  digitalWrite(loadPin, LOW);
  digitalWrite(loadPin, LOW);
  digitalWrite(loadPin, LOW);
  digitalWrite(loadPin, HIGH);
  SPI.endTransaction();
}


void analogOutputWrite() {

  for (int a = 0; a <= 3; a++) {
    dacChannelWrite(dacAddress[a], analogOutput[a], dacLoadPin[0]); //write addr and value to DAC #1 TCL5620
  }
  for (int a = 0; a <= 3; a++) {
    dacChannelWrite(dacAddress[a], analogOutput[a + 4], dacLoadPin[1]); //write addr and value to DAC #2 TCL5620
  }
}

//Fin
