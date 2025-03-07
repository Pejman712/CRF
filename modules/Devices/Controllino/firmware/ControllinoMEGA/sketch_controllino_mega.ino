#include <SPI.h>
#include <Controllino.h>
#include <Modbus.h>
#include <ModbusSerial.h>

// Modbus Registers Offsets (0-9999)
const int DIGITAL_CONF_START_REGISTER = 100;
const int DIGITAL_INPUT_START_REGISTER = 200;
const int DIGITAL_OUTPUT_START_REGISTER = 300;
word DigitalConfigurationArray[24];
word PreviousDigitalConfigurationArray[24];
const int DigitalPins[24] = {CONTROLLINO_D0, CONTROLLINO_D1, CONTROLLINO_D2, CONTROLLINO_D3, CONTROLLINO_D4, CONTROLLINO_D5, CONTROLLINO_D6, CONTROLLINO_D7, CONTROLLINO_D8, CONTROLLINO_D9, CONTROLLINO_D10, CONTROLLINO_D11, CONTROLLINO_D12, CONTROLLINO_D13, CONTROLLINO_D14, CONTROLLINO_D15, CONTROLLINO_D16, CONTROLLINO_D17, CONTROLLINO_D18, CONTROLLINO_D19, CONTROLLINO_D20, CONTROLLINO_D21, CONTROLLINO_D22, CONTROLLINO_D23};

const int ANALOG_CONF_START_REGISTER = 124;
word AnalogConfigurationArray[15];
word PreviousAnalogConfigurationArray[15];
const int AnalogPins[15];

const int RELAY_START_REGISTER = 700;
int RelayPins[16] = {CONTROLLINO_R0, CONTROLLINO_R1, CONTROLLINO_R2, CONTROLLINO_R3, CONTROLLINO_R4, CONTROLLINO_R5, CONTROLLINO_R6, CONTROLLINO_R7, CONTROLLINO_R8, CONTROLLINO_R9, CONTROLLINO_R10, CONTROLLINO_R11, CONTROLLINO_R12, CONTROLLINO_R13, CONTROLLINO_R14, CONTROLLINO_R15};

// ModbusSerial object
ModbusSerial mb;

void updateDigitalConfiguration() {
  
}

void setup() {
    // Config Modbus Serial (port, speed, byte format) 
    mb.config(&Serial, 115200, SERIAL_8N1);
    // Set the Slave ID (1-247)
    mb.setSlaveId(10); 

    mb.addHreg(1, 24);
    mb.addHreg(3, 16);
    
    //
    for (int i=0; i<24; i++) {
        mb.addHreg(DIGITAL_CONF_START_REGISTER+i, 0);
        mb.addHreg(DIGITAL_INPUT_START_REGISTER+i, 0);
        mb.addHreg(DIGITAL_OUTPUT_START_REGISTER+i, 0);

        DigitalConfigurationArray[i] = PreviousDigitalConfigurationArray[i] = 0;
    }

    /*for (int i=0; i<15; i++) {
        mb.addHreg(ANALOG_CONF_START_REGISTER+i, 0);

        AnalogConfigurationArray[i] = PreviousAnalogConfigurationArray[i] = 1;
    }*/

    for (int i=0; i<16; i++) {
        mb.addHreg(RELAY_START_REGISTER + i, 0);
        pinMode(RelayPins[i], OUTPUT);
    }
}

void loop() {
   //Call once inside loop() - all magic here
   mb.task();

   for (int i=0; i<24; i++) {
      /** Update digital configuration **/
      DigitalConfigurationArray[i] = mb.Hreg(DIGITAL_CONF_START_REGISTER+i);
      
      if (PreviousDigitalConfigurationArray[i] != DigitalConfigurationArray[i]) {
        if (DigitalConfigurationArray[i] == 1) {
          pinMode(DigitalPins[i], INPUT);  
        } else if (DigitalConfigurationArray[i] == 2) {
          pinMode(DigitalPins[i], OUTPUT);  
        }
      }

      PreviousDigitalConfigurationArray[i] = DigitalConfigurationArray[i];

      /** Update digital values **/
      if (DigitalConfigurationArray[i] == 1) {
        mb.Hreg(DIGITAL_INPUT_START_REGISTER+i, digitalRead(DigitalPins[i]));
      } else if (DigitalConfigurationArray[i] == 2) {
        digitalWrite(DigitalPins[i], mb.Hreg(DIGITAL_OUTPUT_START_REGISTER+i));
      }
   }

   for (int i=0; i<16; i++) {
      digitalWrite(RelayPins[i], mb.Hreg(RELAY_START_REGISTER+i));
   }
   
   delay(15);
}
