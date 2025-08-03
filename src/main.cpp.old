/*
Code for UGR23/ UGR24 Accelerator Pedal Position Sensor (APPS)
Author: Fraser Wallace
Amended by: Max Wilkinson
Intended microcontroller to run this code: mbed LPC1768
*/

#include "mbed.h"
#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <string.h>


// Pin Setup 
I2C i2c1(p9, p10);                  //I2C pins for APPS1 (sda, scl)
I2C i2c2(p28, p27);                 //I2C pins for APPS2 (sda, scl)
CAN can(p30, p29, 500000);          //CAN pins for transceiver (Rx, Tx, frequency Hz)
DigitalOut SDC(p8);                 //Output to the shutdown circuit
DigitalIn RtD(p21);                 //Ready to drive signal
BusOut leds(LED4,LED3,LED2,LED1);   //Bus to display errors: code of signals below
BufferedSerial pc(p13, p14);        //UART debugging port, use external UART TTL USB adapter

/*
In the event that the team want to limit the motor torque at maximum throttle, the following value is the
only value which needs to change. In normal use it should be 100%. However, it is possible with our current
inverter used in UGR24, to go up to 150% but this is not advised. Value must be greater than 0 or no driving
will occur but should be substantially higher to have much effect on the vehicle's motion.

Note that if the inverter changes, then changes to the CAN ID and output torque ranges must change

Also note that maxDriveThrottlePCT is in the range of integers between 0 and 150.

Constants setup
*/

const uint8_t maxDriveTorquePCT = 100;      //percentage of the torque produced by the motor at maximum throttle position in drive mode

const uint8_t overTravelLeewayPCT = 10;     //percentage of throttle travel added on before the SDC is opened eg 0 => kill car if over maximum, 100 => kill car at 200% max throttle travel 
const uint8_t underTravelLeewayPCT = 10;    //same principle as before but for negative throttles just in case of interference
const uint8_t noThrottleTravelPCT = 5;      //% of throttle travel where the output remains 0% 
const uint8_t fullThrottleTravelPCT = 0;    //% of throttle travel where the output remains 100% 

const uint16_t noThrottlePos_1 = 0x0A17;    //position apps 1 reads at 0% throttle, different as per rules #0x02ED
const uint16_t fullThrottlePos_1 = 0x0B14;  //position apps 1 reads at 100% throttle, different as per rules #0x03FBA
const uint16_t noThrottlePos_2 = 0x0CC5;    //position apps 2 reads at 0% throttle, different as per rules #0x05DB
const uint16_t fullThrottlePos_2 = 0x0DC5;  //position apps 2 reads at 100% throttle, different as per rules #0x06AB

const uint32_t killTime = 100000;           //us time between start of implausibility and killing power 
const uint32_t setupTime = 1000000;         //us time available for sensor setup 

//not constant as a function sets these to within range of [0,4095] so may need to be changed once.
uint16_t overTravelLeewayValue_1 = fullThrottlePos_1 + ((float(overTravelLeewayPCT)/100) * (fullThrottlePos_1 - noThrottlePos_1)); //Declares value for MPOS for APPS 1
uint16_t underTravelLeewayValue_1 = noThrottlePos_1 - ((float(underTravelLeewayPCT)/100) * (fullThrottlePos_1 - noThrottlePos_1)); //Declares value for ZPOS for APPS 1
uint16_t overTravelLeewayValue_2 = fullThrottlePos_2 + ((float(overTravelLeewayPCT)/100) * (fullThrottlePos_2 - noThrottlePos_2)); //Declares value for MPOS for APPS 2
uint16_t underTravelLeewayValue_2 = noThrottlePos_2 - ((float(underTravelLeewayPCT)/100) * (fullThrottlePos_2 - noThrottlePos_2)); //Declares value for ZPOS for APPS 2
 
//constants used in the switch case for throttleOut() - no point doing the same float calculation when it never changes throughout the program
const uint16_t zeroThrottleUpperLim = (4095 * (noThrottleTravelPCT + underTravelLeewayPCT))/(100 + underTravelLeewayPCT + overTravelLeewayPCT);
const uint16_t throttleCalcLowerLim = zeroThrottleUpperLim + 1;
const uint16_t fullThrottleLowerLim = 4095 - (4095 * (fullThrottleTravelPCT + overTravelLeewayPCT))/(100 + underTravelLeewayPCT + overTravelLeewayPCT);
const uint16_t throttleCalcUpperLim = fullThrottleLowerLim - 1;

// implausibility threshold - 10% of pedal travel - upperLim at 95% and lowerLim at 5% so 90% pedal travel = difference. Divide by 9 to get 10% travel
const uint16_t deltaThresh = (throttleCalcUpperLim - throttleCalcLowerLim)/9;

// Communication Constants 
// CAN 
const uint16_t CAN_ID = 0x210;           // ECU message: 0x212, if bypassing ECU for testing then use 0x201 fir direct inverter communication
 
// I2C 
const uint32_t i2cFrequency = 100000;    // I2C frequency in Hz (up to 1MHz) 
const int AS5600_ADDR = 0x36 << 1;       // 7 bit address; address of sensor; written on the bus to call this sensor - shifted to b7 to b1 since b0 is read/write
const char ZMCO_ADDR = 0x00;             // 2 LSB RO;

CANMessage rcv;

const char ZPOS_ADDR = 0x01;             // 12 BIT RW; Address which stores the minimum angle - should be defined in the sensor setup, constants obtained from testing with physical pedalbox
const char MPOS_ADDR = 0x03;             // 12 BIT RW; Address which stores the maximum angle - should be defined in the sensor setup, constants obtained from testing with physical pedalbox
const char MANG_ADDR = 0x05;             // 12 BIT RW; 
const char CONF_ADDR = 0x07;             // 14 BIT RW; Address which stores the 
const char RAW_ANGLE_ADDR = 0x0C;        // 12 BIT RO; Address which stores the raw angle
const char ANGLE_ADDR = 0x0E;            // 12 BIT RO; Address which stores the angle - this is the one which gets read from to understand the actual values
const char STATUS_ADDR = 0x0B;           // 6 LSB RO; 
const char AGC_ADDR = 0x1A;              // 8 BIT RO; 
const char MAGNITUDE_ADDR = 0x1B;        // 12 BIT RO; 
//const char BURN_ADDR = 0xFF;             // 8 BIT WO (Burn_Angle = 0x80;, Burn_Setting = 0x40) Can be used 3 times per sensor so do not use!
 
// Variables 
uint16_t angle_1;           //input from APPS1, range depends on sensor data testing [0,4095]
uint16_t angle_2;           //input from APPS2, range depends on sensor data testing [0,4095]
uint16_t delta;             //difference between APPS1 and APPS2
unsigned long startTime;    //start time of implausibility
unsigned long timeElapsed;  //duration of implausibility
bool killPower = false;     //always send 0 torque request, doesn't kill car power, kills motor power
bool resetSDC = true;       //sets the SDC initially as closed. To open SDC, resetSDC = false

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread canThread;

/******************************** SUPPORTING FUNCTIONS ***********************************/ 
 
/*********************************** Safety Critical *************************************/ 
/* 
LED fault codes: 
    0b0000 = everything is working as expected 
    0b0001 = no acknowledge on apps1 write
    0b0010 = no acknowledge on apps1 read
    0b0011 = no acknowledge on apps2 wirte
    0b0100 = no acknowledge on apps2 read
    0b0101 = value out of range 
    0b0110 = sensro setup not written correctly 
    0b0111 = error (implausibility) 
    0b1000 = RtD Low
    0b1001 = implausibility duration > kill time (kill power) 
    0b1010 = RtD Low && implausibility duration > kill time (kill power) 
    0b1011 = CAN write error 
    0b1100 = 
    0b1101 = Throttle travel out of range
    0b1110 = 
    0b1111 = temp error
*/ 

//Sends the throttle signal to the ECU in little endian format
int throttleOut(uint16_t torque) { 

    char data[3];
    data[0] = 0x90;             //This is for the inverter to know that the next two bytes of data relate to a throttle command as opposed to speed or position commands
    data[1] = char(torque);     //least significant byte of throttle
    data[2] = char(torque>>8);  //most significant byte of throttle
     
    if (!can.write(CANMessage(CAN_ID, data, 3))) {  // writes to CAN bus with defined ID, data and a data length of 2 bytes
        resetSDC = false;                           //doesn't reset the shutdown circuit as error occurs (CAN bus send failed)
        return(0b1011);                             //return error code
    }    
    else {
        return(0b0000);                             //return no error
    } 
} 

//Opens the shutdown circuit 
void openShutdownCircuit(uint8_t ledcode) { 
    SDC = 0;                                        //opens shutdown circuit
    if (leds == 0b0000) {leds = ledcode;}           //displays error on the LED bus - error code found in comments above
    throttleOut(0);                                 //send a torque value of 0 to ECU 
} 

void canHandler() {
    while (can.read(rcv)) {
        if(rcv.id==0x100) {
            openShutdownCircuit(0x1111);
        }
    }
}

//converts sensor data to throttle in inverter range
uint16_t getTorque(uint16_t sensorData) {
    uint16_t throttle = 0;

    switch (sensorData) {
        case 0:
            openShutdownCircuit(0b1101);
            break;
        case 1 ... zeroThrottleUpperLim:
            throttle = 0;
            break;
        case throttleCalcLowerLim ... throttleCalcUpperLim:
            throttle = (float(maxDriveTorquePCT)/150)*((float(32767)/(throttleCalcUpperLim - throttleCalcLowerLim))*(sensorData - throttleCalcLowerLim));
            break;
        case fullThrottleLowerLim ... 4094:
            throttle = 0;
            break;
        case 4095:
            openShutdownCircuit(0b1101);
            break;
    }
    return throttle;
}
 
// Retrieves a value from a sensor 
uint16_t getValue(uint8_t sensor, char regAddr) { 
    char data[2]; 
    uint16_t value = 0x0000; 
    if (sensor == 1) {         
        if (i2c1.write(AS5600_ADDR|0x00, &regAddr, 1, true)) {  //writes address and register to sensor
            resetSDC = false;                                   //if the result is 1 then there was an acknowledge error so open SDC
            // openShutdownCircuit(0b0001);                        //open SDC with this error code
            return 0x0000;                                      //returns this to exit the function and send a 0 torque request. Shouldn't be required but just for safety
        } 
 
        if (i2c1.read(AS5600_ADDR|0x01, &data[0], 2, false)) {  //writes address and reads data from selected register from sensor
            resetSDC = false;                                   //if the result is 1 then there was an acknowledge error so open SDC
            openShutdownCircuit(0b0010);                        //open SDC with this error code
            return 0x0000;                                      //returns this to exit the function and send a 0 torque request. Shouldn't be required but just for safety
        }
     
        value = (data[0] << 8 | data[1]);                       //i2c sends big-endian data - this converts to one 2-byte value
        char message2[64] = {0}; // Message to send
        sprintf(message2, "Sensor 1 Value: 0x%04X\r\n", value);
        //pc.write(message2, sizeof(message2));
    }  
 
    else {         
        if (i2c2.write(AS5600_ADDR|0x00, &regAddr, 1, true)) {  //writes address and register to sensor
            resetSDC = false;                                   //if the result is 1 then there was an acknowledge error so open SDC
            openShutdownCircuit(0b0011);                        //open SDC with this error code
            return 0x0000;                                      //returns this to exit the function and send a 0 torque request. Shouldn't be required but just for safety
        } 
 
        if (i2c2.read(AS5600_ADDR|0x01, &data[0], 2, false)) {  //writes address and reads data from selected register from sensor
            resetSDC = false;                                   //if the result is 1 then there was an acknowledge error so open SDC
            openShutdownCircuit(0b0100);                        //open SDC with this error code
            return 0x0000;                                      //returns this to exit the function and send a 0 torque request. Shouldn't be required but just for safety
        }
     
        value = (data[0] << 8 | data[1]);                       //i2c sends big-endian data - this converts to one 2-byte value
        char message3[64] = {0}; // Message to send
        sprintf(message3, "Sensor 2 Value: 0x%04X\r\n", value);
        //pc.write(message3, sizeof(message3));
    }  
 
    if (value >= 0x1000 && (regAddr == ANGLE_ADDR || regAddr == RAW_ANGLE_ADDR)) { 
        resetSDC = false;  
        openShutdownCircuit(0b0101);                            //Read angle is out of range 
        return 0x0000;
    }  
     
    return value; 
} 
 
//Sets a sensor variable 
bool setValue(uint8_t sensor, uint8_t regAddr, uint16_t value) { 
    //Check to ensure out of range data is not sent to sensors 
    if (value >= 0x1000 && regAddr != CONF_ADDR) { 
        return false; 
    } else if (value >= 0x4000 && regAddr == CONF_ADDR) { 
        return false; 
    } 
     
    char data[3];                   //the 3 bytes written are put into one data array to be sent across the bus
    data[0] = regAddr;              //address of register to write to
    data[1] = char(value >> 8);     //MSB of value to store in selected register
    data[2] = char(value);          //LSB of value to store in selected register
     
    if (sensor == 1) { 
        if (i2c1.write(AS5600_ADDR|0x00, &data[0],3,false)) {
            char message1[] = "I2C1 Broken\r\n"; // Message to send
            pc.write(message1, sizeof(message1));
            return false;           //if send fails then not set up therefore returns false
        }
    }   
    else {  //if (sensor == 2) 
        if (i2c2.write(AS5600_ADDR|0x00, &data[0],3,false)) {
            char message2[] = "I2C2 Broken\r\n"; // Message to send
            pc.write(message2, sizeof(message2));
            return false;           //if send fails then not set up therefore returns false
        }
    }  
 
     // Check for correct transmission 
    if (getValue(sensor, regAddr) != value) { 
        return false; 
    } 
    return true; 
} 
 
/********************************* Non-Safety Critical ***********************************/ 

uint16_t sensorRangeCorrection(uint16_t sensorValue) {
    if (sensorValue > 4095) {
        sensorValue = sensorValue - 4096;
    }
    else if (sensorValue < 0) {
        sensorValue = sensorValue + 4096;
    }

    return sensorValue;
}

// Sets the sensor limits 
bool setupSensors() {    
    //ensures the value the set positions is in the range of [0,4095]
    uint16_t zpos_1 = sensorRangeCorrection(underTravelLeewayValue_1);
    uint16_t zpos_2 = sensorRangeCorrection(underTravelLeewayValue_2);
    uint16_t mpos_1 = sensorRangeCorrection(overTravelLeewayValue_1);
    uint16_t mpos_2 = sensorRangeCorrection(overTravelLeewayValue_2);
     
    bool set[4] = {false,false,false,false};                //zpos_1, mpos_1, zpos_2,mpos_2 all initially false

    uint32_t tf = us_ticker_read() + setupTime;             //defines a value for us_ticker_read() to not exceed so it doesn't get stuck in an infinite loop
    while (us_ticker_read() < tf) {                         //while below this htreshold value, continually try to set up the sensros
        if (!set[0] || !set[1] || !set[2] || !set[3]) {     //early exit condition - if all 4 sensors set up successfully then exit early
            set[0] = setValue(1, ZPOS_ADDR, zpos_1);        //attempts to set zpos of APPS1, returns true if successful
            set[1] = setValue(1, MPOS_ADDR, mpos_1);        //attempts to set mpos of APPS1, returns true if successful
            set[2] = setValue(2, ZPOS_ADDR, zpos_2);        //attempts to set zpos of APPS2, returns true if successful
            set[3] = setValue(2, MPOS_ADDR, mpos_2);        //attempts to set mpos of APPS2, returns true if successful
        } else { 
            break;                                          //exits loop early
        } 
    } 
 
    if (!set[0] || !set[1] || !set[2] || !set[3]) {         //tests all 4 variables, if any are false returns flase, otherwise returns true
        return false;                                       //may be more efficient to test for positives and use and then flip the conditions
    }                                                       //this is only run once so efficiency isn't important and may have been done this way
    else {                                                  //for a specific reason by Fraser.
        return true; 
    } 
} 

void isrCanRx() {
    // output=!output;
    queue.call(canHandler);
}
 
// Initialisation 
void setup() { 
    leds = 0b0000;                      //Initially, no errors so the LEDs are all 0
    SDC = 1;                            //declares shutdown circuit as 1 => SDC closed
     
    i2c1.frequency(i2cFrequency);       //setup of I2C line 
    i2c2.frequency(i2cFrequency);       //setup of I2C line 
 

    wait_us(1000000); 
    while(!setupSensors()) { 
        openShutdownCircuit(0b0110);    //Setup Values incorrectly set after allocated setup time 
    } 

    can.mode(CAN::Normal);              //sets the CAN controller to operate in normal mode
    can.filter(0x100, 0x7FF, CANStandard, 0);
    pc.set_baud(9600);
    char message0[] = "\r\n APPS Debug Terminal\r\n"; // Message to send
    pc.write(message0, sizeof(message0) - 1);
} 
 
/*--------------- Start of Main Program ---------------*/

int main() { 
    setup(); 

     
    while(1) {    
 
        wait_us(500);  

        if (can.read(rcv,0)) {
            resetSDC = false;
            openShutdownCircuit(0x1111);
        } 
        else {
            
            angle_1 = getValue(1, ANGLE_ADDR);  //sensor reading in range 2000-4000
            angle_2 = getValue(2, ANGLE_ADDR);  //sensor reading in range 1000-3000
            delta = abs(angle_1 - angle_2);     //difference in sensor readings - implausibility check

            uint16_t raw_angle_1;
            raw_angle_1 = getValue(1, RAW_ANGLE_ADDR);
            
            uint16_t raw_angle_2;
            raw_angle_2 = getValue(2, RAW_ANGLE_ADDR);   

            if (delta < deltaThresh) {          //delta less than threshold means no implausibility
                killPower = false;              //allows actual torque to be sent
                startTime = us_ticker_read();   //resets the start time
                timeElapsed = 0;                //resets the elapsed time of implausibility
            }   
            else {   
                if (leds == 0b0000) {leds = 0b0111;}            //error (implausibility) 
                timeElapsed = us_ticker_read() - startTime;     //calculates the time implausibilitty has occured for
                
                if (timeElapsed > killTime) {                   //if its greater than the killTime defined by the rules then kill the torque, doesn't need to shutdown
                    if (leds == 0b0000) {leds = 0b1001;};       //kill power 
                    killPower = true; 
                } 
            } 

            uint16_t torque = getTorque(min(angle_1,angle_2));  //Angle 1 and angle 2 checked for implausibility, getting throttle for one value.
    

            char message4[128] = {0}; // Message to send
            sprintf(message4, "Angle 1: %d, Angle 2: %d, Raw Angle 1: %d, Raw Angle 2: %d, Delta: %d, Torque: %d\r\n", angle_1,angle_2,raw_angle_1, raw_angle_2, delta, torque);
            pc.write(message4, sizeof(message4));



            if (RtD && !killPower) {                            // RtD HI means in ready-to-drive and not killPower means no error
                int ledErrorCode = throttleOut(torque);  
                if (ledErrorCode != 0b0000) {                   //if throttleOut causes an error then the code is retuturned as non-zero
                    openShutdownCircuit(ledErrorCode);          //SDC opened and LED bus set to value of error codeee
                }
            }  
            else { 
                int ledErrorCode = throttleOut(0); 
                if (ledErrorCode != 0b0000) {                   //should be inside function but cuases functions calling each other - compile error
                    openShutdownCircuit(ledErrorCode);
                }
    
                if (!RtD && killPower) { 
                    if (leds == 0b0000) {leds = 0b1010;}        // ready to drive signal Low && implausibility time > kill time 
                } 
                else if (!RtD) { 
                    if (leds == 0b0000) {leds = 0b1000;}        // ready to drive signal Low (kill power) 
                } 
                else { 
                    if (leds == 0b0000) {leds = 0b1001;}        // implausibility duration > kill time  
                } 
            } 
        }
    } 
}