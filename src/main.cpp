#include "mbed.h"

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

// Pin Setup 
I2C i2c1(p9, p10);                  //I2C pins for APPS1 (sda, scl)
I2C i2c2(p28, p27);                 //I2C pins for APPS2 (sda, scl)
CAN can(p30, p29, 500000);          //CAN pins for transceiver (Rx, Tx, frequency Hz)
DigitalOut SDC(p8);                 //Output to the shutdown circuit
DigitalIn RtD(p21);                 //Ready to drive signal
BusOut leds(LED4,LED3,LED2,LED1);   //Bus to display errors: code of signals below
BufferedSerial pc(p13, p14);        //UART debugging port, use external UART TTL USB adapter

unsigned long canTimestamp;

// Error code enum

enum ErrorCode {
    NO_ERROR = 0b0000,
    APPS1_WRITE_FAIL = 0b0001,
    APPS1_READ_FAIL = 0b0010,
    APPS2_WRITE_FAIL = 0b0011,
    APPS2_READ_FAIL = 0b0100,
    VALUE_OUT_OF_RANGE = 0b0101,
    SENSOR_SETUP_FAIL = 0b0110,
    IMPLAUSIBILITY = 0b0111,
    RTD_LOW = 0b1000,
    KILL_POWER = 0b1001,
    RTD_LOW_AND_KILL = 0b1010,
    CAN_WRITE_ERROR = 0b1011,
    THROTTLE_OUT_OF_RANGE = 0b1101,
    TEMP_ERROR = 0b1111
};

/* Functions */

// CAN

CANMessage rcv;

bool checkTSCAN() {
    if (us_ticker_read() - canTimestamp > 100000) {
        if (!can.read(rcv)) {
            if(rcv.id==0x100) {
                openShutdownCircuit(TEMP_ERROR);
                return 0;
            }
        }
    }
    return 1;
}


// Shutdown Circuit

void openShutdownCircuit(int error_led) {
    SDC = 0;
    leds = error_led;
}

void closeShutdownCircuit(int error_led = NO_ERROR) {
    SDC = 1;
    leds = error_led;
}

/* Loops */

int setup() {
    can.mode(CAN::Normal);              //sets the CAN controller to operate in normal mode
    can.attach(&isrCanRx, CAN::RxIrq);

    can.filter(0x100, 0x7FF, CANStandard, 0);
    canThread.start(callback(&queue, &EventQueue::dispatch_forever));

    canTimestamp = us_ticker_read();  

    closeShutdownCircuit();
}

int main() {
    setup();

    while (1) {
        if (checkTSCAN()) {
            // APPS logic
        } 
        wait_us(500);
    }
}