#include "mbed.h"

int flag = 0;
int avgfspd=0;
CAN can1(D10, D2, 500000);
AnalogIn FBP(A1);
AnalogIn RBP(A0);
Ticker canint;
DigitalOut output(D9);
float FBPthresh = 0.151;
float RBPthresh = 0.151;

CANMessage msg;
CANMessage rcv;
CANMessage screen;
CANMessage startup;

// Event queue + thread for deferred CAN handling
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread canThread;

void setFlag() {
    flag = 1;
}

// This will safely run outside the ISR
void handleCanRx() {
    while (can1.read(rcv)) {
        switch (rcv.id) {
            case 0x210: 
            //received apps data
            msg.len =3;
            msg.data[0] = rcv.data[0];
            if(FBP>FBPthresh||RBP>RBPthresh){
                msg.data[1]=0;
                msg.data[2]=0;
            }
            else{
                msg.data[1]=rcv.data[1];
                msg.data[2]=rcv.data[2];
            }
            can1.write(msg); break;
            case 0x220: 
            //received wheelspeed data
            avgfspd=(rcv.data[0]*256+rcv.data[1]+rcv.data[2]*256+rcv.data[3])/2;
            avgfspd=int(avgfspd*0.00223694);
            screen.data[0]=avgfspd; break;
            case 0x6B1:
            screen.data[1]=rcv.data[4];
            screen.data[3]=rcv.data[3]; break;
            case 0x181:
            //received inverter data
            break;
            case 0x1838F380:
            screen.data[2]=rcv.data[2];
            break;
        }
    } 
}

// ISR — only schedules the read function
void isrCanRx() {
    // output=!output;
    queue.call(handleCanRx);
}

int main() 
{    
    output=0;
    can1.filter(0x000, 0x000, CANStandard, 0);
    // Start CAN event queue thread
    canThread.start(callback(&queue, &EventQueue::dispatch_forever));

    msg.len = 3;
    msg.id = 0x201;
    msg.format = CANStandard;
    msg.type = CANData;
    msg.data[0] = 0;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = 0;

    // startup.len = 3;
    // startup.id = 0x201;
    // startup.format = CANStandard;
    // startup.type = CANData;
    // startup.data[0] = 0x3D;
    // startup.data[1] = 0x30;
    // startup.data[2] = 0xC8;
    // can1.write(startup);
    // wait_us(100);
    // startup.len = 4;
    // startup.id = 0x190;
    // startup.data[0] = 0x30;
    // startup.data[1] = 0xFF;
    // startup.data[2] = 0x7F;
    // startup.data[3] = 0x10;
    // can1.write(startup);
    // startup.data[1]=0xf6;
    // can1.write(startup);

    screen.len = 4;
    screen.id = 0x123;
    screen.format = CANStandard;
    screen.type = CANData;
    screen.data[0] = 0;
    screen.data[1] = 0;
    screen.data[2] = 0;
    screen.data[3] = 0;

    rcv.data[0] = 0;
    rcv.data[1] = 0;
    rcv.data[2] = 0;
    rcv.data[3] = 0;
    rcv.data[4] = 0;
    rcv.data[5] = 0;
    rcv.data[6] = 0;
    rcv.data[7] = 0;
    canint.attach(&setFlag, 200ms);
    can1.attach(&isrCanRx, CAN::RxIrq);

    while (1) {
        wait_us(1000);
        if (flag == 1) {
            can1.write(screen);
            flag = 0;
        }
    }
}