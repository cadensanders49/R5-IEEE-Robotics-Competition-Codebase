#include "mbed.h"
//
//
//   PIN DECLARATIONS
//
//
DigitalOut FLdirection(PTB18);
DigitalOut FRdirection(PTA4);
DigitalOut magDirection(PTB19);
PwmOut stepFL(PTD3);
PwmOut stepFR(PTA5);
PwmOut magArm(PTA12);
InterruptIn killAll(PTC3);
DigitalIn Start(PTC12);
DigitalOut enableH(PTC11);
DigitalOut highH(PTC10);
DigitalOut enableL(PTC11);
DigitalOut highL(PTC7);
I2C i2c(PTC9, PTC8); //pins for I2C communication (SDA, SCL)
Serial pc(USBTX, USBRX);
DigitalOut LED(PTC4);
DigitalOut green(LED_GREEN);
//
//
//   PROTOTYPE FUNCTION DECLARATIONS
//
//
void move(float dist, bool direction);
void grabToken();//Picks up the token for reading
void dropToken();//Drops the token off
void kill();
void turnRight();
void turnLeft();
void rot180(); //Turns the robot around
int findColor(); //Figures out what color the disk is and makes a decision on where to take the disk
void findPathReturn(int color, int i, float scale, float); //Determines where to take the token based on the color and its relative location and then returns to the previous position
void returnHome(); //Returns to the home white square
//
//
//    GLOBAL VARIABLES
//
//
const int FORWARD = 0;
const int BACKWARD = 1;
const float stepSize = 0.001212; //in meters
const float FREQUENCY = 500; //steps per second
int sensor_addr = 41 << 1;
float leg;
float scale;
int color;
int TIME = 100; //in seconds, this is where you input the round time duration
Timeout timer; // attach this to return home function, set according to round time
//
//
//
//
//
//
int main()
{
    //
    //
    //Start a timer
    //
    timer.attach(&returnHome, TIME);
    //
    //
    //Define premeasured distances
    //
    float radDistance = 0.5;
    float posDistance = 0.5;
    float armDistance = 0.5;
    //
    //
    enableH = 0; //Making sure the H-Bridge starts low and off
    highH = 0; //This starts high for the H-Bridge
    highL = 1; //This starts low for the H-Bridge
    //The start button
    while(true) {
        if (Start == 0)
            break;
    }
    //
    killAll.rise(&kill); //The kill interupt
    //
    //
    //   RGB Sensor Settings
    //
    //
    pc.baud(115200);
    green = 1; // off
    i2c.frequency(200000);
    char id_regval[1] = {146};
    char data[1] = {0};
    i2c.write(sensor_addr,id_regval,1, true);
    i2c.read(sensor_addr,data,1,false);
    if (data[0]==68) {
        green = 0;
        wait (2);
        green = 1;
    } else {
        green = 1;
    }
    //
    //
    //    Initialize color sensor
    //
    //
    char timing_register[2] = {129,0};
    i2c.write(sensor_addr,timing_register,2,false);
    char control_register[2] = {143,0};
    i2c.write(sensor_addr,control_register,2,false);
    char enable_register[2] = {128,3};
    i2c.write(sensor_addr,enable_register,2,false);
    //
    //
    //  Initialize the robot position
    //
    //
    move((0.6096-radDistance+posDistance+armDistance),FORWARD);
    turnLeft();
    move(radDistance,BACKWARD);
    move(0.762, FORWARD);
    //
    //
    //   Begin mapping algorithm
    //
    //
    while(true) {
        for(int i = 0; i <=7; i++) { //begin motion around Nth perimeter   ::No token on one length of the track so we may want to revise this
            leg = 0.762; //2.5 feet in meters
            scale = 1;
            if(i % 2 == 0) {
                turnRight();
            }
            grabToken();
            color = findColor();
            if (color == 9) {
                return(0);
            } else {
                findPathReturn(color, i, scale, leg);
                dropToken();
                //returnPrevious();
            }
            move(leg, FORWARD);
        }
    }
}
//
//
//
//
//   Functions...
//
//
//
//
void move(float dist, bool direction) //Moves the robot (meters)
{
    //
    //
    //  Define the direction
    //
    //
    FLdirection = direction;
    FRdirection = !direction;
    //
    //
    //   Define Speed
    //
    //
    stepFL.period(1.0/FREQUENCY);
    stepFR.period(1/FREQUENCY);
    stepFL.write(0.5f);
    stepFR.write(0.5f);
    //
    //
    //   Wait the correct amount of time to travel the specified distance
    //
    //
    wait(4*(dist/stepSize)*(1/FREQUENCY));  //(dist/stepSize) is the number of steps; 1/FREQUENCY is the time per step
    //
    //
    //  Stop the motors
    //
    //
    stepFL.period(0.0f);
    stepFR.period(0.0f);
    stepFL.write(0.0f);
    stepFR.write(0.0f);
}
//
//
//
//
//
void grabToken()
{
    //
    //
    //  Turn electromagnet on
    //
    //
    highL = 0;
    highH = 1;
    enableH = 1;
    //
    //
    //   Wait for token to magnetize
    //
    //
    wait(1);
    //
    //
    //   Move arm to rgb sensor
    //
    //
    magDirection = 1;
    magArm.period(0.002);
    magArm.write(0.5);
    wait(0.65);
    //
    //
    //   Stop moving the arm
    //
    //
    magArm.period(0);
    magArm.write(0);

}
//
//
//
//
//
void dropToken()
{
    //
    //
    //   Move arm to lower position
    //
    //
    magDirection = 0;
    magArm.period(0.002);
    magArm.write(0.5);
    wait(0.65);
    //
    //
    //   Stop moving arm
    //
    //
    magArm.period(0);
    magArm.write(0);
    //
    //
    //  Reverse magnet polarity
    //
    //
    highL = 1;
    highH = 0;
    wait(2);
    //
    //
    //   Disable electromagnet
    //
    //
    enableH = 0;
}
//
//
//
//
//
void turnRight(float dist, bool direction)
{
    //
    //
    //   Set motor direction
    //
    //
    FLdirection = 0; //to turn right we want this going FORWARD so a 0;
    //
    //
    //   Set motor speed
    //
    //
    stepFL.period(1/FREQUENCY);
    stepFL.write(0.5);
    //
    //
    //   Wait the amount of time needed for one turn
    //
    //
    wait(4*(0.35343/stepSize)*(1/FREQUENCY)); //(dist/stepSize) is the number of steps; 1/FREQUENCY is the time per step
    //
    //
    //   Stop motor
    //
    //
    stepFL.period(0);
    stepFL.write(0);
}
//
//
//
//
//
void turnLeft(float dist, bool direction)
{
    //
    //
    //   Set motor direction
    //
    //
    FRdirection = 1;    //to turn right we want this going FORWARD, since FORWARD = 0, it must be !0
    //
    //
    //   Set motor speed
    //
    //
    stepFR.period(1/FREQUENCY);
    stepFR.write(0.5);
    //
    //
    //   Wait the amount of time needed for one turn
    //
    //
    wait(4*(0.35343/stepSize)*(1/FREQUENCY)); //(dist/stepSize) is the number of steps; 1/FREQUENCY is the time per step
    //
    //
    //   Stop motor
    //
    //
    stepFR.period(0);
    stepFR.write(0);
}
//
//
//
//
//
void rot180()
{
    //
    //
    //   Set motor direction
    //
    //
    FLdirection = 0;    //to turn right we want this going FORWARD so a 0;
    //
    //
    //   Set motor speed
    //
    //
    stepFL.period(1/FREQUENCY);
    stepFL.write(0.5);
    //
    //
    //   Wait the amount of time needed for one turn
    //
    //
    wait(2*4*(0.35343/stepSize)*(1/FREQUENCY)); //(dist/stepSize) is the number of steps; 1/FREQUENCY is the time per step
    //
    //
    //   Stop motor
    //
    //
    stepFL.period(0);
    stepFL.write(0);
}
//
//
//
//
//
void kill() //Kill switch to stop the machine
{
    exit(0);
}
//
//
//
//
//
int findColor()  //Figures out what color the disk is and makes a decision on where to take the disk
{
    //
    //
    //   Loop to determine the color of the disk
    //
    //
    while (true) {
        wait(1);
        char clear_reg[1] = {148};
        char clear_data[2] = {0,0};
        i2c.write(sensor_addr,clear_reg,1, true);
        i2c.read(sensor_addr,clear_data,2, false);
        int clear_value = ((int)clear_data[1] << 8) | clear_data[0];
        char red_reg[1] = {150};
        char red_data[2] = {0,0};
        i2c.write(sensor_addr,red_reg,1, true);
        i2c.read(sensor_addr,red_data,2, false);
        int red_value = ((int)red_data[1] << 8) | red_data[0];
        char green_reg[1] = {152};
        char green_data[2] = {0,0};
        i2c.write(sensor_addr,green_reg,1, true);
        i2c.read(sensor_addr,green_data,2, false);
        int green_value = ((int)green_data[1] << 8) | green_data[0];
        char blue_reg[1] = {154};
        char blue_data[2] = {0,0};
        i2c.write(sensor_addr,blue_reg,1, true);
        i2c.read(sensor_addr,blue_data,2, false);
        int blue_value = ((int)blue_data[1] << 8) | blue_data[0];


        //1=red,2=green,3=blue,4=cyan,5=magenta,6=yellow,7=gray,8=error, 9=nothing
        if(blue_value<10000 && red_value>10000) {
            return(1);
        } else if(green_value>18000 && blue_value<30000) {
            return(2);
        } else if(red_value<10000 && blue_value>15000) {
            return(3);
        } else if(blue_value>30000 && red_value<20000) {
            return(4);
        } else if(red_value>25000 && green_value<15000) {
            return(5);
        } else if(red_value>50000) {
            return(6);
        } else if(red_value<10000 && blue_value<10000) {
            return(7);
        } else if(red_value==0) {
            return(8);
        } else {
            return(9);
        }
        // print sensor readings

        //pc.printf("Clear (%d), Red (%d), Green (%d), Blue (%d)\n", clear_value, red_value, green_value, blue_value);
        //wait(0.5);
    }
//This function decides where to go based on its reletive position and the color written in from the findColor function. Indexes/Positions 3 & 7 not used
    //          3
    //2*--------*--------*4
    // |                 |
    // |                 |
    // |                 |
    // |                 |
    //1*                 *5
    // |                 |
    // |                 |
    // |                 |
    // |                 |
    // *--------*--------*6
    // 0        7
    void findPathReturn(int color, int i, float scale, float leg);
    {
        /*******************************************************************///Red
        if(  i == 0 && color == 1 ) { //condition for return red at index 0
            move((2*leg + 0.3048*scale), FORWARD);
            turnLeft();
            move((0.3048*scale), FORWARD);
            dropToken();
            rot180();
            move((0.3048*scale), FORWARD);
            turnRight();
            move((2*leg + 0.3048*scale), FORWARD);
            rot180();

        }
        if( i == 1 && color == 1 ) { //condition for return red at index 1
            move((leg + 0.3048*scale), FORWARD);
            turnLeft();
            move((0.3048*scale), FORWARD);
            dropToken();
            rot180();
            move((0.3048*scale), FORWARD);
            turnRight();
            move((leg + 0.3048*scale), FORWARD);
            rot180();


        }
        if( i == 2 && color == 1 ) { //condition for return red at index 2
            rot180();
            move((0.3048*scale), FORWARD);
            turnRight();
            move((0.3048*scale), FORWARD);
            dropToken();
            rot180();
            move((0.3048*scale), FORWARD);
            turnLeft();
            move((0.3048*scale), FORWARD);


        }
        if( i == 4 && color == 1 ) { //condition for return red at index 4
            turnRight();
            move(2*leg + 0.3048*scale, FOWARD);
            turnRight();
            move(0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3046*scale, FORWARD);
            turnLeft();
            move(2*leg + 0.3048*scale, FOWARD);
            turnRight();

        }
        if( i == 5 && color == 1 ) {

            rot180();
            move(leg + 0.3046*scale, FORWARD);
            turnLeft();
            move(2*leg + 0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(2*leg + 0.3046*scale, FORWARD);
            turnRight();
            move(leg + 0.3046*scale, FORWARD);
        }
        if( i == 6 && color == 1 ) { //condition for return red at index 5
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD);
            turnLeft();
            move(2*leg + 0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(2*leg + 0.3046*scale, FORWARD);
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD);

        }

        /*******************************************************************///Green
        if( i == 0 && color == 2 ) { //condition for return Green at index 0
            move(leg, FORWARD);
            turnLeft();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnRight();
            move(leg, FORWARD);
            rot180();

        }
        if( i == 1 && color == 2 ) { //condition for return green at index 1
            turnLeft();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnLeft();

        }
        if( i == 2 && color == 2 ) { //condition for return green at index 2
            turnRight();
            move(leg, FORWARD);
            turnRight();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnLeft();
            move(leg, FORWARD);
            turnRight();

        }

        if( i == 4 && color == 2 ) { //condition for return green at index 4
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD);
            turnLeft();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORWARD);
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD);
            turnRight();



        }
        if( i == 5 && color == 2 ) { //condition for return green at index 5
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(2*leg + 0.3046*scale, FORWARD);
            turnRight();

        }
        if( i == 6 && color == 2 ) { //condition for return green at index 6
            move(2*leg + 0.3046*scale, FORWARD);
            turnRight();
            move(leg + 0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(leg + 0.3046*scale, FORWARD);
            turnLeft();
            move(leg + 0.3046*scale, FORWARD);
            rot180();
        }
        /*******************************************************************///Blue
        if( i == 0 && color == 3 ) { //condition for return Blue at index 0 ::
            rot180();
            move(0.3048*scale, FORWARD);
            turnRight();
            move(0.3048*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3048*scale, FORWARD);
            turnLeft();
            move(0.3048*scale, FORWARD);

        }
        if( i == 1 && color == 3 ) { //condition for return blue at index 1
            rot180();
            move((leg + 0.3048*scale), FORWARD);
            turnRight();
            move(0.3048*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3048*scale, FORWARD);
            turnLeft();
            move((leg + 0.3048*scale), FORWARD);

        }
        if( i == 2 && color == 3 ) { //condition for return blue at index 2
            turnRight();
            move((2*leg + 0.3048*scale), FORWARD);
            turnRight();
            move(0.3048*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3048*scale, FORWARD);
            turnLeft();
            move((2*leg + 0.3048*scale), FORWARD);
            turnRight();


        }
        if( i == 4 && color == 3 ) { //condition for return blue at index 4
            turnRight();
            move((2*leg + 0.3048*scale), FORWARD);
            turnRight();
            move((2*leg) + (0.3048*scale), FORWARD);
            dropToken();
            rot180();
            move((2*leg + 0.3048*scale), FORWARD);
            turnLeft();
            move((2*leg) + (0.3048*scale), FORWARD);
            turnRight();


        }
        if( i == 5 && color == 3 ) { //condition for return blue at index 5

            move(leg + 0.3048*scale, FORWARD);
            turnRight();
            move(2*leg + 0.3048*scale, FORWARD);
            dropToken();
            rot180();
            move(2*leg + 0.3048*scale, FORWARD);
            turnLeft();
            move(leg + 0.3048*scale, FORWARD);
            rot180();




        }
        if( i == 6 && color == 3 ) { //condition for return blue at index 5
            move(2*leg + 0.3048*scale, FORWARD);
            turnLeft();
            move(0.3048*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3048*scale, FORWARD);
            turnRight();
            move(2*leg + 0.3048*scale, FORWARD);
            rot180();

        }
        /*******************************************************************///Cyan
        if( i == 0 && color == 4 ) { //condition for return Cyan at index 0
            move(2*leg + 0.3046*scale, FORWARD)
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD)
            dropToken();
            rot180();
            move(2*leg + 0.3046*scale, FORWARD)
            turnLeft();
            move(2*leg + 0.3046*scale, FORWARD)
            rot180();
        }
        if( i == 1 && color == 4 ) { //condition for return Cyan at index 1
            move(leg + 0.3046*scale, FOWARD);
            turnRight();
            move(2*leg + 0.3046*scale, FOWARD);
            dropToken();
            rot180();
            move(2*leg + 0.3046*scale, FOWARD);
            turnleft();
            move(leg + 0.3046*scale, FOWARD);
            rot180();

        }
        if( i == 2 && color == 4 ) { //condition for return Cyan at index 2
            move(2*leg + 0.3046*scale, FOWARD);
            turnLeft();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnRight();
            move(2*leg + 0.3046*scale, FOWARD);
            rot180();

        }
        if( i == 4 && color == 4 ) { //condition for return Cyan at index 4
            rot180();
            move(0.3046*scale, FORWARD);
            turnRight();
            move(0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3046*scale, FORWARD);
            turnLeft();
            move(0.3046*scale, FORWARD);
            turnleft();
            move(0.3046*scale, FORWARD);
            turnRight();

        }
        if( i == 5 && color == 4 ) { //condition for return Cyan at index 5
            rot180();
            move(leg + 0.3046*scale, FORWARD);
            turnRight();
            move(0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3046*scale, FORWARD);
            turnLeft();
            move(leg + 0.3046*scale, FORWARD);

        }
        if( i == 6 && color == 4 ) { //condition for return Cyan at index 6
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD);
            turnRight();
            move(0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3046*scale, FORWARD);
            turnLeft();
            move(leg + 0.3046*scale, FORWARD);
            turnRight();

        }
        /*******************************************************************///Magenta
        if( i == 0 && color == 5 ) { //condition for return Magenta at index 0
            move(leg, FORWARD);
            turnRight();
            move(2*leg + 0.3046*scale);
            dropToken();
            rot180();
            move(2*leg + 0.3046*scale);
            turnLeft();
            move(leg, FORWARD);
            rot180();

        }
        if( i == 1 && color == 5 ) { //condition for return Magenta at index 1
            turnRight();
            move(2*leg + 0.3046*scale);
            dropToken();
            rot180();
            move(2*leg + 0.3046*scale);
            turnRight();

        }
        if( i == 2 && color == 5 ) { //condition for return Magenta at index 2
            move(leg + 0.3046*scale, FORWARD);
            turnRight();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORWARD);
            turnLeft();
            move(2*leg + 0.3046*scalem FORWARD);
            rot180();

        }
        if( i == 4 && color == 5 ) { //condition for return Magenta at index 4
            move(leg, FORWARD);
            turnLeft();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnRight();
            Move(leg, FORWARD);
            rot180();

        }
        if( i == 5 && color == 5 ) { //condition for return Magenta at index 5
            turnLeft();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnLeft();

        }
        if( i == 6 && color == 5 ) { //condition for return Magenta at index 6
            rot180();
            move(leg, FORWARD);
            turnRight();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnLeft();
            move(lef, FORWARD);
            turnRight();

        }
        /*******************************************************************///Yellow
        if( i == 0 && color == 6 ) { //condition for return Yellow at index 0
            turnRight();
            move(2*leg + 0.3046*scale);
            turnRight();
            move(0.3046*scale);
            dropToken();
            rot180();
            move(0.3046*scale);
            turnLeft();
            move(2*leg + 0.3046*scale);
            turnRight();

        }
        if( i == 1 && color == 6 ) { //condition for return Yellow at index 1
            rot180();
            move(leg + 0.3046*scale, FORWARD);
            turnLeft();
            move(2*leg, FORWARD);
            dropToken();
            rot180();
            move(2*leg, FOWARD);
            turnRight();
            move(leg + 0.3046*scale, FORWARD);


        }
        if( i == 2 && color == 6 ) { //condition for return Yellow at index 2
            move(2*leg + 0.3046*scale, FORWARD);
            turnRight();
            move(2*leg, FORWARD);
            dropToken();
            rot180();
            move(2*leg, FORWARD);
            turnLeft();
            move(2*leg + 0.3046*scale, FORWARD);
            rot180();

        }
        if( i == 4 && color == 6 ) { //condition for return Yellow at index 4
            move(2*leg + 0.3046*scale, FORWARD);
            turnLeft();
            move(0.3046*scale, FORWARD);
            dropToken();
            rot180(0);
            move(0.3046*scale, FORWARD);
            turnRight();
            move(2*leg + 0.3046*scale, FORWARD);
            rot180();


        }
        if( i == 5 && color == 6 ) { //condition for return Yellow at index 5
            move(leg + 0.3046*scale, FORWARD);
            turnLeft();
            move(0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3046*scale, FORWARD);
            turnRight();
            move(Leg + 0.3046*scale, FORWARD);
            rot180();


        }
        if( i == 6 && color == 6 ) { //condition for return Yellow at index 6
            rot180();
            move(0.3046*scale, FORWARD);
            turnRight();
            move(0.3046*scale, FORWARD);
            dropToken();
            rot180();
            move(0.3046*scale, FORWARD);
            turnLeft();
            move(0.3046*scale, FORWARD);

        }
        /*******************************************************************///Gray
        if( i == 0 && color == 7 ) { //condition for return Gray at index 0
            move(leg, FORWARD);
            turnRight();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORWARD);
            turnLeft();
            move(leg, FORWARD);


        }
        if( i == 1 && color == 7 ) { //condition for return Gray at index 1
            turnRight();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORWARD);
            turnRight();
        }
        if( i == 2 && color == 7 ) { //condition for return Gray at index 2
            rot180();
            move(leg, FORWARD);
            turnLeft();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORWARD);
            turnRight();
            move(leg, FORWARD);
            turnRight();

        }
        if( i == 4 && color == 7 ) { //condition for return Gray at index 4
            move(leg, FORWARD);
            turnRight();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORARD);
            turnLeft();
            move(lef, FORWARD);
            rot180();

        }
        if( i == 5 && color == 7 ) { //condition for return Gray at index 5
            turnRight();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORWARD);
            turnRight();

        }
        if( i == 6 && color == 7 ) { //condition for return Gray at index 6
            turnRight();
            move(leg, FORWARD);
            turnLeft();
            move(leg, FORWARD);
            dropToken();
            rot180();
            move(leg, FORWARD);
            rightRight();
            move(leg, FORWARD);
            turnRight();

        } else
            {}
        /*******************************************************************/

    }
}
void returnHome(){ //Returns to the home white square


}
