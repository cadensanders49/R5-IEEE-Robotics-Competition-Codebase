#include "mbed.h"
//
//
//
//   PIN DECLARATIONS
//
//
DigitalOut FLdirection(PTB18);    //Front left motor direction pin
DigitalOut FRdirection(PTA4);     //Front right motor direction pin
DigitalOut magDirection(PTB19);   //Magnet arm motor direction pin
PwmOut stepFL(PTD3);              //Front left motor pwm speed pin
PwmOut stepFR(PTA5);              //Front right motor pwm speed pin
PwmOut magArm(PTA12);             //Magnet arm motor pwm speed pin
InterruptIn killAll(PTC3);        //Button for the kill switch
DigitalIn Start(PTC12);           //Button for starting the program
DigitalOut enableH(PTC11);        //Enable pin for the H-Brdige, turns magnet on or off
DigitalOut highH(PTC10);          //One of the current direction control pins on the H-Bridge
DigitalOut highL(PTC7);           //One of the current direction control pins on the H-Bridge
I2C i2c(PTC9, PTC8);              //pins for I2C communication (SDA, SCL)
Serial pc(USBTX, USBRX);          //Serial computer screen printing connection
DigitalOut LED(PTC4);             //LED control pin for the RGB sensor
DigitalOut green(LED_GREEN);      //Control pin for the RGB sensor
//
//
//   PROTOTYPE FUNCTION DECLARATIONS
//
//
void move(double dist, bool direction);                        //Moves the robot in a certain direction a certain distance in meters
void grabToken();                                             //Picks up the token for reading
void dropToken();                                             //Drops the token off
void kill();                                                  //Stops the robot
void turnRight();                                             //Makes the robot turn 90 degrees to the right
void turnLeft();                                              //Makes the robot turn 90 degrees to the left
void rot180();                                                //Turns the robot 180 degrees around
int findColor();                                              //Figures out what color the token is and returns an integer that represents the color of the token
void findPathReturn(int color, int i, double rightsScale, double leftScale, double straightLeg, double straightScale, double leftLeg, double rightLeg);    //Determines where to take the token based on the color and its relative location and then returns to the previous position
void returnHome();                                            //Returns to the home white square
//
//
//    GLOBAL CONSTANTS
//
//
const int FORWARD = 0;                //Sets a constant for choosing the forward direction
const int BACKWARD = 1;               //Sets a constant for choosing the forward direction
const double stepSize = 0.004090615436;    //In feet
const double FREQUENCY = 500.0;          //Steps per second
const int sensor_addr = 41 << 1;            //Calibration for the rgb sensor
const int TIME = 100;                       //In seconds, this is where you input the round time duration
Timeout timer;                        //Attach this to return home function, set according to round time
const double pi = 3.141592653589;
const double A = 0.13021;    // radius of wheel in feet
const double B = 0.61458;    // end of left wheel to middle of magnet in feet
const double C = 0.77604;    // length from wheel to wheel in feet
const double D = 0.22396;    // length from right wheel to middle of magnet in feet
const double E = 0.19792;    // length from right wheel to middle of magnet in feet
const double H = 0.16666;    // length from home square edge
const double J = C - (J/2);  // side to side position of the robot in home square
//
//
//   Main Function
//
//
int main()
{
    //
    //
    //   Start a timer
    //
    //
    timer.attach(&returnHome, TIME);
    //
    //
    //   Initialize some other variable
    //
    //
    enableH = 0;          // Making sure the H-Bridge starts low and off
    highH = 0;            // This starts high for the H-Bridge
    highL = 1;            // This starts low for the H-Bridge
    double straightScale;          // A variable to scale the box size
    double rightScale;
    double leftScale;
    int color;            // A variable to hold the color value
    int round = 1;        // Round number
    double straightLeg;
    double rightLeg;
    double leftLeg;
    //
    //
    //The start button
    //
    //
    while(true)
    {
        if (Start == 0)
            break;
    }
    //
    //
    //   The kill interupt
    //
    //
    killAll.rise(&kill);
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
    if (data[0]==68)
    {
        green = 0;
        wait (2);
        green = 1;
    }
    else
    {
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
    move(2.0283333+H+A-D,FORWARD);
    wait(2.5);
    turnRight();
    wait(2.5);
    move(0.47916667+J+E,FORWARD);
    //
    //
    //   Mapping algorithm
    //
    //
    while(true) {
        //
        //   Chooses which boxes to search for each round
        //
        for(int j = 4; j>0; j--)
        {
          straightLeg = (j/2)+0.5;   // length to travel if not turning
          rightLeg = straightLeg - D - E;      // length to travel before a right turn
          leftLeg = straightLeg - B - E;       // length to travel before a left turn
          straightScale = -(0.5*j)+3;
          rightScale = -(0.5*j)+3 - D - E;
          leftScale = -(0.5*j)+3 - B - E;
          //
          //   Round 1 Box Choices
          //
          if(round == 1)
          {
            if (j == 4 || j ==2)
            {
              move(0.5-D-B,FORWARD);
              turnRight();
              move(0.5-D-B,FORWARD);
              turnLeft();
              continue;
            }
          }
          //
          //   Round 2 Choices
          //
          else if(round == 2)
          {
            if (j == 4)
            {
              move(0.5-D-B,FORWARD);
              turnRight();
              move(0.5-D-B,FORWARD);
              turnLeft();
              continue;
            }
          }
          //
          //   Token Drop Off Algorithm
          //
          for(int i = 0; i <=7; i++)
          {
              if(i % 2 == 0 && i != 0)
              {
                  turnRight();
              }
              if(i != 3 && i != 7)
              {
                grabToken();
                color = findColor();
                if (color == 9)
                {
                  if(i % 2 == 0)
                  {
                    move(straightLeg, FORWARD); //straight leg
                  }
                  else
                  {
                    move(rightLeg, FORWARD); //right leg
                  }
                }
                else
                {
                    findPathReturn(color, i, rightScale, leftScale, straightLeg, straightScale, leftLeg, rightLeg);
                    if(i % 2 == 0)
                    {
                      move(straightLeg, FORWARD); //straight leg
                    }
                    else
                    {
                      move(rightLeg, FORWARD); //right leg
                    }
                }
              }
          }
        }
    }
}
//
// ..........................
// .... Helper Functions ....
// ..........................
//
//
//
//   Move Function
//   Moves the robot (feet)
//
void move(double dist, bool direction)
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
//   Grab Token Function
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
    wait(0.62);
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
//   Drop Token Function
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
    wait(0.62);
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
//   Turn Right Function
//
//
void turnRight()
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
    wait(4*(((2*pi*C)/4)/stepSize)*(1/FREQUENCY)); //(dist/stepSize) is the number of steps; 1/FREQUENCY is the time per step
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
//   Turn Left Function
//
//
void turnLeft()
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
    wait(4*(((2*pi*C)/4)/stepSize)*(1/FREQUENCY)); //(dist/stepSize) is the number of steps; 1/FREQUENCY is the time per step
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
//   Rotate 180 Degrees Function
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
    //   Turns 270 right
    //
    //
    wait(12*(((2*pi*C)/4)/stepSize)*(1/FREQUENCY)); //(dist/stepSize) is the number of steps; 1/FREQUENCY is the time per step
    //
    //
    wait(0.2);
    //  Turn left 90 to complete 180
    turnLeft();
    wait(0.2);
    //
    //   Stop motor
    //
    //
    stepFL.period(0);
    stepFL.write(0);
}
//
//
//   Kill Switch Function
//   Stops the machine
//
void kill()
{
    exit(0);
}
//
//
//   Find Color Function
//   Figures out what color the disk is and makes a decision on where to take the disk
//
int findColor()
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
        //
        //
        //   Return the color value from the reading
        //
        //
        //   1=red, 2=green, 3=blue, 4=cyan, 5=magenta, 6=yellow, 7=gray, 8=error, 9=nothing
        //
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
        //
        //
        // print sensor readings
        //
        //
        //pc.printf("Clear (%d), Red (%d), Green (%d), Blue (%d)\n", clear_value, red_value, green_value, blue_value);
        //wait(0.5);
    }
}
//
//
//   Find Path Return Function
//   This next function decides where to go based on its reletive position and
//   the color written in from the findColor function.
//   Indexes/Positions 3 & 7 not used.
//
//    2         3         4
//     *--------*--------*
//     |                 |
//     |                 |
//     |                 |
//     |                 |
//   1 *                 * 5
//     |                 |
//     |                 |
//     |                 |
//     |                 |
//     *--------*--------*
//    0        7         6
//
//
void findPathReturn(int color, int i, double rightsScale, double leftScale, double straightLeg, double straightScale, double leftLeg, double rightLeg)
{
    //
    //
    //   RED
    //
    //
    //   condition for return red at index 0
    //
    if(  i == 0 && color == 1 )
    {
        move((2*straightLeg + leftScale), FORWARD);
        turnLeft();
        move(straightScale - B + E, FORWARD);
        dropToken();
        move(straightScale + B + E, BACKWARD);
        turnLeft();
        move(2*straightLeg - B + E + straightScale, FORWARD);
        rot180();
        move(C,BACKWARD);
    }
    //
    //   condition for return red at index 1
    //
    if( i == 1 && color == 1 )
    {
        move((straightLeg + straightScale), FORWARD);
        turnLeft();
        move((straightScale), FORWARD);
        dropToken();
        rot180();
        move((straightScale), FORWARD);
        turnRight();
        move((straightLeg + straightScale), FORWARD);
        rot180();
    }
    //
    //   condition for return red at index 2
    //
    if( i == 2 && color == 1 )
    {
        rot180();
        move((straightScale), FORWARD);
        turnRight();
        move((straightScale), FORWARD);
        dropToken();
        rot180();
        move((straightScale), FORWARD);
        turnLeft();
        move((straightScale), FORWARD);
    }
    //
    //   condition for return red at index 4
    //
    if( i == 4 && color == 1 )
    {
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
    }
    //
    //   condition for return red at index 5
    //
    if( i == 5 && color == 1 )
    {
        rot180();
        move(straightLeg + straightScale, FORWARD);
        turnLeft();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(straightLeg + straightScale, FORWARD);
    }
    //
    //   condition for return red at index 6
    //
    if( i == 6 && color == 1 )
    {
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
    }
    //
    //
    //   GREEN
    //
    //
    //   condition for return green at index 0
    //
    if( i == 0 && color == 2 )
    {
        move(straightLeg, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(straightLeg, FORWARD);
        rot180();
    }
    //
    //   condition for return green at index 1
    //
    if( i == 1 && color == 2 )
    {
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
    }
    //
    //   condition for return green at index 2
    //
    if( i == 2 && color == 2 )
    {
        turnRight();
        move(straightLeg, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
        turnRight();
    }
    //
    //   condition for return green at index 4
    //
    if( i == 4 && color == 2 )
    {
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
    }
    //
    //   condition for return green at index 5
    //
    if( i == 5 && color == 2 )
    {
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
    }
    //
    //   condition for return green at index 6
    //
    if( i == 6 && color == 2 )
    {
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightLeg + straightScale, FORWARD);
        rot180();
    }
    //
    //
    //   BLUE
    //
    //
    //   condition for return blue at index 0
    //
    if( i == 0 && color == 3 )
    {
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
    }
    //
    //   condition for return blue at index 1
    //
    if( i == 1 && color == 3 )
    {
        rot180();
        move((straightLeg + straightScale), FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move((straightLeg + straightScale), FORWARD);
    }
    //
    //   condition for return blue at index 2
    //
    if( i == 2 && color == 3 )
    {
        turnRight();
        move((2*straightLeg + straightScale), FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move((2*straightLeg + straightScale), FORWARD);
        turnRight();
    }
    //
    //   condition for return blue at index 4
    //
    if( i == 4 && color == 3 )
    {
        turnRight();
        move((2*straightLeg + straightScale), FORWARD);
        turnRight();
        move((2*straightLeg) + (straightScale), FORWARD);
        dropToken();
        rot180();
        move((2*straightLeg + straightScale), FORWARD);
        turnLeft();
        move((2*straightLeg) + (straightScale), FORWARD);
        turnRight();
    }
    //
    //   condition for return blue at index 5
    //
    if( i == 5 && color == 3 )
    {
        move(straightLeg + straightScale, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightLeg + straightScale, FORWARD);
        rot180();
    }
    //
    //   condition for return blue at index 6
    //
    if( i == 6 && color == 3 )
    {
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        rot180();
    }
    //
    //
    //   CYAN
    //
    //
    //   condition for return cyan at index 0
    //
    if( i == 0 && color == 4 )
    {
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(2*straightLeg + straightScale, FORWARD);
        rot180();
    }
    //
    //   condition for return Cyan at index 1
    //
    if( i == 1 && color == 4 )
    {
        move(straightLeg + straightScale, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightLeg + straightScale, FORWARD);
        rot180();

    }
    //
    //   condition for return Cyan at index 2
    //
    if( i == 2 && color == 4 )
    {
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        rot180();

    }
    //
    //   condition for return Cyan at index 4
    //
    if( i == 4 && color == 4 )
    {
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        turnRight();

    }
    //
    //   condition for return Cyan at index 5
    //
    if( i == 5 && color == 4 )
    {
        rot180();
        move(straightLeg + straightScale, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightLeg + straightScale, FORWARD);

    }
    //
    //   condition for return Cyan at index 6
    //
    if( i == 6 && color == 4 )
    {
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightLeg + straightScale, FORWARD);
        turnRight();

    }
    //
    //
    //   MAGENTA
    //
    //
    //   condition for return magenta at index 0
    //
    if( i == 0 && color == 5 )
    {
        move(straightLeg, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
        rot180();
    }
    //
    //   condition for return Magenta at index 1
    //
    if( i == 1 && color == 5 )
    {
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();

    }
    //
    //   condition for return Magenta at index 2
    //
    if( i == 2 && color == 5 )
    {
        move(straightLeg + straightScale, FORWARD);
        turnRight();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnLeft();
        move(2*straightLeg + straightScale, FORWARD);
        rot180();

    }
    //
    //   condition for return Magenta at index 4
    //
    if( i == 4 && color == 5 )
    {
        move(straightLeg, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(straightLeg, FORWARD);
        rot180();
    }
    //
    //   condition for return Magenta at index 5
    //
    if( i == 5 && color == 5 )
    {
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
    }
    //
    //   condition for return Magenta at index 6
    //
    if( i == 6 && color == 5 )
    {
        rot180();
        move(straightLeg, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
        turnRight();
    }
    //
    //
    //   YELLOW
    //
    //
    //   condition for return yellow at index 0
    //
    if( i == 0 && color == 6 )
    {
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
    }
    //
    //   condition for return Yellow at index 1
    //
    if( i == 1 && color == 6 )
    {
        rot180();
        move(straightLeg + straightScale, FORWARD);
        turnLeft();
        move(2*straightLeg, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg, FORWARD);
        turnRight();
        move(straightLeg + straightScale, FORWARD);
    }
    //
    //   condition for return Yellow at index 2
    //
    if( i == 2 && color == 6 )
    {
        move(2*straightLeg + straightScale, FORWARD);
        turnRight();
        move(2*straightLeg, FORWARD);
        dropToken();
        rot180();
        move(2*straightLeg, FORWARD);
        turnLeft();
        move(2*straightLeg + straightScale, FORWARD);
        rot180();
    }
    //
    //   condition for return Yellow at index 4
    //
    if( i == 4 && color == 6 )
    {
        move(2*straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(2*straightLeg + straightScale, FORWARD);
        rot180();
    }
    //
    //   condition for return Yellow at index 5
    //
    if( i == 5 && color == 6 )
    {
        move(straightLeg + straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(straightLeg + straightScale, FORWARD);
        rot180();
    }
    //
    //   condition for return Yellow at index 6
    //
    if( i == 6 && color == 6 )
    {
        rot180();
        move(straightScale, FORWARD);
        turnRight();
        move(straightScale, FORWARD);
        dropToken();
        rot180();
        move(straightScale, FORWARD);
        turnLeft();
        move(straightScale, FORWARD);
    }
    //
    //
    //   GRAY
    //
    //
    //   condition for return gray at index 0
    //
    if( i == 0 && color == 7 )
    {
        move(straightLeg, FORWARD);
        turnRight();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
    }
    //
    //   condition for return Gray at index 1
    //
    if( i == 1 && color == 7 )
    {
        turnRight();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnRight();
    }
    //
    //   condition for return Gray at index 2
    //
    if( i == 2 && color == 7 )
    {
        rot180();
        move(straightLeg, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnRight();
        move(straightLeg, FORWARD);
        turnRight();
    }
    //
    //   condition for return Gray at index 4
    //
    if( i == 4 && color == 7 )
    {
        move(straightLeg, FORWARD);
        turnRight();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
        rot180();
    }
    //
    //   condition for return Gray at index 5
    //
    if( i == 5 && color == 7 )
    {
        turnRight();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnRight();
    }
    //
    //   condition for return Gray at index 6
    //
    if( i == 6 && color == 7 )
    {
        turnRight();
        move(straightLeg, FORWARD);
        turnLeft();
        move(straightLeg, FORWARD);
        dropToken();
        rot180();
        move(straightLeg, FORWARD);
        turnRight();
        move(straightLeg, FORWARD);
        turnRight();
    }
    //
    //  ??
    //
    else
    {

    }
}
//
//
//   Return Home Function
//   Returns to the home white square
//
void returnHome()
{
// Caden is booty
}



/*

cout<<" /dNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNm+"<<endl;
cout<<"mMs::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::sMN"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mM+                                     -s-                                                      +MM"<<endl;
cout<<"mM+                                     /NN:                                                     +MM"<<endl;
cout<<"mM+                                      -yN+                                                    +MM"<<endl;
cout<<"mM+                                       `+Mo                                                   +MM"<<endl;
cout<<"mM+                                      `/yy-                                                   +MM"<<endl;
cout<<"mM+                             `..`   `+yo.                                                     +MM"<<endl;
cout<<"mM+                          `/ydmmmd:`o+.                                    ``..--             +MM"<<endl;
cout<<"mM+                         -dMMMMMMy/ymmdy:                      ``..-:/+osyhdmmmds             +MM"<<endl;
cout<<"mM+                    `:oo`mMMMMMMs/NMMMMMN/         ```.--:/osyhhhhhhyyyhdNmds/-.-+`           +MM"<<endl;
cout<<"mM+                  `+dNs`:MMMMMMMNNMMMMMMMm `-:/+ssyhhhhhyyo+/:--..-/shdho:..-ohmMN`           +MM"<<endl;
cout<<"mM+                  yMMs  /MMMMMMMMMMMMMMMMm :dNmyo+:-.``      `-+yhdy+-..-ohmMMMMMN`           +MM"<<endl;
cout<<"mM+                ``oMMs  `mMMMMMMMMMMMMMMMs   :NMNm:     `.:oydds/-``:ohmMMMMMMMMMN`           +MM"<<endl;
cout<<"mM+           -+syyhh-+NN.  -dNMMMMMMMMMMNm+   `+NNm+` `./shdho:.``:ohmMMMMMMMMMMMMMN`           +MM"<<endl;
cout<<"mM+           yMN+-..  -hs  +y/ohdmNNmdy+:`  `odms:.-+yhdy+:```:ohNMMMMMMMMMMMMMMMNdo            +MM"<<endl;
cout<<"mM+           yMdms-`    .  `ym+.``.-:/      :+::oydds/-` `:ohNMMMMMMMMMMMMMMMNds:.              +MM"<<endl;
cout<<"mM+           yM.-smds/-``    +mNmdmds-   `./shmho:.  `:ohNMMMMMMMMMMMMMMMNdo:.   `-/`           +MM"<<endl;
cout<<"mM+           yM.   -+sdmdhsso++yhy+////+ydmy+-`  `:odNMMMMMMMMMMMMMMMNdo:`   `-+ymMN`           +MM"<<endl;
cout<<"mM+           yM.        .-:/+ossssysssss/.   `:sdNMMMMMMMMMMMMMMMNdo:`   `-ohNMMMMMN`           +MM"<<endl;
cout<<"mM+           yM.                         `:sdNMMMMMMMMMMMMMMMNdo:`   `-ohNMMMMMMMMMN`           +MM"<<endl;
cout<<"mM+           yM.                        yMMMMMMMMMMMMMMMMNho-`   `-ohNMMMMMMMMMMMMMN`           +MM"<<endl;
cout<<"mM+           yM.                       `mMMMMMMMMMMMMNho-`   `-ohNMMMMMMMMMMMMMMMMdo            +MM"<<endl;
cout<<"mM+           yM.                       `mMMMMMMMMNh+-`   `:ohNMMMMMMMMMMMMMMMNds:`              +MM"<<endl;
cout<<"mM+           yM.                       `mMMMMNho-`   `:odNMMMMMMMMMMMMMMMNdo:`  ./yd`           +MM"<<endl;
cout<<"mM+           yM.                       `mmy+-`   `:ohNMMMMMMMMMMMMMMMNho:`  -/ymNMMN`           +MM"<<endl;
cout<<"mM+           yM.                        .`   .:odNMMMMMMMMMMMMMMMNho:` `-+ymNMMMMMMN`           +MM"<<endl;
cout<<"mM+           yM.                         .:sdNMMMMMMMMMMMMMMMNho:```-+ymNMMMMMMMMMMN`           +MM"<<endl;
cout<<"mM+           yM.                        yNMMMMMMMMMMMMMMMNho:.``-+ymNMMMMMMMMMMMMMMm`           +MM"<<endl;
cout<<"mM+           yM.                       `mMMMMMMMMMMMMmho:. `-+ymNMMMMMMMMMMMMMMNdy/.            +MM"<<endl;
cout<<"mM+           yM.                       `mMMMMMMMMmho:.``-+ymNMMMMMMMMMMMMMMNdy/-`               +MM"<<endl;
cout<<"mM+           yM.                       `mMMMMmho:. `-+ymNMMMMMMMMMMMMMMNds/.`                   +MM"<<endl;
cout<<"mM+           yM.                       `dmho:.``-+ymNMMMMMMMMMMMMMMNdy/.`                       +MM"<<endl;
cout<<"mM+           yM.                        .. .-+ymNMMMMMMMMMMMMMMNds/.`                           +MM"<<endl;
cout<<"mM+           yM.                        -+hmNMMMMMMMMMMMMMMNds/-`                               +MM"<<endl;
cout<<"mM+           +M+                       `dMMMMMMMMMMMMMMNds/.`                                   +MM"<<endl;
cout<<"mM+           `sNo.                     `mMMMMMMMMMMNds/.`                                       +MM"<<endl;
cout<<"mM+             :hdy:.`                 `mMMMMMMNds/.                                            +MM"<<endl;
cout<<"mM+               -+hdhyo/:-...````````..mMMNds/.                                                +MM"<<endl;
cout<<"mM+                  `-:+syhdhhhhhhhhhhhhhs/.                                                    +MM"<<endl;
cout<<"mM+                           ``````````                                                         +MM"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mM+                                                                                              +MM"<<endl;
cout<<"mMs----------------------------------------------------------------------------------------------oMN"<<endl;
cout<<"/mMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMm+"<<endl;
cout<<"the cake is a lie"<<endl;

*/
