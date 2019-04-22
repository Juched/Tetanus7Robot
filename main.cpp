#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <cstdlib>
#include <math.h>

AnalogInputPin cds(FEHIO::P1_0);
FEHMotor right_motor(FEHMotor::Motor0,9);
FEHMotor left_motor(FEHMotor::Motor1,9);
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_7);
FEHServo coin_servo(FEHServo::Servo0);
FEHServo foos_servo(FEHServo::Servo1);


float light = 1.7 ;
float smallLight;
float ddrLight ;
double encInch = 40.5;
int turnSpeed = 40;
float currHeading = 0;
float coinServoMax = 2300;
float coinServoMin = 500;
float foosServoMax = 2450;
float foosServoMin = 1192;
float initHeading = 0;
float DDRY = -1;
float DDRX = -1;
float DDRHeading = 90;
float coinX = -1;
float coinY = -1;
float coinHeading = 180;


//int coinX = 0;
//int coinY = 0;

struct data{
    int leftEncoder;
    int rightEncoder;
    int leftSpeed;
    int rightSpeed;
    int totalTicks;
    double RPSx ;
    double RPSy;
    double heading ;
    double expectedHeading;
    double error;
    double cdsVal;
    float cdsLight;};

data printData;
/*resets the motor encoder counts*/
void resetCounts() {
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
}
/* DEBUG FUNCTION
 *  Writes all variables to screen and shows current expected location/error
 * Takes in {@code Data} struct
 */
void writeScreen(data printData){
    LCD.Clear();
    LCD.WriteRC(printData.leftEncoder,1,120);
    LCD.WriteRC(printData.rightEncoder,2,120);
    LCD.WriteRC(printData.leftSpeed,3,120);
    LCD.WriteRC(printData.rightSpeed,4,120);
    LCD.WriteRC(printData.totalTicks,5,120);
    LCD.WriteRC("RPS x: ",6,0 );
    LCD.WriteRC(printData.RPSx,6,120);
    LCD.WriteRC("RPS y: ",7,0 );
    LCD.WriteRC(printData.RPSy,7,120);
    LCD.WriteRC("RPS heading: ",8,0 );
    LCD.WriteRC(printData.heading,8,120);
    LCD.WriteRC("Expected: ", 9, 0);
    LCD.WriteRC(printData.expectedHeading, 9,120);
    LCD.WriteRC("Error: ",10,0 );
    LCD.WriteRC(printData.error,10,120);
    LCD.WriteRC("CDS VAL: ", 12,0);
    LCD.WriteRC(printData.cdsLight,12,120);
}
/* Function to go up ramp
 * This function runs the motors at a high speed with a right skewed curve to run up the wall while going up the ramp.
 * Waits until in RPS Deadzone to disable.
 * BLOCKING
 */
void upRamp() {
    left_motor.SetPercent(85);
    right_motor.SetPercent(80);
    while((RPS.Y() >=  -1));
    left_motor.Stop();
    right_motor.Stop();
}
/*Drive straight for a certain distance PID function
 * Drives straight for the input (tenths of an inch) using PID *
 * BLOCKING (NEEDS ENCODER VALUE TO HIT ABSOLUTE NUMBER)
 */
void driveStraightDistance(double tenthsOfIn, int masterPower)
{
    //calculate distance
    int tickGoal = ( tenthsOfIn/10) * 40.5;
    //This will count up the total encoder ticks despite the fact that the encoders are constantly reset.
    int totalTicks = 0;
    //Initialise slavePower as masterPower - 5 so we don't get huge error for the first few iterations. The
    //-5 value is based off a rough guess of how much the motors are different, which prevents the robot from
    //veering off course at the start of the function.
    double slavePower = masterPower;
    if(masterPower > 0){
        slavePower = masterPower + 2;
    }else{
        slavePower = masterPower - 1;
    }
    //variables needed for calculation
    int error = 0;
    int error_prior = 0;
    int leftPrior = 0;
    int rightPrior = 0;
    //p an D values. ki was not needed.
    double kp = 160;
    //double ki;
    double kd = 0.2 ;
    //reset motors first
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    //Monitor 'totalTicks', instead of the values of the encoders which are constantly reset.
    while(std::abs(totalTicks) < tickGoal){
        //Proportional algorithm to keep the robot going straight.
        left_motor.SetPercent(masterPower);
        right_motor.SetPercent(slavePower);
        error = (left_encoder.Counts()) - (right_encoder.Counts());
        double errAdj = error/kp;
        double kdAdj = kd * (error - error_prior);
        if(masterPower > 0){
            slavePower += errAdj + kdAdj;
        }else{
            slavePower -= errAdj + kdAdj;
        }
        //set motors to highest speed if reached
        if (slavePower < -100){
            slavePower = -100;
        }else if(slavePower> 100){
            slavePower = 100;
        }

        totalTicks = left_encoder.Counts();
        //DEBUG
        printData.leftEncoder = left_encoder.Counts();
        printData.rightEncoder= right_encoder.Counts();
        printData.totalTicks= tickGoal;
        printData.leftSpeed =  masterPower;
        printData.rightSpeed = slavePower;
        printData.error = error;
    //send to debug
        writeScreen(printData);
        //wait for controls cycle
        Sleep(50);
    //find error
        error_prior= error;
        leftPrior = left_encoder.Counts();
        rightPrior= right_encoder.Counts();
        //Add this iteration's encoder values to totalTicks.

    }
    left_motor.Stop(); // Stop the loop once the encoders have counted up the correct number of encoder ticks.
    right_motor.Stop();
}
/*Wait on start light to reach CDS threshold
 * Stops the robot from moving before start light turns on
 * BLOCKING
 */
float CDS() {
    float value = cds.Value();

    while(value > light) {
        value = cds.Value();
    }

    return value;
}
/*Turns both wheels for a one point turn
 *Takes in angle for encoder ticks
 */
void turnBoth(float angle){
    float circ = 3.1415*6.75;//size of wheel circumference
    //make sure motor encoders are reset
    resetCounts();
    //left or right
    if(angle > 0){
        right_motor.SetPercent(-20);
        left_motor.SetPercent(20);
    }else{
        right_motor.SetPercent(20);
        left_motor.SetPercent(-20);
        angle = -angle;
    }
    //calculate counts
    float counts = ((angle/360) * circ) * encInch;
    //wait until turn is over
    while(right_encoder.Counts() < counts || left_encoder.Counts() < counts) {
        if(right_encoder.Counts() >= counts) {
            right_motor.Stop();
        }
        if(left_encoder.Counts() >= counts) {
            left_motor.Stop();
        }
    }
    right_motor.Stop();
    left_motor.Stop();
}

/*Turn using the left motor
 * takes in angle for encoder ticks
 */
void turnRight(float angle) {
    float circ = 3.14159 * (2*6.75);// half robot circumference
    //reset motor counts
    resetCounts();
    float distance;
    //direction of turn
    if(angle >0){
        left_motor.SetPercent(turnSpeed);
        distance = (angle/360)*circ;
    }else{
        left_motor.SetPercent(-turnSpeed);
        distance = (-angle/360)*circ;
    }
    //wait on turn WHILE debug streaming values
    while(left_encoder.Counts()<(distance*encInch)) {
        printData.leftSpeed = turnSpeed;
        printData.leftEncoder = right_encoder.Counts();
        printData.error = (distance*encInch) - left_encoder.Counts();
        printData.heading = RPS.Heading();
        writeScreen(printData);
    }
    left_motor.Stop();
}
/*Turn using the right motor
 * takes in angle for encoder ticks
 */
void turnLeft(float angle) {
    float circ = 3.14159 * (2*6.75);// half robot circumference
//reset motors
    resetCounts();
    float distance;
    //direction of turn
    if(angle >0){
        right_motor.SetPercent(turnSpeed);
        distance = (angle/360)*circ;
    }else{
        right_motor.SetPercent(-turnSpeed);
        distance = (-angle/360)*circ;
    }
    //wait on turn WHILE debug streaming values
    while(right_encoder.Counts()<(distance*encInch)) {
        printData.rightSpeed = turnSpeed;
        printData.rightEncoder = right_encoder.Counts();
        printData.error = (distance*encInch) - right_encoder.Counts();
        printData.heading = RPS.Heading();
        writeScreen(printData);
    }
    right_motor.Stop();
}
/*Turn left and then "square" using RPS
 * BLOCKING
 * DO NOT USE IF NOT OUTSIDE OF DEADZONE *
 */
void turnLeftRPS(float angle){
    //finds RPS heading and uses new one if old prev heading doesn't work
    if(RPS.Heading() > 0){
        currHeading = RPS.Heading();
    }
    //turns towards the angle first
    turnLeft(angle);
    //finds the difference between the current angle and the correct angle
    float adjustedAngle = currHeading + angle;
    if(adjustedAngle < 0){
        adjustedAngle = 360 + adjustedAngle;
    }else if(adjustedAngle > 360){
        adjustedAngle = adjustedAngle - 360;
    }
    //turns in small increments to reach correct angle
    while((adjustedAngle > (RPS.Heading() +1.5))||(adjustedAngle < (RPS.Heading()-1.5))){
        if(adjustedAngle > (RPS.Heading()+1)){
            turnLeft(.5);
            printData.heading = RPS.Heading();
            printData.expectedHeading = adjustedAngle;
            writeScreen(printData);
        }else if(adjustedAngle < (RPS.Heading() -1)){
            turnLeft(-.5);
            printData.heading = RPS.Heading();
            printData.expectedHeading = adjustedAngle;
            writeScreen(printData);
        }
        //wait before using RPS
        Sleep(300);
    }
}
/*Turn right and then "square" using RPS
 * BLOCKING
 * DO NOT USE IF NOT OUTSIDE OF DEADZONE *
 */
void turnRightRPS(float angle){
    //finds RPS heading and uses new one if old prev heading doesn't work
    if(RPS.Heading() > 0){
        currHeading = RPS.Heading();
    }
    //turns towards the angle first
    turnRight(angle);
    //finds the difference between the current angle and the correct angle
    float adjustedAngle = currHeading - angle;
    if(adjustedAngle < 0){
        adjustedAngle = 360 + adjustedAngle;
    }else if(adjustedAngle > 360){
        adjustedAngle = adjustedAngle - 360;
    }
 //turns in small increments to reach correct angle
    while((adjustedAngle> RPS.Heading() +1.5)||(adjustedAngle<RPS.Heading()-1.5)){
        if(adjustedAngle > (RPS.Heading()+1)){
            turnRight(-.5);
            printData.heading = RPS.Heading();
            printData.expectedHeading = adjustedAngle;
            writeScreen(printData);
        }else if(adjustedAngle < (RPS.Heading() -1)){
            turnRight(.5);
            printData.heading = RPS.Heading();
            printData.expectedHeading = adjustedAngle;
            writeScreen(printData);
        }
        Sleep(300);
    }

}
/*Adjusts X distance between robot's pre recorded coinx and current location to line up with coin
 * BLOCKING
 * MAKE SURE YOU ARE WITHIN RPS BEFORE CALLING THIS
 */
void coinFinder(){
    float currX = RPS.X();
    float currY = RPS.Y();
    float currH = RPS.Heading();

    while(fabs(RPS.X() - coinX) > .2){
        if(RPS.X() - coinX > 0){
            driveStraightDistance(1,-30);
        }else{
            driveStraightDistance(1,30);
        }
        Sleep(200);
    }
}
/*Initializes all variables and peripherals then calls CDS function
 * Needs to be by course to use this function
 * BLOCKING
 */
void initialize(){
    //setup servos and LCD
    foos_servo.SetMin((int)foosServoMin);
    foos_servo.SetMax((int)foosServoMax);
    foos_servo.SetDegree(50);
    coin_servo.SetMin((int)coinServoMin);
    coin_servo.SetMax((int)coinServoMax);
    coin_servo.SetDegree(20);
    RPS.InitializeTouchMenu();
    LCD.SetBackgroundColor(WHITE);
    LCD.SetFontColor(BLACK);
    LCD.Clear();
    float x, y;
//gather DDR location from RPS
    while(LCD.Touch(&x,&y)) {}
    while(!LCD.Touch(&x,&y)){
        DDRX = RPS.X();
        DDRY = RPS.Y();
        DDRHeading = RPS.Heading();
        LCD.WriteRC(DDRX, 0, 0);
        LCD.WriteRC(DDRY,1,0);
        LCD.WriteRC(DDRHeading, 2,0);
    }
    Sleep(.1);
    while(LCD.Touch(&x,&y)) {
        LCD.WriteLine("coin");
    }
    while(LCD.Touch(&x,&y)) {
        LCD.WriteLine("waiting on coin again");
    }
    LCD.Clear();
    //gather COIN location from RPS
    while(!LCD.Touch(&x,&y)){
        coinX = RPS.X();
        coinY = RPS.Y();
        coinHeading = RPS.Heading();
        LCD.WriteRC(coinX, 0, 0);
        LCD.WriteRC(coinY,1,0);
        LCD.WriteRC(coinHeading, 2,0);
    }
    Sleep(.5);
    LCD.Clear();
    while(!LCD.Touch(&x,&y));
    Sleep(.5);
    while(LCD.Touch(&x,&y)) {
        LCD.WriteLine("waiting for touch");
    }
    Sleep(1.0);
    while(!LCD.Touch(&x,&y));
    LCD.Clear();
    LCD.SetBackgroundColor(WHITE);
    LCD.SetFontColor(BLACK);
    LCD.WriteLine("ready");
    Sleep(1.0);
    //final start action
    while(!LCD.Touch(&x,&y));
    while(LCD.Touch(&x,&y));
    LCD.SetBackgroundColor(GREEN);
    LCD.Clear();
    CDS();
}
/*Turn both but with RPS
 * Uses RPS to correct heading
 */
void rotateRPS(float start, float end){
    float adjAngLow;
    float adjAngHigh;
    if(start < 180){
        adjAngHigh = 360 + start;
        adjAngLow = start;
    }else{
        adjAngLow = -1*(360 - start);
        adjAngHigh = start;
    }
    float highLim = end + .2;
    float lowLim = end - .2;
    if(highLim > 359.99){
        highLim = highLim - 360;
    }
    if(lowLim < 0){
        lowLim = 360 - lowLim;
    }
    if(!(((end <=  1) && (end < -0.1)) ||((end >= 359)&&(end <=360)))){
        while(!(RPS.Heading() > lowLim) && (RPS.Heading() < highLim)){
            if(RPS.Heading() < 180){
                adjAngHigh = 360 + RPS.Heading();
                adjAngLow = RPS.Heading();
            }else{
                adjAngLow = -1*(360 - RPS.Heading());
                adjAngHigh = RPS.Heading();
            }
            if(fabs(end - adjAngHigh) > fabs(end - adjAngLow)){
                turnRight(.4);
            }else{
                turnLeft(.4);
            }
            Sleep(300);
        }
    }else{
        while(!(((RPS.Heading() >= 0) && (RPS.Heading() <= .5)) || ((RPS.Heading() >= 359.5) && (RPS.Heading() <= 360)))){
            if(RPS.Heading() < 180){
                adjAngHigh = 360 + RPS.Heading();
                adjAngLow = RPS.Heading();
            }else{
                adjAngLow = -1*(360 - RPS.Heading());
                adjAngHigh = RPS.Heading();
            }
            if(fabs(end - adjAngHigh) > fabs(end - adjAngLow)){
                turnRight(.3);
            }else{
                turnLeft(.3);
            }
            Sleep(100);
        }
    }

}
//Finds DDR using RPS
void DDRRPS () {
    float currX = RPS.X();
    float currY = RPS.Y();
    float currH = RPS.Heading();
    float time = TimeNow();
    smallLight = 3.3;
    float val = 3.3;
//go past the starting light
    driveStraightDistance(20, -60);
    float angAdj = RPS.Heading() - DDRHeading;
    rotateRPS(RPS.Heading(), DDRHeading);
    while(currX < DDRX) {
//drive until wall
        driveStraightDistance(50,-50);
        rotateRPS(RPS.Heading(), DDRHeading);
        currX = RPS.X();
        currY = RPS.Y();
        currH = RPS.Heading();
        Sleep(200);

    }
    //driveStraightDistance(30, -50);
    Sleep(200);
    //curve to the left or right depending on how far in the Y direction youve gone and read light during curve
    if(RPS.Y() > DDRY){
        right_encoder.ResetCounts();
        left_motor.SetPercent(8);
        right_motor.SetPercent(20);
        while(right_encoder.Counts() < 70 ){
            if(smallLight > cds.Value()){
                smallLight = cds.Value();
                LCD.Clear();
                printData.cdsLight = smallLight;
                writeScreen(printData);
            }
        }
        left_motor.Stop();
        right_motor.Stop();
    }else{
        left_encoder.ResetCounts();
        left_motor.SetPercent(20);
        right_motor.SetPercent(8);
        while(left_encoder.Counts() < 70 ){
            if(smallLight > cds.Value()){
                smallLight = cds.Value();
                LCD.Clear();
                printData.cdsLight = smallLight;
                writeScreen(printData);
            }
        }
        left_motor.Stop();
        right_motor.Stop();
    }

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    driveStraightDistance(30, -60); //square
}
/*If light is blue// routine
 */
void blue() {
    driveStraightDistance(10,15);
    turnRightRPS(90);
    driveStraightDistance(70,-40);
    left_motor.SetPercent(-70);
    right_motor.SetPercent(-65);
    Sleep(5.5);
    left_motor.Stop();
    right_motor.Stop();
    turnRight(7);
    driveStraightDistance(80,40);
    upRamp();
}
/*If light is red// routine
 */
void red() {
    driveStraightDistance(88,35);

    turnBoth(90);
    driveStraightDistance(70,-40);
    left_motor.SetPercent(-67);
    right_motor.SetPercent(-74);
    Sleep(5.3);
    left_motor.Stop();
    right_motor.Stop();
    turnRight(45);
    driveStraightDistance(70,35);
    turnRight(-10);
    upRamp();
}
//lever turn method
void lever() {
    int degree = 0;
    while(degree<=20) {
        driveStraightDistance(10,30);
        turnLeft(3);
        degree += 3;
    }
}

int main(void)
{
    initialize();
    LCD.SetBackgroundColor(WHITE);
    LCD.Clear();

    currHeading = RPS.Heading();
    initHeading = currHeading;
    float timeStart = TimeNow();
    //wait for RPS heading or 15 second timeout
    while(currHeading == -1 || currHeading == -2||TimeNow() - timeStart == 15){
        currHeading = RPS.Heading();
    }
    //DDr
    driveStraightDistance(2,15);
    turnLeft(135);
    driveStraightDistance(10,70);
    DDRRPS();
    if(smallLight < .6) {
        printData.cdsLight = smallLight;
        writeScreen(printData);
        red();
    } else {
        printData.cdsLight = smallLight;
        writeScreen(printData);
        blue();
    }

    LCD.Clear();
    //overshoot to the FOOSBALL task
    driveStraightDistance(100,55);
    //back up three inches
    driveStraightDistance(30,-40);
    //drive to the wall
    turnLeft(45);
    driveStraightDistance(6.5,45);
    turnLeft(37.5);    //jolt back towards wall (served as a slight turn when wheels were not on center)
    driveStraightDistance(2,-70);
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    //square
    driveStraightDistance(60,-50);
    //foosball
    driveStraightDistance(38,25);
    foos_servo.SetDegree(178);//178
    Sleep(200);
    foos_servo.SetDegree(172.5);//172
    Sleep(200);
    left_motor.SetPercent(42);
    right_motor.SetPercent(50);
    Sleep(1.25);
    left_motor.Stop();
    right_motor.Stop();
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    Sleep(100);
    foos_servo.SetDegree(20);
    //end foos    
    driveStraightDistance(30,35);
    //drive to lever area
    driveStraightDistance(51,35);
    turnRight(-6);
    turnLeft(35);
    driveStraightDistance(65,45);
    driveStraightDistance(1,-40);
    turnLeft(45);
    //return into RPS
    while(RPS.Y()<0) {
        driveStraightDistance(10,55);
    }
    rotateRPS(RPS.Heading(), 0);
    driveStraightDistance(4,40);
    turnLeft(90);
    //find the coin slot
    coinFinder();
    //turn into it
    turnRight(90);
    driveStraightDistance(50,55);
    //drop
    coin_servo.SetDegree(150);
    driveStraightDistance(20,70);
    //driveStraightDistance(10,-70);
    turnLeft(-90);
    driveStraightDistance(120,50);
    //return
    turnRight(-65);
    driveStraightDistance(40,55);
    turnRight(-12);
    driveStraightDistance(150,70);
    //find final button
    float lastTurn = initHeading + 180;
    if(lastTurn > 360){
        lastTurn = lastTurn - 360;
    }
    //run into it
    driveStraightDistance(200,90);

}


