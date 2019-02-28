#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <cstdlib>

AnalogInputPin cds(FEHIO::P0_3);
FEHMotor right_motor(FEHMotor::Motor0,9);
FEHMotor left_motor(FEHMotor::Motor1,9);
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_7);

float light = 1.7;
double encInch = 40.5;
int turnSpeed = 35;
float initHeading = 0;

struct data{
    int leftEncoder;
    int rightEncoder;
    int leftSpeed;
    int rightSpeed;
    int totalTicks;
    int RPSx ;
    int RPSy;
    int heading ;
    int expectedHeading;
    int error;
};

void resetCounts() {
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
}
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

}

void driveStraightDistance(double tenthsOfIn, int masterPower)
{
  int tickGoal = ( tenthsOfIn/10) * 40.5;

  //This will count up the total encoder ticks despite the fact that the encoders are constantly reset.
  int totalTicks = 0;

  //Initialise slavePower as masterPower - 5 so we don't get huge error for the first few iterations. The
  //-5 value is based off a rough guess of how much the motors are different, which prevents the robot from
  //veering off course at the start of the function.
  double slavePower = masterPower -1;

  int error = 0;
  int error_prior =0;
  int leftPrior = 0;
  int rightPrior = 0;

  double kp = 30;
  //double ki;
  double kd = 0.2 ; //18

  left_encoder.ResetCounts();
  right_encoder.ResetCounts();
  data printData;
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

    if (slavePower < -100){
        slavePower = -100;
    }else if(slavePower> 100){
        slavePower = 100;
    }

    totalTicks = left_encoder.Counts();
    printData.leftEncoder = left_encoder.Counts();
    printData.rightEncoder= right_encoder.Counts();
    printData.totalTicks= tickGoal;
    printData.leftSpeed =  masterPower;
    printData.rightSpeed = slavePower;
    printData.error = error;

    writeScreen(printData);


    Sleep(50);


    error_prior= error;
    leftPrior = left_encoder.Counts();
    rightPrior= right_encoder.Counts();

    //Add this iteration's encoder values to totalTicks.

  }
  left_motor.Stop(); // Stop the loop once the encoders have counted up the correct number of encoder ticks.
  right_motor.Stop();
}

float CDS() {
    float value = cds.Value();

    while(value > light) {
        value = cds.Value();
    }

    return value;
}

void turnRight(float angle) {
    float circ = 3.14159 * (2*6.75);
    data printData;
    resetCounts();
    float distance;
    if(angle >0){
    left_motor.SetPercent(turnSpeed);
    distance = (angle/360)*circ;
    }else{
    left_motor.SetPercent(-turnSpeed);
    distance = (-angle/360)*circ;
    }
    while(left_encoder.Counts()<(distance*encInch)) {
        printData.leftSpeed = turnSpeed;
        printData.leftEncoder = right_encoder.Counts();
        printData.error = (distance*encInch) - left_encoder.Counts();
        printData.heading = RPS.Heading();
        writeScreen(printData);
    }
    left_motor.Stop();

}

void turnLeft(float angle) {
    float circ = 3.14159 * (2*6.75);
    data printData;
    resetCounts();
    float distance;
    if(angle >0){
    right_motor.SetPercent(turnSpeed);
     distance = (angle/360)*circ;
    }else{
    right_motor.SetPercent(-turnSpeed);
     distance = (-angle/360)*circ;
    }
    while(right_encoder.Counts()<(distance*encInch)) {
    printData.rightSpeed = turnSpeed;
    printData.rightEncoder = right_encoder.Counts();
    printData.error = (distance*encInch) - right_encoder.Counts();
    printData.heading = RPS.Heading();
    writeScreen(printData);
    }
    right_motor.Stop();
}

void turnLeftRPS(float angle){
    turnLeft(angle);
    float adjustedAngle = initHeading + angle;
    if(adjustedAngle < 0){
      adjustedAngle = 360 - adjustedAngle;
    }else if(adjustedAngle >= 360){
      adjustedAngle = adjustedAngle - 360;
    }
      data printdata;
    while((adjustedAngle > (RPS.Heading() +1.5))||(adjustedAngle < (RPS.Heading()-1.5))){
        if(adjustedAngle > (RPS.Heading()+1)){
            turnLeft(1);
            printdata.heading = RPS.Heading();
            printdata.expectedHeading = adjustedAngle;
            writeScreen(printdata);
        }else if(adjustedAngle < (RPS.Heading() -1)){
            turnLeft(-1);
            printdata.heading = RPS.Heading();
            printdata.expectedHeading = adjustedAngle;
            writeScreen(printdata);
        }
        Sleep(300);
    }

}
void turnRightRPS(float angle){
    turnRight(angle);
    float adjustedAngle = initHeading + angle;
    if(adjustedAngle < 0){
      adjustedAngle = 360 - adjustedAngle;
    }else if(adjustedAngle >= 360){
      adjustedAngle = adjustedAngle - 360;
    }
    data printdata;
    while((adjustedAngle> RPS.Heading() +1.5)||(adjustedAngle<RPS.Heading()-1.5)){
        if(adjustedAngle > (RPS.Heading()+1)){
            turnRight(-1);
            printdata.heading = RPS.Heading();
            printdata.expectedHeading = adjustedAngle;
            writeScreen(printdata);
        }else if(adjustedAngle < (RPS.Heading() -1)){
            turnRight(1);
            printdata.heading = RPS.Heading();
            printdata.expectedHeading = adjustedAngle;
            writeScreen(printdata);
        }
        Sleep(300);
    }

}

int main(void)
{
  RPS.InitializeTouchMenu();

  LCD.Clear();
  LCD.SetBackgroundColor(WHITE);
  LCD.SetFontColor(BLACK);
  CDS();
  initHeading = RPS.Heading();
 //Distances specified in tenths of an inch.
  turnLeftRPS(135);
  //turnRight(120);

 driveStraightDistance(10,30);
 driveStraightDistance(210,-30);
 while(cds.Value() > 1.2){
     left_motor.SetPercent(-20);
     right_motor.SetPercent(-20);
 }
 left_motor.Stop();
 right_motor.Stop();
 float light_value = cds.Value();
 driveStraightDistance(70,-30);
 if(light_value>.4){
  turnLeft(50);
  Sleep(5.0);
 }else{
  driveStraightDistance(40,30);
  turnLeft(50);
  Sleep(5.0);
 }
 initHeading = RPS.Heading();

 turnLeftRPS(-50);
 turnRightRPS(90);
 driveStraightDistance(300, 50);

  /*
   driveStraightDistance(44,30);
   turnRight(80);
   driveStraightDistance(52,-30);
   turnLeft(-74.5);
   driveStraightDistance(68,-30);
   turnRight(-70);
   driveStraightDistance(35,30);
    */

  /*
  turnLeft(70);
  driveStraightDistance(20,30);
  turnRight(35);
  driveStraightDistance(280,60);
  driveStraightDistance(80, -20);
  driveStraightDistance(160, 40);
  turnRight(24);
  driveStraightDistance(10,30);
  turnRight(4);
  driveStraightDistance(40,50);
  driveStraightDistance(70,-50);
  turnRight(-24);
  driveStraightDistance(20,-30);
  turnRight(-4);
  driveStraightDistance(10,-30);
  turnRight(-5);
  driveStraightDistance(130,-50);
  driveStraightDistance(200,-55);
    */



}
