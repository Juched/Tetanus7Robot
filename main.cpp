#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <cstdlib>

AnalogInputPin cds(FEHIO::P3_2);
FEHMotor right_motor(FEHMotor::Motor0,9);
FEHMotor left_motor(FEHMotor::Motor1,9);
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_7);

float light = 1.2;
double encInch = 40.5;
int turnSpeed = 20;

struct data{
    int leftEncoder;
    int rightEncoder;
    int leftSpeed;
    int rightSpeed;
    int totalTicks;
    int RPSx ;
    int RPSy;
    int heading ;
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
    LCD.WriteRC("Error: ",9,0 );
    LCD.WriteRC(printData.error,9,120);

}

void driveStraightDistance(double tenthsOfIn, int masterPower)
{
  int tickGoal = ( tenthsOfIn/10) * 40.5;

  //This will count up the total encoder ticks despite the fact that the encoders are constantly reset.
  int totalTicks = 0;

  //Initialise slavePower as masterPower - 5 so we don't get huge error for the first few iterations. The
  //-5 value is based off a rough guess of how much the motors are different, which prevents the robot from
  //veering off course at the start of the function.
  double slavePower = masterPower + 5;

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
    slavePower += errAdj + kdAdj;
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
    float circ = 3.14159*2*3.5;

    float distance = (angle/360)*circ;

    resetCounts();

    left_motor.SetPercent(-turnSpeed);
    while(left_encoder.Counts()<(distance*encInch)) {

    }
    left_motor.Stop();

}

void turnLeft(float angle) {
    float circ = 3.14159*2*3.5;

    float distance = (angle/360)*circ;

    resetCounts();
    left_motor.SetPercent(turnSpeed);
    while(left_encoder.Counts()<(distance*encInch)) {

    }
    left_motor.Stop();
}

int main(void)
{
  LCD.Clear();
  LCD.SetBackgroundColor(WHITE);
  LCD.SetFontColor(BLACK);
  CDS();
  //Distances specified in tenths of an inch.
  turnRight(280);
  driveStraightDistance(500,50);
}
