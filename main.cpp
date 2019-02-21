#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <cstdlib>

DigitalEncoder right_encoder(FEHIO::P0_2);
DigitalEncoder left_encoder(FEHIO::P0_1);
FEHMotor right_motor(FEHMotor::Motor0,9);
FEHMotor left_motor(FEHMotor::Motor1,9);

void writeScreen(int leftEncoder, int rightEncoder, int leftSpeed, int rightSpeed, int totalTicks){
    LCD.Clear();
    LCD.WriteRC(leftEncoder,4,0);
    LCD.WriteRC(rightEncoder,5,0);
    LCD.WriteRC(leftSpeed,6,0);
    LCD.WriteRC(rightSpeed,7,0);
    LCD.WriteRC(totalTicks,8,0);
}

void driveStraightDistance(double tenthsOfIn, int masterPower)
{
  int tickGoal = ( tenthsOfIn/10) * 40.5;
  int leftPrior = 0;
  int rightPrior = 0;


  //This will count up the total encoder ticks despite the fact that the encoders are constantly reset.
  int totalTicks = 0;

  //Initialise slavePower as masterPower - 5 so we don't get huge error for the first few iterations. The
  //-5 value is based off a rough guess of how much the motors are different, which prevents the robot from
  //veering off course at the start of the function.
  int slavePower = masterPower - 3;

  int error = 0;
  int error_prior =0;

  double kp = 1.91325 ;
  double ki = .014;
  double kd = .5 ; //18

  left_encoder.ResetCounts();
  right_encoder.ResetCounts();

  //Monitor 'totalTicks', instead of the values of the encoders which are constantly reset.
  while(std::abs(totalTicks) < tickGoal){


    //Proportional algorithm to keep the robot going straight.
    left_motor.SetPercent(masterPower);
    right_motor.SetPercent(slavePower);

    error = (left_encoder.Counts()-leftPrior) - (right_encoder.Counts()-rightPrior);
    leftPrior = left_encoder.Counts();
    rightPrior = right_encoder.Counts();
    slavePower += (error * kp) + (kd *(error - error_prior)) +(ki*error) ;

   totalTicks = left_encoder.Counts();
   writeScreen(left_encoder.Counts(), right_encoder.Counts(), masterPower, slavePower, totalTicks);
   //left_encoder.ResetCounts();
   //right_encoder.ResetCounts();

   Sleep(2);


    error_prior= error;

    //Add this iteration's encoder values to totalTicks.

  }
  left_motor.Stop(); // Stop the loop once the encoders have counted up the correct number of encoder ticks.
  right_motor.Stop();

}



int main(void)
{
  LCD.Clear();  
  RPS.InitializeTouchMenu();
  int heading = RPS.Heading();
  int x = RPS.X();
  int y = RPS.Y();

  LCD.SetBackgroundColor(WHITE);
  LCD.SetFontColor(BLACK);

  //Distances specified in tenths of an inch.
  driveStraightDistance(180.80,30);
  driveStraightDistance(97.77,60);
  left_motor.Stop();
  right_motor.Stop();
  driveStraightDistance(12,20);
  driveStraightDistance(178.51,30);
  left_encoder.ResetCounts();
  right_encoder.ResetCounts();
  left_motor.SetPercent(40);
  right_motor.SetPercent(17);
  while(left_encoder.Counts()-right_encoder.Counts() < 415){}
  left_motor.Stop();
  right_motor.Stop();

  LCD.WriteRC(heading, 1,0);//273
  LCD.WriteRC(x,2,0);//5
  LCD.WriteRC(y,3,0);//18



}
