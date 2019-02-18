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


void driveStraightDistance(double tenthsOfIn, int masterPower)
{
  int tickGoal = ( tenthsOfIn/10) * 40.5;

  //This will count up the total encoder ticks despite the fact that the encoders are constantly reset.
  int totalTicks = 0;

  //Initialise slavePower as masterPower - 5 so we don't get huge error for the first few iterations. The
  //-5 value is based off a rough guess of how much the motors are different, which prevents the robot from
  //veering off course at the start of the function.
  int slavePower = masterPower + 5;

  int error = 0;
  int error_prior =0;

  double kp = 0.15;
  //double ki;
  double kd = 30.0 ; //18

  left_encoder.ResetCounts();
  right_encoder.ResetCounts();

  //Monitor 'totalTicks', instead of the values of the encoders which are constantly reset.
  while(std::abs(totalTicks) < tickGoal){

    //Proportional algorithm to keep the robot going straight.
    left_motor.SetPercent(masterPower);
    right_motor.SetPercent(slavePower);

    error = left_encoder.Counts() - right_encoder.Counts();

    slavePower -= (error / kp) + (kd * (error - error_prior));

    totalTicks+= left_encoder.Counts();

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    Sleep(5);

    error_prior= error;

    //Add this iteration's encoder values to totalTicks.

  }
  left_motor.Stop(); // Stop the loop once the encoders have counted up the correct number of encoder ticks.
  right_motor.Stop();
}


int main(void)
{
  //Distances specified in tenths of an inch.

  driveStraightDistance(240,30);
}
