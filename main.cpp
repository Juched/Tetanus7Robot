#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <cstdlib>


AnalogInputPin cds(FEHIO::P0_3);
FEHMotor rightMotor(FEHMotor::Motor0,9);
FEHMotor leftMotor(FEHMotor::Motor1,9);
DigitalEncoder rightEnc(FEHIO::P0_7);
DigitalEncoder leftEnc(FEHIO::P0_0);

float light = 2;
int rightFaster = 1; //how many times faster right is than left
float encInch = 40.5; //encodes/inch
int turnSpeed = 30;

void resetCounts() {
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
}

float abs(float a) {
    if(a<0) {
        a *= -1;
    }

    return a;
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

  leftEnc.ResetCounts();
  rightEnc.ResetCounts();
  //Monitor 'totalTicks', instead of the values of the encoders which are constantly reset.
  while(std::abs(totalTicks) < tickGoal){


    //Proportional algorithm to keep the robot going straight.

    leftMotor.SetPercent(masterPower);
    rightMotor.SetPercent(slavePower);

    error = (leftEnc.Counts()) - (rightEnc.Counts());
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

    totalTicks = leftEnc.Counts();


    Sleep(50);


    error_prior= error;
    leftPrior = leftEnc.Counts();
    rightPrior= rightEnc.Counts();

    //Add this iteration's encoder values to totalTicks.

  }
  leftMotor.Stop(); // Stop the loop once the encoders have counted up the correct number of encoder ticks.
  rightMotor.Stop();
}

float CDS() {
    float value = cds.Value();

    while(value > light) {
        value = cds.Value();
    }

    return value;
}

void driveStraightCounts(int counts, int power) {
    resetCounts();
    int stop = counts;

    leftMotor.SetPercent(power);
    rightMotor.SetPercent(power*rightFaster);

    while((leftEnc.Counts() < stop) || (rightEnc.Counts() < stop)) {
        if(leftEnc.Counts() >= stop) {
            leftMotor.Stop();
        }
        if(rightEnc.Counts() >= stop) {
            rightMotor.Stop();
        }
    }

    rightMotor.Stop();
    leftMotor.Stop();
}

void turnRight(float angle) {
    float circ = 3.14159*2*6.75;

    float distance = (angle/360)*circ;

    resetCounts();
    leftMotor.SetPercent(turnSpeed);
    while(leftEnc.Counts()<distance*encInch) {
        //LCD.WriteLine(leftEnc.Counts());
    }
    leftMotor.Stop();

}

void turnLeft(float angle) {
    float circ = 3.14159*2*6.75;

    float distance = (angle/360.)*circ;

    resetCounts();
    rightMotor.SetPercent(turnSpeed);
    while(rightEnc.Counts()<distance*encInch) {
        //LCD.WriteLine(rightEnc.Counts());
    }
    rightMotor.Stop();
}

void headingAlign(float heading) {
    float difference = abs(RPS.Heading() - heading);
    while(difference > 2) {
        difference = abs(RPS.Heading() - heading);
        if(RPS.Heading() > heading) {
            turnRight(1);
        } else {
            turnLeft(1);
        }
        Sleep(.5);
    }
}

void turnBoth(float angle) {
    float circ = 3.1415*6.75;
    float counts = 214.67;

    rightMotor.SetPercent(-20);
    leftMotor.SetPercent(20);

    resetCounts();

    while(rightEnc.Counts() < counts || leftEnc.Counts() < counts) {
        if(rightEnc.Counts() >= counts) {
            rightMotor.Stop();
        }
        if(leftEnc.Counts() >= counts) {
            leftMotor.Stop();
        }
    }

    rightMotor.Stop();
    leftMotor.Stop();
}

void red() {
    driveStraightDistance(105,30);
    turnBoth(0);
    driveStraightDistance(500,-30);

    LCD.WriteLine("REDREDREDREDREDRED");

    turnRight(45);

    driveStraightDistance(70,50);

    turnLeft(40);

    driveStraightDistance(700,50);
}

void blue() {
    driveStraightDistance(43,30);
    turnBoth(0);
    driveStraightDistance(500,-30);

    LCD.WriteLine("BLUEBLUEBLUEBLUE");

    driveStraightDistance(500,50);

    turnLeft(30);

    driveStraightDistance(200,50);
}

int main(void) {
    //wait for starting light
    RPS.InitializeTouchMenu();

    CDS();

    turnLeft(135);

    driveStraightDistance(40,25);

    float squaredHeading = RPS.Heading();
    int inches = 0;
    float cdsValue = 3.3;
    int count = 0;

    while(inches < 35) {
        driveStraightDistance(10, -30);
        if(cds.Value() < light && inches > 15 && cds.Value() < cdsValue) {
            cdsValue = cds.Value();
            LCD.WriteLine(cdsValue);
            count++;
        }
        headingAlign(squaredHeading);
        inches++;
    }

    //driveStraightDistance(5,30);
    //cdsValue = cds.Value();

    if(cdsValue < 1.5) {
        red();
    } else {
        blue();
    }

    LCD.Write("CDS: ");
    LCD.WriteLine(cdsValue);




}
