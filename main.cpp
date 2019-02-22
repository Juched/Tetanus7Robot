#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>

AnalogInputPin cds(FEHIO::P3_2);
FEHMotor rightMotor(FEHMotor::Motor0,9);
FEHMotor leftMotor(FEHMotor::Motor1,9);
DigitalEncoder rightEnc(FEHIO::P0_0);
DigitalEncoder leftEnc(FEHIO::P0_7);

float light = 1.5;
int rightFaster = 1; //how many times faster right is than left
float encInch = 40.5; //encodes/inch
int turnSpeed = 25;

void resetCounts() {
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
}

void stop() {
    leftMotor.Stop();
    rightMotor.Stop();
}

void printEnc() {
    LCD.Write("Right Encodes: ");
    LCD.WriteLine(rightEnc.Counts());

    LCD.Write("Left Encodes: ");
    LCD.WriteLine(leftEnc.Counts());
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

void driveStraightInches(int counts, int power) {
    resetCounts();
    int stop = counts*encInch;

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
    float circ = 3.14159*2*3.5;

    float distance = (angle/360)*circ;

    resetCounts();

    leftMotor.SetPercent(turnSpeed);
    while(leftEnc.Counts()<(distance*encInch)) {
        printEnc();
    }
    leftMotor.Stop();

}

void turnLeft(float angle) {
    float circ = 3.14159*2*3.5;

    float distance = (angle/360)*circ;

    resetCounts();
    leftMotor.SetPercent(turnSpeed);
    while(leftEnc.Counts()<(distance*encInch)) {
        printEnc();
    }
    leftMotor.Stop();
}

int main(void) {

    //wait for starting light
    CDS();

    turnRight(45);
    driveStraightCounts(800, -50);
    driveStraightCounts(800, -50);

}
