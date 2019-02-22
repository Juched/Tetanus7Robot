#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>

AnalogInputPin cds(FEHIO::P0_0);
FEHMotor rightMotor(FEHMotor::Motor0,9);
FEHMotor leftMotor(FEHMotor::Motor1,9);
AnalogEncoder rightEnc(FEHIO::P0_1);
AnalogEncoder leftEnc(FEHIO::P0_2);

float light = 1.5;
int rightFaster = 1; //how many times faster right is than left
float encInch = 40.5; //encodes/inch
int turnSpeed = 10;

void resetCounts() {
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
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

void turnRight(int angle) {
    float circ = 3.14159*2*3.5;

    float distance = (angle/360)*circ;

    resetCounts();
    leftMotor.SetPercent(turnSpeed);
    while(leftEnc.Counts()<distance*encInch);
    leftMotor.Stop();

}

void turnLeft(int angle) {
    float circ = 3.14159*2*3.5;

    float distance = (angle/360)*circ;

    resetCounts();
    leftMotor.SetPercent(turnSpeed);
    while(leftEnc.Counts()<distance*encInch);
    leftMotor.Stop();
}

int main(void) {

    //wait for starting light
    CDS();


}
