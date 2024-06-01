/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       humanfriend22                                             */
/*    Created:      5/3/2024, 6:28:51 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "config.h"
#include "drivetrain.h"
#include "motor.h"
#include "vex.h"

using namespace vex;

competition Competition = competition();
brain Brain = brain();
controller Controller = controller();

motor left1 = motor(MOTOR_LEFT_1);
motor left2 = motor(MOTOR_LEFT_2);
motor left3 = motor(MOTOR_LEFT_3);
motor right1 = motor(MOTOR_RIGHT_1);
motor right2 = motor(MOTOR_RIGHT_2);
motor right3 = motor(MOTOR_RIGHT_3);

MotorCluster leftMotorCluster(left1, left2, left3);
MotorCluster rightMotorCluster(right1, right2, right3);

void driverControl() {
    while (true) {
        int x = Controller.Axis1.position();
        int y = Controller.Axis3.position();
        bool activateEbrake = Controller.ButtonA.pressing();

        if (activateEbrake) {
            leftMotorCluster.stop(hold);
            rightMotorCluster.stop(hold);
        } else {
            leftMotorCluster.setDefaultBrakeMode(coast);
            rightMotorCluster.setDefaultBrakeMode(coast);

            leftMotorCluster.spin(x + y);
            rightMotorCluster.spin(x - y);
        }
    }
}

void autonomous() {
    inertial inertialSensor = inertial(INERTIAL_SENSOR);
    SmartDrivetrain smartDrivetrain(leftMotorCluster, rightMotorCluster, inertialSensor);
};

int main() {
    // Competition.drivercontrol(driverControl);
    // Competition.autonomous(autonomous);
    driverControl();
}