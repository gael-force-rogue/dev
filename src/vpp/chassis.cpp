#include "vpp/chassis.h"
#include "pid.h"

#include <iostream>

// void vpp::Chassis::moveToPoint(Pose target, float maxSpeed, float timeout) {
//     PIDAlgorithm algorithm(config.drive, timeout);
//     PIDAlgorithm headingAlgorithm(config.heading, timeout);

//     std::cout << "Moving from: (" << odometry.pose.x << ", " << odometry.pose.y << ")" << "\n";
//     std::cout << "to: (" << target.x << ", " << target.y << ")" << "\n";
//     std::cout << "in: " << algorithm.timeout / 1000 << " seconds" << std::endl;

//     do {
//         float lateralError = odometry.pose.distance(target);
//         float angularError = odometry.pose.angle(target);

//         std::cout << "Angular Error: " << angularError << std::endl;

//         float lateralMotorPower = algorithm.update(lateralError);
//         float angularMotorPower = headingAlgorithm.update(angularError);

//         arcade(lateralMotorPower, angularMotorPower, maxSpeed);

//         sleep(20);
//     } while (algorithm.shouldContinue());

//     arcade(0, 0);

//     std::cout << "Done" << std::endl;
// };

// void vpp::Chassis::turnToPoint(Pose target, float maxSpeed, float timeout) {
//     PIDAlgorithm algorithm(config.turn, timeout);

//     std::cout << "Turning from: " << odometry.pose.theta << "\n";
//     std::cout << "to: " << target.theta << "\n";
//     std::cout << "in: " << algorithm.timeout / 1000 << " seconds" << std::endl;

//     do {
//         // float error = odometry.pose.angle(target);
//         float error = target.theta - odometry.pose.theta;
//         float motorPower = algorithm.update(error);

//         std::cout << "Error: " << error << std::endl;

//         arcade(0, motorPower, maxSpeed);

//         sleep(20);
//     } while (algorithm.shouldContinue());

//     arcade(0, 0);

//     std::cout << "Done" << std::endl;
// };