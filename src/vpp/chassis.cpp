#include "vpp/chassis.h"

void vpp::Chassis::moveToPoint(Pose target, float maxSpeed, float timeout) {
    PIDAlgorithm algorithm(config.drive, timeout);
    PIDAlgorithm headingAlgorithm(config.heading, timeout);

    std::cout << "Moving from: (" << odometry.pose.x << ", " << odometry.pose.y << ")" << "\n";
    std::cout << "to: (" << target.x << ", " << target.y << ")" << "\n";
    std::cout << "in: " << algorithm.timeout / 1000 << " seconds" << std::endl;

    do {
        float lateralError = odometry.pose.distance(target);
        float angularError = odometry.pose.angle(target);

        float lateralMotorPower = algorithm.update(lateralError);
        float angularMotorPower = headingAlgorithm.update(angularError);

        std::cout << "Angular Error: " << angularError << std::endl;
        std::cout << "Angular Motor Power: " << angularMotorPower << std::endl;

        arcade(lateralMotorPower, angularMotorPower, maxSpeed);

        sleep(20);
    } while (algorithm.shouldContinue());

    std::cout << "Done" << std::endl;
};

void vpp::Chassis::turnToPoint(Pose target, float maxSpeed, float timeout) {
    PIDAlgorithm algorithm(config.turn, timeout);

    std::cout << "Turning from: " << odometry.pose.theta << "\n";
    std::cout << "to: " << target.theta << "\n";
    std::cout << "in: " << algorithm.timeout / 1000 << " seconds" << std::endl;

    do {
        float error = odometry.pose.angle(target);
        float motorPower = algorithm.update(error);

        std::cout << "Error: " << error << std::endl;
        std::cout << "Motor Power: " << motorPower << std::endl;

        arcade(0, motorPower, maxSpeed);

        sleep(20);
    } while (algorithm.shouldContinue());

    std::cout << "Done" << std::endl;
};