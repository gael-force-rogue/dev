// void odometryThreadF() {
//     std::cout << "Odometry thread: " << vex::this_thread::get_id() << std::endl;

//     auto screen = controller.getScreen();

//     inertialSensor.calibrate(3);

//     while (true) {
//         odometry.update(leftMotorGroup.averagePosition(), rightMotorGroup.averagePosition(), inertialSensor.heading());

//         sleep(10);
//     }
// };

// void odometryLogThreadF() {
//     auto screen = controller.getScreen();
//
//     while (true) {
//         screen.clearScreen();
//         screen.setCursor(1, 1);
//         screen.print("X: %f", odometry.pose.x);
//         screen.setCursor(2, 1);
//         screen.print("Y: %f", odometry.pose.y);
//         screen.setCursor(3, 1);
//         screen.print("Theta: %f", odometry.pose.theta);
//
//         sleep(50);
//     }
// }