vex::brain Brain;
vex::motor LeftDriveMotorOne = vex::motor(vex::PORT1, false);
vex::motor LeftDriveMotorTwo = vex::motor(vex::PORT2, false);
vex::motor RightDriveMotorOne = vex::motor(vex::PORT15, true);
vex::motor RightDriveMotorTwo = vex::motor(vex::PORT13, true);
vex::gyro Gyro = vex::gyro(Brain.ThreeWirePort.A);

