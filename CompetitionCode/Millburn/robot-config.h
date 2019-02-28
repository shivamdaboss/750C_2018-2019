vex::brain Brain = vex::brain();

vex::controller cont = vex::controller();
//drive motors
vex::motor LeftDriveMotorOne = vex::motor(vex::PORT1, false);
vex::motor LeftDriveMotorTwo = vex::motor(vex::PORT2, false);
vex::motor RightDriveMotorOne = vex::motor(vex::PORT15, true);
vex::motor RightDriveMotorTwo = vex::motor(vex::PORT13, true);

//roller motor

vex::motor Intake = vex::motor(vex::PORT20, true);

//shooter

vex::motor Shooter = vex::motor(vex::PORT7, false);
vex::motor ShooterSecond = vex::motor(vex::PORT12, true);

//cap knocker

vex::motor Knocker = vex::motor(vex::PORT18, false);

//gyro
vex::gyro Gyro = vex::gyro(Brain.ThreeWirePort.A);
vex::pot Pot = vex::pot(Brain.ThreeWirePort.B);
