vex::brain Brain;

vex::controller Controller; 
 

//drive motors 
vex::motor LD1(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor LD2(vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::motor RD1(vex::PORT18, vex::gearSetting::ratio18_1, false);
vex::motor RD2(vex::PORT17, vex::gearSetting::ratio18_1, false);

//shooter motors
vex::motor S1(vex::PORT16, vex::gearSetting::ratio18_1, false);
vex::motor S2(vex::PORT15, vex::gearSetting::ratio18_1, true);


//intake motors
vex::motor I1(vex::PORT14, vex::gearSetting::ratio18_1, false);
vex::motor I2(vex::PORT13, vex::gearSetting::ratio18_1, true);

//vision sensor
vex::vision::signature RED_FLAG (1, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature BLUE_FLAG (2, -4059, -2221, -3140, 4577, 13997, 9288, 1.2999999523162842, 0);
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision (vex::PORT12, 50, RED_FLAG, BLUE_FLAG, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);



//leds

vex::digital_out RED = vex::digital_out(Brain.ThreeWirePort.A);
vex::digital_out YELLOW = vex::digital_out(Brain.ThreeWirePort.B);
vex::digital_out GREEN = vex::digital_out(Brain.ThreeWirePort.C);
