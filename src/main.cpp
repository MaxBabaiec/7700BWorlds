/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       babiecfamily                                              */
/*    Created:      4/8/2024, 9:23:35 AM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain  Brain;
controller Controller1 = controller(primary);
motor LF = motor(PORT11, ratio6_1, true);
motor RF = motor(PORT1, ratio6_1, false);
motor LB = motor(PORT21, ratio6_1, true);
motor RB = motor(PORT20, ratio6_1, false);
motor RM = motor(PORT2, ratio6_1, false);
motor LM = motor(PORT12, ratio6_1, true);
motor R4 = motor(PORT10, ratio6_1, false);
motor L4 = motor(PORT18, ratio6_1, true);
motor slapper = motor(PORT9, ratio36_1, false);
motor Intake = motor(PORT13, ratio6_1, true);
inertial gt = inertial(PORT10);
inertial gt2 = inertial(PORT11);
digital_out wings = digital_out(Brain.ThreeWirePort.A);
digital_out wingsBack = digital_out(Brain.ThreeWirePort.E);
digital_out Hang = digital_out(Brain.ThreeWirePort.C);
digital_out Blocker = digital_out(Brain.ThreeWirePort.D);
gps GPS = gps(PORT14, 0.00, 0.00, inches, 0);


int W = 12;
double pi = 3.14;
int D = 4;
int autonSelect = 0;
int autonMin = 0;
int autonMax = 4;
bool DriveReverse = false;
bool Stop = false;
double YOFFSET = 20; // offset for the display

// Writes a line for the diagnostics of a motor on the Brain
void MotorDisplay(double y, double curr, double temp)
{
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
  if (curr < 1)
    Brain.Screen.setFillColor(green);
  else if (curr >= 1 && curr <= 2.5)
    Brain.Screen.setFillColor(yellow);
  else
    Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);

  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
  if (temp < 45)
    Brain.Screen.setFillColor(green);
  else if (temp <= 50 && temp >= 45)
    // TRUE and TRUE --> True
    // TRUE and FALSE --> False
    // FALSE and FALSE --> False
    Brain.Screen.setFillColor(yellow);
  else
    Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
  Brain.Screen.setFillColor(transparent);
}

// Displays information on the brain
void Display()
{
  double leftFrontCurr = LF.current(amp);
  double leftFrontTemp = LF.temperature(celsius);
  double leftBackCurr = LB.current(amp);
  double leftBackTemp = LB.temperature(celsius);
  double rightFrontCurr = RF.current(amp);
  double rightFrontTemp = RF.temperature(celsius);
  double rightBackCurr = RB.current(amp);
  ;
  double rightBackTemp = RB.temperature(celsius);
  double rightMiddleCurr = RM.velocity(rpm);

  Brain.Screen.printAt(1, 200, "right middle amps %.2f  ", rightMiddleCurr);
  double rightMiddleTemp = RM.temperature(celsius);
  double leftMiddleCurr = LM.velocity(rpm);
  ;
  double leftMiddleTemp = LM.temperature(celsius);

  if (LF.installed())
  {
    MotorDisplay(1, leftFrontCurr, leftFrontTemp);
    Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
  }
  else
    Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront Problem");

  if (LB.installed())
  {
    MotorDisplay(31, leftBackCurr, leftBackTemp);
    Brain.Screen.printAt(300, YOFFSET + 31, "LeftBack");
  }
  else
    Brain.Screen.printAt(5, YOFFSET + 31, "LeftBack Problem");

  if (RF.installed())
  {
    MotorDisplay(61, rightFrontCurr, rightFrontTemp);
    Brain.Screen.printAt(300, YOFFSET + 61, "RightFront");
  }
  else
    Brain.Screen.printAt(5, YOFFSET + 61, "RightFront Problem");

  if (RB.installed())
  {
    MotorDisplay(91, rightBackCurr, rightBackTemp);
    Brain.Screen.printAt(300, YOFFSET + 91, "RightBack");
  }
  else
    Brain.Screen.printAt(5, YOFFSET + 91, "RightBack Problem");

  if (RM.installed())
  {
    MotorDisplay(121, rightMiddleCurr, rightMiddleTemp);
    Brain.Screen.printAt(300, YOFFSET + 121, "RightMiddle");
  }
  else
    Brain.Screen.printAt(5, YOFFSET + 121, "RightMiddle Problem");

  if (LM.installed())
  {
    MotorDisplay(151, leftMiddleCurr, leftMiddleTemp);
    Brain.Screen.printAt(300, YOFFSET + 151, "LeftMiddle");
  }
  else
    Brain.Screen.printAt(5, YOFFSET + 151, "LeftMiddle Problem");
}

void reverseDrive() { DriveReverse = !DriveReverse; }

// Motor stop
void Motorbrake()
{
  if (Stop == true)
  {
    RF.setStopping(brake);
    LF.setStopping(brake);
    RB.setStopping(brake);
    LB.setStopping(brake);
    RM.setStopping(brake);
    LM.setStopping(brake);
    R4.setStopping(brake);
    L4.setStopping(brake);
  }
  if (Stop == false)
  {
    RF.setStopping(coast);
    LF.setStopping(coast);
    RB.setStopping(coast);
    LB.setStopping(coast);
    RM.setStopping(coast);
    LM.setStopping(coast);
    R4.setStopping(coast);
    L4.setStopping(coast);
  }
}

// Drivebase code
void driveVolts(int rspeed, int lspeed, int wt, bool isVolts = true)
{
  /* if (DriveReverse) {
     rspeed = -rspeed;
     lspeed = -lspeed;
     int tempr = rspeed;
     rspeed = lspeed;
     lspeed = tempr;
   }*/
  if (isVolts)
  {
    RF.spin(forward, rspeed * 120, voltageUnits::mV);
    LF.spin(forward, lspeed * 120, voltageUnits::mV);
    RB.spin(forward, rspeed * 120, voltageUnits::mV);
    LB.spin(forward, lspeed * 120, voltageUnits::mV);
    RM.spin(forward, rspeed * 120, voltageUnits::mV);
    LM.spin(forward, lspeed * 120, voltageUnits::mV);
    R4.spin(forward, rspeed * 120, voltageUnits::mV);
    L4.spin(forward, lspeed * 120, voltageUnits::mV);

    wait(wt, msec);
  }
  else
  {
    RF.spin(forward, rspeed, percent);
    LF.spin(forward, lspeed, percent);
    RB.spin(forward, rspeed, percent);
    LB.spin(forward, lspeed, percent);
    RM.spin(forward, rspeed, percent);
    LM.spin(forward, lspeed, percent);
    R4.spin(forward, rspeed, percent);
    L4.spin(forward, lspeed, percent);
    wait(wt, msec);
  }
}

// Time and spped code for auton
void drive(int rs, int ls, int wt)
{
  RF.spin(forward, rs, percent);
  LF.spin(forward, ls, percent);
  RB.spin(forward, rs, percent);
  LB.spin(forward, ls, percent);
  RM.spin(forward, rs, percent);
  LM.spin(forward, ls, percent);
  R4.spin(forward, rs, percent);
  L4.spin(forward, ls, percent);
  wait(wt, msec);
}

// Custom inch drive
void driveDeg(float deg)
{
  Motorbrake();
  Stop = true;
  deg = deg * (360 / 8.635); // formula for larger number: 360(wheel gear/motor gear)
  LF.setVelocity(80, percent);
  RF.setVelocity(80, percent);
  LB.setVelocity(80, percent);
  RB.setVelocity(80, percent);
  RM.setVelocity(80, percent);
  LM.setVelocity(80, percent);
  RF.spinFor(forward, deg, degrees, false);
  LF.spinFor(forward, deg, degrees, false);
  RB.spinFor(forward, deg, degrees, false);
  LB.spinFor(forward, deg, degrees, false);
  RM.spinFor(forward, deg, degrees, false);
  LM.spinFor(forward, deg, degrees);
  wait(50, msec);
}

void driveDegF(float deg)
{
  Motorbrake();
  Stop = true;
  deg = deg * (600 / 10.205); // formula for larger number: 360(wheel gear/motor gear)
  LF.setVelocity(100, percent);
  RF.setVelocity(100, percent);
  LB.setVelocity(100, percent);
  RB.setVelocity(100, percent);
  RM.setVelocity(100, percent);
  LM.setVelocity(100, percent);
  RF.spinFor(forward, deg, degrees, false);
  LF.spinFor(forward, deg, degrees, false);
  RB.spinFor(forward, deg, degrees, false);
  LB.spinFor(forward, deg, degrees, false);
  RM.spinFor(forward, deg, degrees, false);
  LM.spinFor(forward, deg, degrees);
  wait(50, msec);
}

void inchTurn(float target)
{
  float x = 0.0;
  target = W * pi * target / 360.0;
  LF.setPosition(0.0, rev);
  while (x < target)
  {
    drive(50, -50, 10);
    x = pi * D * LF.position(rev);
  }
}

// PID inch drive
void inchDrive(float speed, float target)
{
  Motorbrake();
  LF.setPosition(0, rev);
  float inches = 0;
  float d = 0.5;
  Stop = true;
  while (fabs(target - inches) > d)
  {
    Brain.Screen.printAt(1, 120, "%f.2 inches  %f.2 target", inches, target);
    driveVolts(speed, speed, 10);
    inches = pi * D * (LF.position(rev) * 1.4);
  }
  driveVolts(0, 0, 0);
}

void claw()
{
  if (Controller1.ButtonL2.pressing())
  {
    wings.set(true);
  }
  if (Controller1.ButtonL1.pressing())
  {
    wings.set(false);
  }
}

// Slapper diagnostic
void slapperMonitor()
{
  double current2 = slapper.current();
  double t2 = slapper.temperature(celsius);
  Brain.Screen.printAt(1, 220, "slapper current = %.1f   Temp = %.1f   ", current2, t2);
}

// Auton select
void drawGUI()
{
  // Draws 2 buttons to be used for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 100, "Auton Selected =  %d   ", autonSelect);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(20, 110, 50, 50); // x,y,l,w
  Brain.Screen.drawCircle(300, 120, 25);
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(170, 110, 50, 50); // x,y,l,w
  Brain.Screen.setFillColor(black);
}

void selectAuton()
{
  bool selectingAuton = true;

  int x = Brain.Screen
              .xPosition(); // get the x position of last touch of the screen
  int y = Brain.Screen
              .yPosition(); // get the y position of last touch of the screen
  // check to see if buttons were pressed
  if (x >= 20 && x <= 150 && y >= 50 && y <= 150) // select button pressed
  {
    autonSelect++;
    if (autonSelect > autonMax)
      autonSelect = autonMin; // rollover
    Brain.Screen.printAt(1, 100, "Auton Selected =  %d   ", autonSelect);
  }
  if (x >= 170 && x <= 270 && y >= 50 && y <= 150)
  {
    selectingAuton = false; // GO button pressed
    Brain.Screen.printAt(1, 150, "Auton  =  %d   GO           ", autonSelect);
  }
  else
  {
    selectingAuton = true;
    Brain.Screen.clearScreen();
    wait(50, msec);
    drawGUI();
  }
  /*if(x < 170 && x >= 0 && y < 50 && y >= 0){
    selectingAuton = true;
  }*/
  if (!selectingAuton)
  {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(300, 120, 25);
  }
  else
  {
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawCircle(300, 120, 25);
  }
  wait(10, msec); // slow it down
  Brain.Screen.setFillColor(black);
}

// hang
void hang()
{
  if (Controller1.ButtonA.pressing())
  {
    Blocker.set(false);
    wait(100, msec);
    Hang.set(false);
  }
  else
  {
  }
}

// Wings
void Expan()
{
  if (Controller1.ButtonA.pressing())
  {
    wings.set(true);
    wingsBack.set(true);
  }
  if (Controller1.ButtonY.pressing())
  {
    wings.set(false);
    wingsBack.set(false);
  }
  if (Controller1.ButtonUp.pressing())
  {
    wings.set(true);
  }
  if (Controller1.ButtonDown.pressing())
  {
    wingsBack.set(true);
  }
}

// Auton wings
void AutoExpan()
{
  wait(105, msec);
  wings.set(true);
  wingsBack.set(true);
}

// Slapper control
void VCS(int SS)
{
  // F1.spin(forward, FWS * 120, voltageUnits::mV);

  slapper.spin(reverse, SS * 120, voltageUnits::mV);
}

// PID gyro turn
void gyroTurn(int deg)
{
  Motorbrake();
  float heading = 0.0;  // initialize a variable for heading
  float accuracy = 3.0; // how accurate to make the turn in degrees
  float error = deg - heading;
  float kp = 0.58;
  float speed = kp * error;
  Stop = true;
  gt.setRotation(0.0, degrees); // reset Gyro to zero degrees
  while (fabs(error) >= accuracy)
  {

    speed = kp * error;
    drive(-speed, speed, 10); // turn right at half speed
    heading = gt.rotation();  // measure the heading of the robot
    error = deg - heading;    // calculate error
  }
  drive(0, 0, 0);
}

// Absolute gyro turn
void AGT(int deg)
{
  Motorbrake();
  float heading = 0.0;  // initialize a variable for heading
  float accuracy = 3.0; // how accurate to make the turn in degrees
  float error = deg - heading;
  float kp = 0.58;
  float speed = kp * error;
  Stop = true;
  gt.setRotation(0.0, degrees);
  while (fabs(error) >= accuracy)
  {

    speed = kp * error;
    drive(-speed, speed, 10); // turn right at half speed
    heading = gt2.rotation();  // measure the heading of the robot
    error = deg - heading;    // calculate error
  }
  drive(0, 0, 0);
  wait(500, msec);
  // stop the drive'
}

void LAGT(int deg)
{
  Motorbrake();
  float heading = 0.0;  // initialize a variable for heading
  float accuracy = 3.0; // how accurate to make the turn in degrees
  float error = deg - heading;
  float kp = -0.58;
  float speed = kp * error;
  Stop = true;
  // gyroturn.setRotation(0.0, degrees);  //reset Gyro to zero degrees
  while (fabs(error) >= accuracy)
  {

    speed = kp * error;
    drive(-speed, speed, 10); // turn right at half speed
    heading = gt2.rotation(); // measure the heading of the robot
    error = deg - heading;    // calculate error
  }
  drive(0, 0, 0);
  wait(500, msec);
  // stop the drive'
}

void OPturnR(double deg){
  Motorbrake();
  float heading = 0.0;  // initialize a variable for heading
  float accuracy = 3.0; // how accurate to make the turn in degrees
  float error = deg - gt2.rotation();
  float kp = 0.58;
  float speed = kp * error;
  Stop = true;
  deg = error;
  gt.setRotation(0.0, degrees); // reset Gyro to zero degrees
 
  while (fabs(error) >= accuracy)
  {
    speed = kp * error;
    drive(-speed, speed, 10); // turn right at half speed
    heading = gt.rotation();  // measure the heading of the robot
    error = deg - heading;    // calculate error
  }
  
  drive(0, 0, 0);
}

void turn(float deg){
  Motorbrake();
  Stop = true;
  float heading = gt2.rotation();  // initialize a variable for heading
  float accuracy = 0.0; // how accurate to make the turn in degrees
  float error = heading - deg;
  float kp = 0.58;
  float speed = kp * error;
  gt.setRotation(0.0, degrees); // reset Gyro to zero degrees
  deg = error;
  while (fabs(error) >= accuracy)
  {
    if (speed > 10){
    speed = kp * error;
    driveVolts(speed, -speed, 10); // turn right at half speed
    heading = gt.rotation();  // measure the heading of the robot
    error = deg - heading;    // calculate error
    }
    else if (speed <= 10 and error > 0){
    speed = 10;
    driveVolts(speed, -speed, 10); // turn right at half speed
    heading = gt.rotation();  // measure the heading of the robot
    error = deg - heading;    // calculate error
    }
    else if (speed <= 10 and error < 0){
    speed = -10;
    driveVolts(speed, -speed, 10); // turn right at half speed
    heading = gt.rotation();  // measure the heading of the robot
    error = deg - heading;    // calculate error
    }
  }
  
  driveVolts(0, 0, 0);
  Stop = false;
}


void OPturnL(double deg){
  Motorbrake();
  float heading = gt2.rotation();  // initialize a variable for heading
  float accuracy = 3.0; // how accurate to make the turn in degrees
  float error = deg - gt2.rotation();
  float kp = 0.58;
  float speed = kp * error;
  Stop = true;
  deg = error - 360;
  gt.setRotation(0.0, degrees); // reset Gyro to zero degrees
 
  while (fabs(error) >= accuracy)
  {
    speed = kp * error;
    drive(-speed, speed, 10); // turn right at half speed
    heading = gt.rotation();  // measure the heading of the robot
    error = deg - heading;    // calculate error
  }
  
  drive(0, 0, 0);
}

void backWings()
{
  wings.set(!wings.value());
  wait(200, msec);
}

void frontWings()
{
  wingsBack.set(!wingsBack.value());
  wait(200, msec);
}

void Block()
{
  Blocker.set(!Blocker.value());
  wait(150, msec);
}

void GPStracking(float x, float y)
{
  int cx = GPS.xPosition(inches);
  int cy = GPS.yPosition(inches);
  if ((cx - x) == 0 and (cy - y) < 0){
  OPturnR(90);
  driveDeg(fabs(cy - y));
  }
  else if ((cx - x) == 0 and (cy - y) > 0){
  OPturnR(270);
  driveDeg(fabs(cy - y));
  }
  else if ((cy - y) == 0 and (cx - x) < 0){
  OPturnL(180);
  driveDeg(fabs(cx - x));
  }
  else if ((cy - y) == 0 and (cx - x) > 0){
  OPturnR(0);
  driveDeg(fabs(cx - x));
  }
  else if(-(cy - y) > 0 and (cx - x) < 0){
  OPturnR((atan2(fabs(cx - x),fabs(cy - y) + 10) * 57.3) + 90);
  driveDeg(hypot((cx - x), (cy - y)));
  }
  else if (-(cy - y) < 0 and (cx - x) < 0){
  OPturnL(270 - atan2(fabs(cx - x),fabs(cy - y) + 10) * 57.3);
  driveDeg(hypot((cx - x), (cy - y)));
  }
  else if(-(cy - y) > 0){
  OPturnR(90 - atan2(fabs(cx - x),fabs(cy - y) + 10) * 57.3);
  driveDeg(hypot((cx - x), (cy - y)));
  }
  else if (-(cy - y) < 0){
  OPturnL((atan2(fabs((cx + 4) - x),fabs(cy - y) + 10) * 57.3) + 270);
  driveDeg(hypot((cx - x), (cy - y)+1));
  }
  
  else{}
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{
  drawGUI();
  Brain.Screen.pressed(selectAuton);
  gt.calibrate();
  gt2.calibrate();
  GPS.calibrate();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auton(void)
{

  switch (autonSelect)
  {

  case 0:
    // near side auton
    
    break;
  case 1:
    // skills auton
    
    break;
  case 2:
    // far side auton
    
    break;
  case 3:
    // elims far side auton
   
    break;
  case 4:
    // elims near side
   
    break;
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void)
{
  // Function callbacks and local variables
  int dir = 1;
  Controller1.ButtonLeft.pressed(reverseDrive);
  // Controller1.ButtonR1.pressed(rightWing);
  // Controller1.ButtonR2.pressed(leftWing);
  // Brain.Screen.clearScreen();
  Hang.set(true);
  while (true)
  {
    // Functions
    Motorbrake();
    Display();
    hang();
    claw();

    Stop = false;

    // Drivebase
    int LeftStick = dir * Controller1.Axis3.position();
    int RightStick = dir * Controller1.Axis2.position();
    if (dir == 1)
    {
      driveVolts(RightStick * 1.2, LeftStick * 1.2, 10);
    }
    else
    {
      driveVolts(LeftStick, RightStick, 10);
    }

    // Shooter
    if (Controller1.ButtonX.pressing())
    {

      VCS(0);
    }
    if (Controller1.ButtonB.pressing())
    {

      VCS(90);
    }

    // Intake
    if (Controller1.ButtonR1.pressing())
    {
      Intake.spin(forward, 100, percent);
    }
    else if (Controller1.ButtonR2.pressing())
    {
      Intake.spin(forward, -100, percent);
    }
    else
    {
      Intake.spin(forward, 0, percent);
    }

    // Wings
    if (Controller1.ButtonL1.pressing())
    {
      frontWings();
    }
    if (Controller1.ButtonL2.pressing())
    {
      backWings();
    }

    // Hang
    if (Controller1.ButtonUp.pressing())
    {
      Block();
    }
    else
    {}
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
