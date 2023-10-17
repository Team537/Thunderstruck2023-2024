##Robot Class

The robot class is a class created to easily define and initialize motors

To make use of this class, simply create an object of the class:
    Robot robot;

Initialize the motors and set them to their default positions using the object method initializeRobot:
    robot.initializeRobot();

Note that none of the robots functions will activate until the initializeRobot function is called.


##List of all attributes of the Robot object:
1. Drivetrain drivetrain - A drivetrain object containing the four drivetrain motors and several methods (see: Drivetrain Class)
2. IMU imu - An IMU reference to the Control Hub's internal imu
3. DcMotor arm - A DcMotor reference to the CoreHex motor that lifts the arm
4. Servo wrist - A Servo reference to the SmartServo that manipulates the intake angle
5. CRServo claw - A CRServo reference to the Continuous Rotation SmartServo that intakes pixels
6. Servo launcher - A Servo reference to the SmartServo that launches the drone
7. Servo dropper - A Servo reference to the SmartServo that drops the purple pixel during autonomous
8. ColorSensor colorSensor - A ColorSensor reference to the Color Sensor on the robot
9. SettingSwitches settingSwitches - A settingSwitches object containing the three setting switches (touch sensors) and several methods (See: SettingSwitches Class)
10. double startAngle - The imu reading of the robot upon the call of the method initializeRobot

##List of all methods of the Robot object:
1. public void initializeRobot() - Attaches hardware to actuator and sensor attributes, and defines the startAngle variable
2. public double getBotHeading() - Returns the current heading of the robot subtracted by its initial orientation


##Drivetrain Class

The drivetrain class is a class created to easily group the wheel motors and their functions
This class is intended to be used in the robot class, but can be created in another class if need be.

##List of all attributes of the Drivetrain object:
1. DcMotor lfMotor - A DcMotor reference to the motor driving the left front wheel
2. DcMotor rfMotor - A DcMotor reference to the motor driving the right front wheel
3. DcMotor lbMotor - A DcMotor reference to the motor driving the left back wheel
4. DcMotor rbMotor - A DcMotor reference to the motor driving the right back wheel

##List of all methods of the Drivetrain object:
1. public void runDrivetrain(MotorMatrix motorMatrix) - Drives the wheels based on the values found in the MotorMatrix parameter (see: MotorMatrix Class)
2. public void runDrivetrainFromCartesian(double x, double y, double rx, double botHeading) - Directly plugs the values into a MotorMatrix and drives the wheels based on the output (see: MotorMatrix Class)
3. public void stop() - a shorthand for stopping all the wheels
4. public void rave() - a shorthand for running all the motors in a way where they will be at full speed and not move the robot


##MotorMatrix Class

The MotorMatrix class is a class created to calculate mecanum values using the Rational Mecanum function

##List of all attributes of the MotorMatrix object:
1. double lf - A value corresponding to the motor driving the left front wheel
2. double rf - A value corresponding to the motor driving the right front wheel
3. double lb - A value corresponding to the motor driving the left back wheel
4. double rb - A value corresponding to the motor driving the right back wheel

##List of all methods of the MotorMatrix object:
1. public void setMotorMatrixFromCartesian(double x, double y, double rx, double botHeading) - Uses the parameters to generate set the attributes based on the Rational Mecanum function