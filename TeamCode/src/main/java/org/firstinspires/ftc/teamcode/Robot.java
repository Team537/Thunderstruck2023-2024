package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {

    //defining attributes which will be used in the code (sensors, motors, and other variables)
    Drivetrain drivetrain = new Drivetrain();
    IMU imu;
    DcMotor arm;
    Servo wrist;
    CRServo claw;
    Servo launcher;
    Servo dropper;
    ColorSensor colorSensor;
    private double startAngle;

    //returns the bot heading of the robot by subtracting the reading from the initial angle
    public double getBotHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - startAngle;
    }

    //launches the paper drone
    public void launchDrone() {
        launcher.setPosition(1);
    }

    //drops the purple pixel on the spike mark during autonomous
    public void dropPixel() {
        dropper.setPosition(1);
    }

    //attaches attributes to physical inputs and outputs, and makes sure all the motors/servos are in the correct position
    public void initializeRobot() {

        //attaching the individual motors of drivetrain
        drivetrain.lfMotor = hardwareMap.get(DcMotor.class, "lf_motor");
        drivetrain.rfMotor = hardwareMap.get(DcMotor.class, "rf_motor");
        drivetrain.lbMotor = hardwareMap.get(DcMotor.class, "lb_motor");
        drivetrain.rbMotor = hardwareMap.get(DcMotor.class, "rb_motor");

        //configuring motors so they move in the right direction
        drivetrain.lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.rfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrain.lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //attaching imu to variable and getting the gyroscope set up
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        //getting the initial angle on the robot
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //attaching arm/launcher motors and servos
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(CRServo.class,"claw");
        launcher = hardwareMap.get(Servo.class,"launcher");
        dropper = hardwareMap.get(Servo.class,"dropper");

        //configuring the arm so it is in the right mode at the right power
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        //setting up the servos to be in the right position
        wrist.setPosition(0.8);
        claw.setPower(0);
        launcher.setPosition(0);
        dropper.setPosition(0);

        //attaching color sensor
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

    }


}
