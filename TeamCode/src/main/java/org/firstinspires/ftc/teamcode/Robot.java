package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {

    //defining attributes which will be used in the code (sensors, motors, and other variables)
    Drivetrain drivetrain = new Drivetrain();
    IMU imu;
    DcMotor arm;
    Servo wrist;
    CRServo claw;
    Servo launcher;
    CRServo dropper;
    ColorSensor colorSensor;
    private double startAngle;
    private LinearOpMode opMode;

    public Robot(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;
    }

    //returns the bot heading of the robot by subtracting the reading from the initial angle
    public double getBotHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - startAngle;
    }

    //launches the paper drone
    public void launchDrone() {
        launcher.setPosition(0);
    }

    //drops the purple pixel on the spike mark during autonomous
    public void dropPixel() {
        dropper.setPower(1);
        opMode.sleep(1000);
        dropper.setPower(0);
    }

    //attaches attributes to physical inputs and outputs, and makes sure all the motors/servos are in the correct position
    public void initializeRobot() {

        //attaching the individual motors of drivetrain
        drivetrain.lfMotor = opMode.hardwareMap.get(DcMotor.class, "lf_motor");
        drivetrain.rfMotor = opMode.hardwareMap.get(DcMotor.class, "rf_motor");
        drivetrain.lbMotor = opMode.hardwareMap.get(DcMotor.class, "lb_motor");
        drivetrain.rbMotor = opMode.hardwareMap.get(DcMotor.class, "rb_motor");

        //configuring motors so they move in the right direction
        drivetrain.lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.rfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrain.lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //attaching imu to variable and getting the gyroscope set up
        imu = opMode.hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        //getting the initial angle on the robot
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //attaching arm/launcher motors and servos
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        wrist = opMode.hardwareMap.get(Servo.class,"wrist");
        claw = opMode.hardwareMap.get(CRServo.class,"claw");
        launcher = opMode.hardwareMap.get(Servo.class,"launcher");
        dropper = opMode.hardwareMap.get(CRServo.class,"dropper");

        //configuring the arm so it is in the right mode at the right power
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        //setting up the servos to be in the right position
        wrist.setPosition(0.65);
        claw.setPower(0);
        launcher.setPosition(1);
        dropper.setPower(0);

        //attaching color sensor
        colorSensor = opMode.hardwareMap.get(ColorSensor.class,"color_sensor");

    }


}
