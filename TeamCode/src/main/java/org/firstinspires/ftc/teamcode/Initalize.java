package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Initalize {

    public void initializeRobot(Drivetrain drivetrain, IMU imu, DcMotor arm, Servo wrist, CRServo claw, Servo launcher, ColorSensor colorSensor, SettingSwitches settingSwitches) {
        //attaching motors to variables

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

        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

        settingSwitches.alliance = hardwareMap.get(TouchSensor.class,"alliance");
        settingSwitches.startPosition = hardwareMap.get(TouchSensor.class,"startPosition");
        settingSwitches.endPosition = hardwareMap.get(TouchSensor.class,"endPosition");
    }


}
