//Importing packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Drive")

public class Drive extends LinearOpMode {

    //defining drivetrain
    Drivetrain drivetrain = new Drivetrain();

    //defining imu variable
    IMU imu;

    //defining arm, wirst, claw, and launcher variables
    DcMotor arm;
    Servo wrist;
    CRServo claw;
    Servo launcher;

    //defining color sensor
    ColorSensor colorSensor;

    //defining setting-switchew
    SettingSwitches settingSwitches = new SettingSwitches();


    //defining variables which are used later in the code
    double botHeading = 0;
    double x = 0;
    double y = 0;
    double rx = 0;
    double startAngle = 0;

    MotorMatrix motorMatrix;

    @Override
    public void runOpMode() {

        new Initalize().initializeRobot(drivetrain, imu, arm, wrist, claw, launcher, colorSensor, settingSwitches);

        //wait until the start button is clicked
        waitForStart();

        //calculating start angle (this is subtracted from the gyroscope so "forward" is always the direction the robot faces upon initialization
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //loop that runs while the program is active
        while (opModeIsActive()) {

            //resets the gyroscope when button is clicked
            if (gamepad1.guide) {
                imu.resetYaw();
            }

            //calculating bot heading using the gyroscope subtracted by the initial orientation. pi/4 is added to this, as mecanum wheels drive as if they are on a pi/4 radian angle
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - startAngle;

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            drivetrain.runDrivetrainFromCartesian(x,y,rx,botHeading);

        }
    }
}