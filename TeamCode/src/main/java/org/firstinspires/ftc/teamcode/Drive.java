//Importing packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Drive")

public class Drive extends LinearOpMode {

    //defining motor variables
    private DcMotor lf_motor = null;
    private DcMotor rf_motor = null;
    private DcMotor lb_motor = null;
    private DcMotor rb_motor = null;
    private DcMotor arm = null;
    private Servo launcher = null;
    private Servo wrist = null;
    private CRServo claw = null;

    //defining imu variable
    IMU imu = null;

    //defining variables which are used later in the code
    double botHeading = 0;
    double cosineMove = 0;
    double sineMove = 0;
    double cosinePivot = 0;
    double sinePivot = 0;
    double x = 0;
    double y = 0;
    double rx = 0;
    double xRot = 0;
    double yRot = 0;
    double mag = 0;
    double startAngle = 0;

    @Override
    public void runOpMode() {

        //attaching motors to variables
        lf_motor = hardwareMap.get(DcMotor.class, "lf_motor");
        rf_motor = hardwareMap.get(DcMotor.class, "rf_motor");
        lb_motor = hardwareMap.get(DcMotor.class, "lb_motor");
        rb_motor = hardwareMap.get(DcMotor.class, "rb_motor");
        arm = hardwareMap.get(DcMotor.class, "arm");
        launcher = hardwareMap.get(Servo.class,"launcher");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(CRServo.class,"claw");

        //configuring motors so they move in the right direction
        lf_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rf_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        lb_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rb_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        launcher.setPosition(0);
        wrist.setPosition(0.8);
        claw.setPower(0);

        //attaching imu to variable and getting the gyroscope set up
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        //wait until the start button is clicked
        waitForStart();

        //calculating start angle (this is subtracted from the gyroscope so "forward" is always the direction the robot faces upon initialization
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //loop that runs while the program is active
        while (opModeIsActive()) {

            //resets the gyroscope when button is clicked
            if (gamepad1.back) {
                imu.resetYaw();
            }

            //calculating bot heading using the gyroscope subtracted by the initial orientation. pi/4 is added to this, as mecanum wheels drive as if they are on a pi/4 radian angle
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - startAngle;

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            mag = Math.sqrt(x*x + y*y);
            if (mag > 1) {
                x = x/mag;
                y = y/mag;
            }

            if (Math.abs(rx) > 1) {
                rx = rx/Math.abs(rx);
            }

            xRot = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            yRot = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            cosineMove = (yRot + xRot)/Math.sqrt(2);
            sineMove = (yRot - xRot)/Math.sqrt(2);

            //because the function divides by the magnitude, we have to run two functions if mag = 0 to avoid dividing by zero. Just so you know, the limit of the second function as x approaches 0 is 1, so they are essentially identical
            if (mag == 0) {

                //setting the rotation speeds to the right stick x variable
                cosinePivot = rx;
                sinePivot = rx;

            } else {

                //setting the rotation speeds to the right stick x variable multiplied by a changing function which ensures that the motor speed can't exceed 0, resulting in smoother movement
                cosinePivot = rx * ((1 - mag) + (sineMove * sineMove) / (2 * mag));
                sinePivot = rx * ((1 - mag) + (cosineMove * cosineMove) / (2 * mag));

            }

            //Setting the motor speeds to their various linear + rotational speeds.
            lf_motor.setPower(cosineMove + cosinePivot);
            rf_motor.setPower(sineMove - sinePivot);
            rb_motor.setPower(cosineMove - cosinePivot);
            lb_motor.setPower(sineMove + sinePivot);

            double lf = cosineMove + cosinePivot;
            double rf = sineMove - sinePivot;
            double rb = cosineMove - cosinePivot;
            double lb = sineMove + sinePivot;

            if (gamepad1.left_bumper) {
                arm.setTargetPosition(-160);
            }

            if (gamepad1.right_bumper) {
                arm.setTargetPosition(0);
            }

            if (gamepad1.x) {
                wrist.setPosition(0.6);
            }

            if (gamepad1.y) {
                wrist.setPosition(0.8);
            }

            if (gamepad1.a == gamepad1.b) {
                claw.setPower(0);
            } else {
                if (gamepad1.a) {
                    claw.setPower(1);
                } else {
                    claw.setPower(-1);
                }
            }

            if (gamepad1.guide) {
                launcher.setPosition(1);
            }

            telemetry.addData("armPos",arm.getCurrentPosition());
            telemetry.addData("servoTarget",claw.getPower());
            telemetry.update();

        }
    }
}