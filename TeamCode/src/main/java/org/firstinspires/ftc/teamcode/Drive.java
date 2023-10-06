//Importing packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    //defining imu variable
    IMU imu = null;

    //defining variables which are used later in the code
    double botHeading = 0;
    double x = 0;
    double y = 0;
    double rx = 0;
    double startAngle = 0;

    MotorMatrix motorMatrix;

    @Override
    public void runOpMode() {

        //attaching motors to variables
        lf_motor = hardwareMap.get(DcMotor.class, "lf_motor");
        rf_motor = hardwareMap.get(DcMotor.class, "rf_motor");
        lb_motor = hardwareMap.get(DcMotor.class, "lb_motor");
        rb_motor = hardwareMap.get(DcMotor.class, "rb_motor");

        //configuring motors so they move in the right direction
        lf_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rf_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        lb_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rb_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        //attaching imu to variable and getting the gyroscope set up
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        motorMatrix = new MotorMatrix();

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

            motorMatrix.setMotorMatrixFromCartesian(x,y,rx,botHeading);

            lf_motor.setPower(motorMatrix.lf);
            rf_motor.setPower(motorMatrix.rf);
            rb_motor.setPower(motorMatrix.rb);
            lb_motor.setPower(motorMatrix.lb);

        }
    }
}