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
    private DcMotor lfMotor = null;
    private DcMotor rfMotor = null;
    private DcMotor lbMotor = null;
    private DcMotor rbMotor = null;

    Drivetrain drivetrain = new Drivetrain(lfMotor,rfMotor,lbMotor,rbMotor);

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
        lfMotor = hardwareMap.get(DcMotor.class, "lf_motor");
        rfMotor = hardwareMap.get(DcMotor.class, "rf_motor");
        lbMotor = hardwareMap.get(DcMotor.class, "lb_motor");
        rbMotor = hardwareMap.get(DcMotor.class, "rb_motor");

        //configuring motors so they move in the right direction
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

            drivetrain.runDrivetrainFromCartesian(x,y,rx,botHeading);

        }
    }
}