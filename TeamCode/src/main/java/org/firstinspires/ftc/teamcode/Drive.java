//Importing packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Drive")

public class Drive extends LinearOpMode {

    //defining variables used later in the code
    double x = 0;
    double y = 0;
    double rx = 0;

    //creating a robot object, every input (ex. reading a sensor value) and output (ex. running a motor) is run through this class
    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        //wait until the start button is clicked
        waitForStart();

        //this attaches actual functions to attributes found within the robot object
        robot.initializeRobot();

        //loop that runs while the program is active
        while (opModeIsActive()) {

            //resets the gyroscope when button is clicked
            if (gamepad1.back) {
                robot.imu.resetYaw();
            }

            //setting variables to gamepad values
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            //setting the drivetrain speed as a function of input values
            robot.drivetrain.runDrivetrainFromCartesian(x,y,rx,robot.getBotHeading());

            //setting arm to go up (-80 ticks) when the left bumper is pressed
            if (gamepad1.left_bumper) {
                robot.arm.setTargetPosition(-160);
                robot.arm.setPower(0.5);
            }

            //setting arm to go down (0 ticks) when the right bumper is pressed
            if (gamepad1.right_bumper) {
                robot.arm.setTargetPosition(0);
                robot.arm.setPower(0.3);
            }

            if (gamepad1.x) {
                robot.wrist.setPosition(0.6);
            }

            //moving the wrist up when the x button is pressed
            if (gamepad1.y) {
                robot.wrist.setPosition(0.8);
            }

            //setting the claw to move in, out, or neither depending on what combination of boolean inputs a and b give
            if (gamepad1.a == gamepad1.b) {
                robot.claw.setPower(0);
            } else {
                if (gamepad1.a) {
                    robot.claw.setPower(1);
                } else {
                    robot.claw.setPower(-1);
                }
            }

            //launching the paper airplane when the guide button is clicked
            if (gamepad1.guide) {
                robot.launchDrone();
            }

        }
    }
}