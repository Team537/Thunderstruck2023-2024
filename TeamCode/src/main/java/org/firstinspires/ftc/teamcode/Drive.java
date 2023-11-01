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

    DriveMode driveMode = DriveMode.MANUALDRIVE;

    boolean driveModeToggle = false;

    Alliance alliance = Alliance.RED;

    boolean allianceToggle = false;

    ScoringPosition scoringPosition = ScoringPosition.CENTER;

    boolean scoringPositionToggle = false;

    //creating a robot object, every input (ex. reading a sensor value) and output (ex. running a motor) is run through this class
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        //Init loop to swap between auto settings
        while ( !isStarted() ) {

            //Toggle alliance on rising edge of a button
            if (gamepad1.a) {
                if (allianceToggle == false) {
                    if (alliance == Alliance.RED) {
                        alliance = Alliance.BLUE;
                    } else {
                        alliance = Alliance.RED;
                    }
                }
                allianceToggle = true;
            } else {
                allianceToggle = false;
            }

            //displaying alliance settings
            if (alliance == Alliance.RED) {
                telemetry.addData("Alliance","RED");
            } else {
                telemetry.addData("Alliance","BLUE");
            }

            telemetry.update();

        }

        //this attaches actual functions to attributes found within the robot object
        robot.initializeRobot();

        //loop that runs while the program is active
        while (opModeIsActive()) {

            //running drive mode specific code
            switch (driveMode) {
                case MANUALDRIVE:

                    //Manual drive controls

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
                    if (gamepad1.x) {
                        robot.arm.setTargetPosition(-340);
                        robot.arm.setPower(1);
                    }

                    //setting arm to go down (0 ticks) when the right bumper is pressed
                    if (gamepad1.y) {
                        robot.arm.setTargetPosition(0);
                        if (robot.arm.getCurrentPosition() < -300) {
                            robot.arm.setPower(1);
                        } else {
                            robot.arm.setPower(0.3);
                        }
                    }

                    //setting the claw to move in, out, or neither depending on what combination of boolean inputs a and b give
                    if (gamepad1.a == gamepad1.b) {
                        robot.claw.setPower(0);
                        robot.wrist.setPosition(0.7);
                    } else {
                        if (gamepad1.a) {
                            robot.claw.setPower(1);
                            robot.wrist.setPosition(0.7);
                        } else {
                            robot.claw.setPower(-1);
                            robot.wrist.setPosition(0.65);
                        }
                    }

                    //launching the paper airplane when the guide button is clicked
                    if (gamepad1.guide) {
                        robot.launchDrone();
                    }

                    break;

                case AUTOSCORE:

                    //Auto scoring


                    break;

            }

            //Toggle scoring position on rising edge of a button
            if (gamepad2.a) {
                if (scoringPositionToggle == false) {
                    switch (scoringPosition) {
                        case LEFT:
                            scoringPosition = ScoringPosition.CENTER;
                            break;
                        case CENTER:
                            scoringPosition = ScoringPosition.RIGHT;
                            break;
                        case RIGHT:
                            scoringPosition = ScoringPosition.LEFT;
                            break;
                    }
                }
                allianceToggle = true;
            } else {
                allianceToggle = false;
            }

            if (gamepad2.guide) {
                if (driveModeToggle == false) {
                    switch (driveMode) {
                        case MANUALDRIVE:
                            driveMode = DriveMode.AUTOSCORE;
                            break;
                        case AUTOSCORE:
                            driveMode = DriveMode.MANUALDRIVE;
                            break;
                    }
                }
            } else {
                driveModeToggle = false;
            }

            //displaying alliance settings
            switch (scoringPosition) {
                case LEFT:
                    telemetry.addData("Scoring Position","LEFT");
                    break;
                case CENTER:
                    telemetry.addData("Scoring Position","CENTER");
                    break;
                case RIGHT:
                    telemetry.addData("Scoring Position","RIGHT");
                    break;
            }

            switch (driveMode) {
                case MANUALDRIVE:
                    telemetry.addData("Drive Mode","Manual Drive");
                    break;
                case AUTOSCORE:
                    telemetry.addData("Drive Mode","Auto Scoring");
                    break;
            }

            telemetry.update();
        }
    }
}

