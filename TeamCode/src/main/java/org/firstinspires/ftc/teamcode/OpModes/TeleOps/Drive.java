//Importing packages
package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveMode;
import org.firstinspires.ftc.teamcode.Systems.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.ScoringPosition;
import org.firstinspires.ftc.teamcode.Utilities.Vector;

@TeleOp(name = "Drive")

public class Drive extends LinearOpMode {

    //defining variables used later in the code
    double x = 0;
    double y = 0;
    double rx = 0;

    DriveMode driveMode = DriveMode.MANUALDRIVE;

    DriveMode driveModeSetting = DriveMode.MANUALDRIVE;

    boolean driveModeSettingToggle = false;

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
                    switch (alliance) {
                        case RED:
                            alliance = Alliance.BLUE;
                            break;
                        case BLUE:
                            alliance = Alliance.RED;
                            break;
                    }
                }
                allianceToggle = true;
            } else {
                allianceToggle = false;
            }

            //displaying alliance settings
            switch (alliance) {
                case RED:
                    telemetry.addData("Alliance:","RED");
                    break;
                case BLUE:
                telemetry.addData("Alliance:","BLUE");
            }
            telemetry.update();

        }

        //this attaches actual functions to attributes found within the robot object
        robot.initializeRobot();

        //loop that runs while the program is active
        while (opModeIsActive()) {

            robot.update();

            telemetry.addData("Position:",robot.getPosition().string());
            telemetry.addData("Orientation:",robot.getBotHeading());
            telemetry.addData("TEMPORARY-TESTING-START-ANGLE:",robot.startAngle);

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
                    robot.drivetrain.runDrivetrainFromCartesian(new Vector(x,y),rx,robot.getBotHeading());

                    //setting arm to go up (-80 ticks) when the left bumper is pressed
                    if (gamepad1.x) {
                        robot.arm.setTargetPosition(-340);
                        robot.arm.setPower(1);
                    }

                    //setting arm to go down (0 ticks) when the right bumper is pressed
                    if (gamepad1.y) {
                        robot.arm.setTargetPosition(0);
                        robot.arm.setPower(0.8);
                    }

                    //setting the claw to move in, out, or neither depending on what combination of boolean inputs a and b give
                    if (gamepad1.a == gamepad1.b) {
                        robot.claw.setPower(0);
                        robot.wrist.setPosition(0.3);
                    } else {
                        if (gamepad1.a) {
                            robot.claw.setPower(1);
                            robot.wrist.setPosition(0.3);
                        } else {
                            robot.claw.setPower(-1);
                            robot.wrist.setPosition(0.25);
                        }
                    }

                    //launching the paper airplane when the guide button is clicked
                    if (gamepad1.guide) {
                        robot.launchDrone();
                    }

                    if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                        robot.dropPixel();
                    }

                    break;

                case AUTOSCORE:

                    //Auto scoring


                    break;

            }

            //Toggle alliance on rising edge of a button
            if (gamepad2.a) {
                if (allianceToggle == false) {
                    switch (alliance) {
                        case RED:
                            alliance = Alliance.BLUE;
                            break;
                        case BLUE:
                            alliance = Alliance.RED;
                            break;
                    }
                }
                allianceToggle = true;
            } else {
                allianceToggle = false;
            }

            //Toggle scoring position on rising edge of a button
            if (gamepad2.b) {
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
                scoringPositionToggle = true;
            } else {
                scoringPositionToggle = false;
            }

            if (gamepad2.x) {
                if (driveModeSettingToggle == false) {
                    switch (driveModeSetting) {
                        case MANUALDRIVE:
                            driveModeSetting = DriveMode.AUTOSCORE;
                            break;
                        case AUTOSCORE:
                            driveModeSetting = DriveMode.MANUALDRIVE;
                            break;
                    }
                }
                driveModeSettingToggle = true;
            } else {
                driveModeSettingToggle = false;
            }

            //commit drive mode when guide button is clicked
            if (gamepad2.guide) {
                driveMode = driveModeSetting;
            }

            //displaying alliance settings
            switch (alliance) {
                case RED:
                    telemetry.addData("Alliance:","RED");
                    break;
                case BLUE:
                    telemetry.addData("Alliance:","BLUE");
            }

            switch (scoringPosition) {
                case LEFT:
                    telemetry.addData("Scoring Position:","LEFT");
                    break;
                case CENTER:
                    telemetry.addData("Scoring Position:","CENTER");
                    break;
                case RIGHT:
                    telemetry.addData("Scoring Position:","RIGHT");
                    break;
            }

            switch (driveModeSetting) {
                case MANUALDRIVE:
                    telemetry.addData("Drive Mode Setting:","Manual Drive");
                    break;
                case AUTOSCORE:
                    telemetry.addData("Drive Mode Setting:","Auto Scoring");
                    break;
            }

            telemetry.addData("Press [GUIDE] to commit mode","");

            switch (driveMode) {
                case MANUALDRIVE:
                    telemetry.addData("Drive Mode Setting:","Manual Drive");
                    break;
                case AUTOSCORE:
                    telemetry.addData("Drive Mode Setting:","Auto Scoring");
                    break;
            }

            telemetry.update();
        }
    }
}

