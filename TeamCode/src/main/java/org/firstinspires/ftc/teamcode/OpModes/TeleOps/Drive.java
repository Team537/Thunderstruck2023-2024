//Importing packages
package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveMode;
import org.firstinspires.ftc.teamcode.Systems.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveSpeed;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.FieldOfReference;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.ScoringPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.TurningMode;
import org.firstinspires.ftc.teamcode.Utilities.TargetTurn;
import org.firstinspires.ftc.teamcode.Utilities.Vector;

@TeleOp(name = "Drive")

public class Drive extends LinearOpMode {

    //defining variables used later in the code
    double x = 0;
    double y = 0;
    double rx = 0;
    double ry = 0;

    double turn = 0;

    boolean turningModeToggle = false;
    boolean allianceToggle = false;
    boolean scoringPositionToggle = false;
    boolean fieldOfReferenceToggle = false;

    boolean orientInvertToggle = false;
    boolean orientSetToggle = false;

    double secondsUpdate = 0;
    int updatesPerSecond = 0;
    int loopCounter = 0;

    Vector joystickOrient;
    double target = 0;

    //creating a robot object, every input (ex. reading a sensor value) and output (ex. running a motor) is run through this class
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {

        //this attaches actual functions to attributes found within the robot object
        robot.initializeRobot();

        waitForStart();

        //loop that runs while the program is active
        while (opModeIsActive()) {

            //some code to get the update speed
            loopCounter++;
            if (robot.runtime.seconds() >= secondsUpdate + 1) {
                secondsUpdate += 1;
                updatesPerSecond = loopCounter;
                loopCounter = 0;
            }

            robot.update();

            telemetry.addData("Position:",robot.getPosition().string());
            telemetry.addData("Orientation:",Float.toString( (float) (robot.getBotHeading() / Math.PI) ) + "π" );
            if (robot.driveMode == DriveMode.ORIENT) {
                telemetry.addData("Target Orientation:",Float.toString( (float) (robot.getTargetOrientation() / Math.PI) ) + "π" );
            }
            telemetry.addData("Updates per Second:",updatesPerSecond);

            //running drive mode specific code
            switch (robot.driveMode) {
                case MANUAL_DRIVE:

                    //Manual drive controls

                    //setting variables to gamepad values
                    x = gamepad1.left_stick_x;
                    y = -gamepad1.left_stick_y;
                    rx = gamepad1.right_stick_x;
                    ry = -gamepad1.right_stick_y;

                    switch (robot.turningMode) {
                        case STANDARD:
                            turn = -rx;
                            break;
                        case STANDARD_SNAP:
                            if (rx == 0) {
                                turn = TargetTurn.getTurn(robot.getBotHeading(), (0.5 * Math.PI) * Math.round( robot.getBotHeading() / (0.5 * Math.PI) ));
                            } else {
                                turn = -rx;
                            }
                            break;
                        case TARGET:
                            if (new Vector(rx,ry).magnitude() > 0.9) {
                                target = new Vector(rx,ry).angle();
                            }
                            turn = TargetTurn.getTurn(robot.getBotHeading(),target);;
                            break;
                        case TARGET_SNAP:
                            if (new Vector(rx,ry).magnitude() > 0.9) {
                                target = new Vector(rx,ry).angle();
                            }
                            turn = TargetTurn.getTurn(robot.getBotHeading(), (0.5 * Math.PI) * Math.round(target / (0.5 * Math.PI)));
                            break;
                    }

                    //setting the drivetrain speed as a function of input values
                    robot.drivetrain.runDrivetrainFromCartesian(new Vector(x,y),turn, (robot.fieldOfReference == FieldOfReference.FIELD_CENTRIC) ? robot.getBotHeading() : 0.5 * Math.PI );

                    //setting arm to go up when the left bumper is pressed
                    if (gamepad1.y) {
                        robot.arm.armUp();
                    }

                    //setting arm to go down when the right bumper is pressed
                    if (gamepad1.x) {
                        robot.arm.armDown();
                    }

                    //setting the claw to move in, out, or neither depending on what combination of boolean inputs a and b give
                    if (gamepad1.a == gamepad1.b) {
                        robot.arm.stopClaw();
                    } else {
                        if (gamepad1.a) {
                            robot.arm.discharge();
                        } else {
                            robot.arm.intake();
                        }
                    }

                    //launching the paper airplane when the guide button is clicked
                    if (gamepad1.guide) {
                        robot.launchDrone();
                    }

                    if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                        robot.dropPixel();
                    } else {
                        robot.stopDropping();
                    }

                    if (gamepad1.left_bumper) {
                        robot.drivetrain.setDriveSpeed(DriveSpeed.STANDARD);
                    }
                    if (gamepad1.right_bumper) {
                        robot.drivetrain.setDriveSpeed(DriveSpeed.PRECISE);
                    }

                    break;

                case ERROR_CORRECT:

                    //error correct controls

                    //resets yaw when guide button is clicked
                    if (gamepad1.guide) {
                        robot.imu.resetYaw();
                    }

                    //manual changes to arm heights
                    if (gamepad1.a) {
                        robot.arm.armDownPosition += 1;
                    }
                    if (gamepad1.b) {
                        robot.arm.armDownPosition -= 1;
                    }
                    if (gamepad1.y) {
                        robot.arm.armUpPosition += 1;
                    }
                    if (gamepad1.x) {
                        robot.arm.armUpPosition -= 1;
                    }

                    //manual changes to wrist orientations
                    if (gamepad1.right_bumper) {
                        robot.arm.wristActivePosition += 0.01;
                    }
                    if (gamepad1.left_bumper) {
                        robot.arm.wristActivePosition -= 0.01;
                    }
                    if (gamepad1.right_trigger > 0) {
                        robot.arm.wristNeutralPosition += 0.01;
                    }
                    if (gamepad1.left_trigger > 0) {
                        robot.arm.wristNeutralPosition -= 0.01;
                    }

                    //manual changes to drive speed
                    if (gamepad1.left_stick_y > 0.9) {
                        robot.drivetrain.standardDriveSpeed += 0.01;
                    }
                    if (gamepad1.left_stick_y < -0.9) {
                        robot.drivetrain.standardDriveSpeed -= 0.01;
                    }
                    if (gamepad1.left_stick_x > 0.9) {
                        robot.drivetrain.preciseDriveSpeed += 0.01;
                    }
                    if (gamepad1.left_stick_x < -0.9) {
                        robot.drivetrain.preciseDriveSpeed -= 0.01;
                    }

                    //manual changes to arm power
                    if (gamepad1.right_stick_y > 0.9) {
                        robot.arm.armDownPower += 0.01;
                    }
                    if (gamepad1.right_stick_y < -0.9) {
                        robot.arm.armDownPower -= 0.01;
                    }
                    if (gamepad1.right_stick_x > 0.9) {
                        robot.arm.armUpPower += 0.01;
                    }
                    if (gamepad1.right_stick_x< -0.9) {
                        robot.arm.armUpPower -= 0.01;
                    }

                    break;

                case ORIENT:

                    //orient controls

                    //resets yaw when guide button is clicked
                    if (gamepad1.guide) {
                        robot.imu.resetYaw();
                    }

                    if (gamepad1.b) {
                        robot.setTargetOrientation(0); //right
                    }
                    if (gamepad1.y) {
                        robot.setTargetOrientation(0.5 * Math.PI); //forwards
                    }
                    if (gamepad1.x) {
                        robot.setTargetOrientation(Math.PI); //left
                    }
                    if (gamepad1.a) {
                        robot.setTargetOrientation(1.5 * Math.PI); //backwards
                    }

                    //make changes to target orientation
                    if (gamepad1.left_stick_y > 0.9) {
                        robot.setTargetOrientation(robot.getTargetOrientation() - 0.002);
                    }
                    if (gamepad1.left_stick_y < -0.9) {
                        robot.setTargetOrientation(robot.getTargetOrientation() + 0.002);
                    }
                    if (gamepad1.left_stick_x > 0.9) {
                        robot.setTargetOrientation(robot.getTargetOrientation() - 0.01);
                    }
                    if (gamepad1.left_stick_x < -0.9) {
                        robot.setTargetOrientation(robot.getTargetOrientation() + 0.01);
                    }

                    if (gamepad1.left_bumper) {
                        if (!orientInvertToggle) {
                            robot.setTargetOrientation(robot.getTargetOrientation() + Math.PI);
                        }
                        orientInvertToggle = true;
                    } else {
                        orientInvertToggle = false;
                    }

                    if (gamepad1.right_bumper) {
                        if (!orientSetToggle) {
                            robot.setTargetOrientation(robot.getBotHeading());
                        }
                        orientSetToggle = true;
                    } else {
                        orientSetToggle = false;
                    }

                    //sets the orientation to the joystick
                    joystickOrient = new Vector(gamepad1.right_stick_x,-gamepad1.right_stick_y);

                    if (joystickOrient.magnitude() > 0.9) {
                        robot.setTargetOrientation(joystickOrient.angle());
                    }

                    break;

            }

            //stop robot upon emergency brake (both operator and driver can use this)
            if (gamepad2.back || gamepad1.back) {
                robot.setDriveMode(DriveMode.EMERGENCY_BRAKE);
            }

            //Toggle alliance on rising edge of a button
            if (gamepad2.a) {
                if (!allianceToggle) {
                    switch (robot.alliance) {
                        case RED:
                            robot.alliance = Alliance.BLUE;
                            break;
                        case BLUE:
                            robot.alliance = Alliance.RED;
                            break;
                    }
                }
                allianceToggle = true;
            } else {
                allianceToggle = false;
            }

            //Toggle scoring position on rising edge of a button
            if (gamepad2.b) {
                if (!scoringPositionToggle) {
                    switch (robot.scoringPosition) {
                        case LEFT:
                            robot.scoringPosition = ScoringPosition.MIDDLE;
                            break;
                        case MIDDLE:
                            robot.scoringPosition = ScoringPosition.RIGHT;
                            break;
                        case RIGHT:
                            robot.scoringPosition = ScoringPosition.LEFT;
                            break;
                    }
                }
                scoringPositionToggle = true;
            } else {
                scoringPositionToggle = false;
            }

            if (gamepad2.y) {
                if (!fieldOfReferenceToggle) {
                    switch (robot.fieldOfReference) {
                        case FIELD_CENTRIC:
                            robot.fieldOfReference = FieldOfReference.ROBOT_CENTRIC;
                            break;
                        case ROBOT_CENTRIC:
                            robot.fieldOfReference = FieldOfReference.FIELD_CENTRIC;
                            break;
                    }
                }
                fieldOfReferenceToggle = true;
            } else {
                fieldOfReferenceToggle = false;
            }

            if (gamepad2.x) {
                if (!turningModeToggle) {
                    switch (robot.turningMode) {
                        case STANDARD:
                            robot.turningMode = TurningMode.STANDARD_SNAP;
                            break;
                        case STANDARD_SNAP:
                            robot.turningMode = TurningMode.TARGET;
                            break;
                        case TARGET:
                            target = robot.getBotHeading();
                            robot.turningMode = TurningMode.TARGET_SNAP;
                            break;
                        case TARGET_SNAP:
                            target = (0.5 * Math.PI) * Math.round( robot.getBotHeading() / (0.5 * Math.PI) );
                            robot.turningMode = TurningMode.STANDARD;
                            break;
                    }
                }
                turningModeToggle = true;
            } else {
                turningModeToggle = false;
            }

            //drive mode quick selects (both operator and driver can use these)
            if ( (gamepad2.dpad_up || gamepad1.dpad_up) && robot.getDriveMode() != DriveMode.MANUAL_DRIVE) {
                robot.setDriveMode(DriveMode.MANUAL_DRIVE);
            }
            if ( (gamepad2.dpad_left || gamepad1.dpad_left) && robot.getDriveMode() != DriveMode.AUTO_OP) {
                robot.setDriveMode(DriveMode.AUTO_OP);
            }
            if ( (gamepad2.dpad_right || gamepad1.dpad_right) && robot.getDriveMode() != DriveMode.ORIENT) {
                robot.setDriveMode(DriveMode.ORIENT);
            }
            if ( (gamepad2.dpad_down || gamepad1.dpad_down) && robot.getDriveMode() != DriveMode.ERROR_CORRECT) {
                robot.setDriveMode(DriveMode.ERROR_CORRECT);
            }

            //Displays Alliance
            switch (robot.alliance) {
                case RED:
                    telemetry.addData("Alliance:","RED");
                    break;
                case BLUE:
                    telemetry.addData("Alliance:","BLUE");
            }

            //Displays Scoring Position
            switch (robot.scoringPosition) {
                case LEFT:
                    telemetry.addData("Scoring Position:","LEFT");
                    break;
                case MIDDLE:
                    telemetry.addData("Scoring Position:","CENTER");
                    break;
                case RIGHT:
                    telemetry.addData("Scoring Position:","RIGHT");
                    break;
            }

            //Displays Field Of Reference
            switch (robot.fieldOfReference) {
                case FIELD_CENTRIC:
                    telemetry.addData("Field Of Reference:","FIELD CENTRIC");
                    break;
                case ROBOT_CENTRIC:
                    telemetry.addData("Field Of Reference:","ROBOT CENTRIC");
                    break;
            }

            //Displays Turning Mode
            switch (robot.turningMode) {
                case STANDARD:
                    telemetry.addData("Turning Mode:","STANDARD");
                    break;
                case STANDARD_SNAP:
                    telemetry.addData("Turning Mode:","STANDARD SNAP");
                    break;
                case TARGET:
                    telemetry.addData("Turning Mode:","TARGET");
                    break;
                case TARGET_SNAP:
                    telemetry.addData("Turning Mode:","TARGET SNAP");
                    break;
            }

            //Displays Committed Drive Mode
            switch (robot.driveMode) {
                case MANUAL_DRIVE:
                    telemetry.addData("Drive Mode:","MANUAL DRIVE");
                    break;
                case AUTO_OP:
                    telemetry.addData("Drive Mode:","AUTO OP");
                    break;
                case ERROR_CORRECT:
                    telemetry.addData("Drive Mode:","ERROR CORRECT");
                    break;
                case ORIENT:
                    telemetry.addData("Drive Mode:","ORIENT");
                    break;
                case EMERGENCY_BRAKE:
                    telemetry.addData("Drive Mode:","EMERGENCY BRAKE");
                    break;
            }

            //Displays error correct settings only when in error correct mode
            if (robot.driveMode == DriveMode.ERROR_CORRECT) {
                telemetry.addData("Arm Up Position:",robot.arm.armUpPosition);
                telemetry.addData("Arm Down Position:",robot.arm.armDownPosition);
                telemetry.addData("Arm Up Power:",robot.arm.armUpPower);
                telemetry.addData("Arm Down Power:",robot.arm.armDownPower);
                telemetry.addData("Wrist Active Position:",robot.arm.wristActivePosition);
                telemetry.addData("Wrist Neutral Position:",robot.arm.wristNeutralPosition);
                telemetry.addData("Standard Drive Speed:",robot.drivetrain.standardDriveSpeed);
                telemetry.addData("Precise Drive Speed:",robot.drivetrain.preciseDriveSpeed);
            }

            telemetry.addData("arm power",robot.arm.shoulder.getPower());

            telemetry.update();
        }
    }
}