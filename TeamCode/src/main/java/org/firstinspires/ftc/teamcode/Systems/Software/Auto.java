package org.firstinspires.ftc.teamcode.Systems.Software;

import static org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance.RED;
import static org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.EndPosition.CORNER;
import static org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.EndPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.StartPosition.BACKSTAGE;
import static org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.StartPosition.FRONTSTAGE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveMode;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.EndPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.PropPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.StartPosition;
import org.firstinspires.ftc.teamcode.Systems.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Vector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Auto extends LinearOpMode {

    //defining a robot and allowing for opMode functions to be used in the object
    Robot robot = new Robot(this);
    ElapsedTime runtime = new ElapsedTime();

    /**
     * sleeps for s seconds using the robots internal clock
     * @param seconds time in seconds
     */

    /**
     * runs auto from manual parameters
     * @param alliance robot's alliance color
     * @param startPosition robot's starting position
     * @param endPosition robot's parking position
     */
    public void runAutoFromParameters(Alliance alliance, StartPosition startPosition, EndPosition endPosition) {
        runAuto(alliance, startPosition, endPosition);
    }

    /**
     * runs auto from manual switches mounted on the robot
     */
    public void runAutoFromSettings() {

        Alliance alliance = Alliance.RED;
        StartPosition startPosition = BACKSTAGE;
        EndPosition endPosition = CORNER;

        boolean allianceToggle = false;
        boolean startPositionToggle = false;
        boolean endPositionToggle = false;

        //Init loop to swap between auto settings
        while (!gamepad2.guide && !isStarted()) {

            //Toggle alliance on rising edge of a button
            if (gamepad2.a) {
                if (!allianceToggle) {
                    switch (alliance) {
                        case RED:
                            alliance = BLUE;
                            break;
                        case BLUE:
                            alliance = RED;
                            break;
                    }
                }
                allianceToggle = true;
            } else {
                allianceToggle = false;
            }

            //Toggle start position on rising edge of b button
            if (gamepad2.b) {
                if (!startPositionToggle) {
                    switch (startPosition) {
                        case BACKSTAGE:
                            startPosition = FRONTSTAGE;
                            break;
                        case FRONTSTAGE:
                            startPosition = BACKSTAGE;
                            break;
                    }
                }
                startPositionToggle = true;
            } else {
                startPositionToggle = false;
            }

            //Toggle end position on rising edge of x button
            if (gamepad2.x) {
                if (!endPositionToggle) {
                    switch (endPosition) {
                        case CORNER:
                            endPosition = MIDDLE;
                            break;
                        case MIDDLE:
                            endPosition = CORNER;
                            break;
                    }
                }
                endPositionToggle = true;
            } else {
                endPositionToggle = false;
            }

            //displaying alliance settings
            switch (alliance) {
                case RED:
                    telemetry.addData("Alliance:","RED");
                    break;
                case BLUE:
                    telemetry.addData("Alliance:","BLUE");
            }

            //displaying start position settings
            switch (startPosition) {
                case BACKSTAGE:
                    telemetry.addData("Start Position:","BACKSTAGE");
                    break;
                case FRONTSTAGE:
                    telemetry.addData("Start Position:","FRONTSTAGE");
            }

            //displaying end position settings
            switch (endPosition) {
                case CORNER:
                    telemetry.addData("End Position:","CORNER");
                    break;
                case MIDDLE:
                    telemetry.addData("End Position:","MIDDLE");
            }

            telemetry.update();

        }

        runAuto(alliance, startPosition, endPosition);
    }

    /**
     * drives the robot in any direction for a certain amount of time before stopping
     * @param linear the desired velocity vector
     * @param seconds number of seconds the robot will drive
     */
    public void linearDrive(Vector linear, double seconds) {
        robot.setDriveMode(DriveMode.MANUAL_DRIVE);
        robot.drivetrain.runDrivetrainFromCartesian(linear,0,robot.getBotHeading());
        robot.smartSleep(seconds);
        robot.drivetrain.stop();
    }

    /**
     * orients the robot for a certain amount of time before stopping
     * @param targetOrientation the orientation the robot will attempt to match
     * @param seconds number of seconds the robot will spend attempting to orient itself
     */
    public void orient(double targetOrientation, double seconds) {
        robot.setTargetOrientation(targetOrientation);
        robot.setDriveMode(DriveMode.ORIENT);
        robot.smartSleep(seconds);
        robot.setDriveMode(DriveMode.MANUAL_DRIVE);
    }

    /**
     * runs the autonomous code
     * @param alliance robot's alliance color
     * @param startPosition robot's starting position
     * @param endPosition robot's parking position
     */
    public void runAuto(Alliance alliance, StartPosition startPosition, EndPosition endPosition) {

        PropPosition propPosition;
        propPosition = PropPosition.MIDDLE;

        int[] colorMatrix = {0,0,0};

        //attaching motor functions to robot object
        robot.initializeRobot();

        waitForStart();

        //Drive forward to center of spike marks
        linearDrive(new Vector(0,0.5),2.1);

        //Rotate so robot is facing away from left spike mark
        orient(0,0.9);
        robot.smartSleep(0.5);

        //detect the prop at the first position (searches for different color depending on alliance)
        if (alliance == Alliance.RED) {
            colorMatrix[0] = robot.colorSensor.red();
        } else {
            colorMatrix[0] = robot.colorSensor.blue();
        }

        //Rotate so robot is facing away from middle spike mark
        orient(1.5 * Math.PI,0.9);

        //detect the prop at the second position (searches for different color depending on alliance)
        if (alliance == Alliance.RED) {
            colorMatrix[1] = robot.colorSensor.red();
        } else {
            colorMatrix[1] = robot.colorSensor.blue();
        }

        //Rotate so robot is facing away from right spike mark
        orient(Math.PI,0.9);

        //detect the prop at the third position (searches for different color depending on alliance)
        if (alliance == Alliance.RED) {
            colorMatrix[1] = robot.colorSensor.red();
        } else {
            colorMatrix[1] = robot.colorSensor.blue();
        }

        if (colorMatrix[0] >= colorMatrix[1] && colorMatrix[0] >= colorMatrix[2]) {
            propPosition = PropPosition.LEFT;
        }
        if (colorMatrix[1] >= colorMatrix[2] && colorMatrix[1] >= colorMatrix[0]) {
            propPosition = PropPosition.MIDDLE;
        }
        if (colorMatrix[2] >= colorMatrix[0] && colorMatrix[2] >= colorMatrix[1]) {
            propPosition = PropPosition.RIGHT;
        }

        //score differently depending on where the prop was detected
        switch (propPosition) {
            case LEFT:

                //drop pixel on left spike mark
                orient(1.5 * Math.PI,0.9);
                linearDrive(new Vector(-0.5,0),0.9);
                robot.dropPixel();
                robot.smartSleep(1);
                robot.stopDropping();

                //dislodge pixel (may be stuck)
                linearDrive(new Vector(0,0.5),0.3);
                linearDrive(new Vector(0,-0.5),0.3);

                //drive back to center
                linearDrive(new Vector(0.5,0),0.9);
                orient((alliance == Alliance.RED) ? Math.PI:0,0.9);

                break;

            case MIDDLE:

                //drop pixel on center spike mark
                linearDrive(new Vector(0,0.5),0.9);
                robot.dropPixel();
                robot.smartSleep(1);
                robot.stopDropping();

                //dislodge pixel (may be stuck)
                linearDrive(new Vector(0.5,0),0.3);
                linearDrive(new Vector(-0.5,0),0.3);

                //drive back to center
                linearDrive(new Vector(0,-0.5),0.9);
                orient((alliance == Alliance.RED) ? Math.PI:0,0.9);
                break;

            case RIGHT:

                //drop pixel on right spike mark
                orient(0.5 * Math.PI,0.9);
                linearDrive(new Vector(0.5,0),0.9);
                robot.dropPixel();
                robot.smartSleep(1);
                robot.stopDropping();

                //dislodge pixel (may be stuck)
                linearDrive(new Vector(0,-0.5),0.3);
                linearDrive(new Vector(0,0.5),0.3);

                //drive back to center
                linearDrive(new Vector(-0.5,0),0.9);
                orient((alliance == Alliance.RED) ? Math.PI:0,0.9);
                break;
        }

        //drives to the board (different depending on start position)
        switch (startPosition) {
            case BACKSTAGE:
                linearDrive(new Vector((alliance == Alliance.RED) ? 0.5:-0.5, 0),2.5);
                break;
            case FRONTSTAGE:
                linearDrive(new Vector(0, -0.5),2);
                linearDrive(new Vector((alliance == Alliance.RED) ? 0.5:-0.5, 0),5);
                linearDrive(new Vector(0, 0.5),2);
                break;
        }

        //makes robot raise arm
        robot.arm.armUp();
        robot.smartSleep(2);

        //moves drivetrain to right position for scoring
        switch (propPosition) {

            case LEFT:
                linearDrive(new Vector(0,-0.5),0.3);
                break;

            case RIGHT:
                linearDrive(new Vector(0,0.5),0.3);
                break;

        }

        //drops pre-loaded pixel and lowers arm
        robot.arm.discharge();
        robot.smartSleep(1);
        robot.arm.stopClaw();
        robot.arm.armDown();
        robot.smartSleep(2);

        //re-centers robot
        switch (propPosition) {

            case LEFT:
                linearDrive(new Vector(0,0.5),0.3);
                break;

            case RIGHT:
                linearDrive(new Vector(0,-0.5),0.3);
                break;

        }

        //orients so field centric is correct upon TeleOp
        orient(0.5 * Math.PI,0.9);

        //splits depending on target end position
        switch (endPosition) {
            case CORNER:
                linearDrive(new Vector(0,-0.5),2);
                break;
            case MIDDLE:
                linearDrive(new Vector(0,0.5),1.8);
                break;
        }

        //drives into corner and reorients servo
        linearDrive(new Vector((alliance == Alliance.RED) ? 0.5:-0.5,0),1);
        robot.arm.wrist.setPosition(0.5);

    }

}