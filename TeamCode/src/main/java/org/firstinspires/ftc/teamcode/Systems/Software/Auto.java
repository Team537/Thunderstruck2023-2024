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

    public void smartSleep(double seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            robot.update();
        }
    }

    /**
     * runs auto from manual parameters
     * @param alliance robot's alliance color
     * @param startPosition robot's starting position
     * @param endPosition robot's parking position
     */
    public void runAutoFromParameters(Alliance alliance, StartPosition startPosition, EndPosition endPosition) {
        runAuto(false, alliance, startPosition, endPosition);
    }

    /**
     * runs auto from manual switches mounted on the robot
     */
    public void runAutoFromSettings() {

        Alliance alliance = Alliance.RED;
        StartPosition startPosition = BACKSTAGE;
        EndPosition endPosition = CORNER;

        boolean allianceState = false;
        boolean startPositionState = false;
        boolean endPositionState = false;

        //Init loop to swap between auto settings
        while (!isStarted()) {

            //Toggle alliance on rising edge of a button
            if (gamepad1.a) {
                if (allianceState == false) {
                    switch (alliance) {
                        case RED:
                            alliance = BLUE;
                            break;
                        case BLUE:
                            alliance = RED;
                            break;
                    }
                }
                allianceState = true;
            } else {
                allianceState = false;
            }

            //Toggle start position on rising edge of b button
            if (gamepad1.b) {
                if (startPositionState == false) {
                    switch (startPosition) {
                        case BACKSTAGE:
                            startPosition = FRONTSTAGE;
                            break;
                        case FRONTSTAGE:
                            startPosition = BACKSTAGE;
                            break;
                    }
                }
                startPositionState = true;
            } else {
                startPositionState = false;
            }

            //Toggle end position on rising edge of x button
            if (gamepad1.x) {
                if (endPositionState == false) {
                    switch (endPosition) {
                        case CORNER:
                            endPosition = MIDDLE;
                            break;
                        case MIDDLE:
                            endPosition = CORNER;
                            break;
                    }
                }
                endPositionState = true;
            } else {
                endPositionState = false;
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

        runAuto(false, alliance, startPosition, endPosition);
    }

    /**
     * running the auto, except it will use parameters to determine what to do
     */
    public void runAutoFromSensors() {
        waitForStart();
        runAuto(true, Alliance.RED, FRONTSTAGE, CORNER);
    }

    /**
     * runs the autonomous code
     * @param useSensors whether or not the robot uses sensors to determine its position
     * @param alliance robot's alliance color
     * @param startPosition robot's starting position
     * @param endPosition robot's parking position
     */
    public void runAuto(boolean useSensors, Alliance alliance, StartPosition startPosition, EndPosition endPosition) {

        PropPosition propPosition;

        propPosition = PropPosition.MIDDLE;

        //attaching motor functions to robot object
        robot.initializeRobot();

        waitForStart();

        //creates a simple way of inverting some of the x and rx values plugged into a motor matrix. This is because these values are usually negated when switching alliance
        //if you want a value negated when the alliance color is blue, simply multiply it by "allianceInvert"
        int allianceInvert;
        if (alliance == Alliance.RED) {
            allianceInvert = 1;
        } else {
            allianceInvert = -1;
        }

        waitForStart();

        smartSleep(1);
        robot.drivetrain.runDrivetrainFromCartesian(new Vector(0,0.5), 0, robot.getBotHeading());
        smartSleep(2.1);
        robot.drivetrain.stop();
        smartSleep(1);

        robot.setTargetOrientation( Math.PI/2 );
        robot.driveMode = DriveMode.ORIENT;
        smartSleep(0.9); //0.9
        robot.driveMode = DriveMode.MANUALDRIVE;
        robot.drivetrain.stop();
        smartSleep(0.5);

        //detect prop

        robot.setTargetOrientation( Math.PI );
        robot.driveMode = DriveMode.ORIENT;
        smartSleep(0.9); //0.9
        robot.driveMode = DriveMode.MANUALDRIVE;
        robot.drivetrain.stop();
        smartSleep(0.5);

        //detect prop

        robot.setTargetOrientation( 3 * (Math.PI/2) );
        robot.driveMode = DriveMode.ORIENT;
        smartSleep(0.9); //0.9
        robot.driveMode = DriveMode.MANUALDRIVE;
        smartSleep(0.5);

        //detect prop

        switch(propPosition) {
            case LEFT:
                robot.setTargetOrientation( Math.PI );
                robot.driveMode = DriveMode.ORIENT;
                smartSleep(0.9); //0.9
                robot.driveMode = DriveMode.MANUALDRIVE;
                robot.drivetrain.runDrivetrainFromCartesian(new Vector(-0.5,0), 0, robot.getBotHeading());
                smartSleep(0.9);
                robot.drivetrain.stop();
                smartSleep(0.3);
                robot.dropPixel();
                smartSleep(0.3);

                robot.drivetrain.runDrivetrainFromCartesian(new Vector(0.5,0),0,robot.getBotHeading());
                smartSleep(0.9);
                robot.setTargetOrientation( Math.PI/2 );
                robot.driveMode = DriveMode.ORIENT;
                smartSleep(0.9); //0.9
                robot.driveMode = DriveMode.MANUALDRIVE;
                robot.drivetrain.stop();

                break;

            case MIDDLE:
                smartSleep(0.9);
                robot.drivetrain.runDrivetrainFromCartesian(new Vector(0,0.5), 0, robot.getBotHeading());
                smartSleep(0.9);
                robot.drivetrain.stop();
                smartSleep(0.3);
                robot.dropPixel();
                smartSleep(0.3);

                robot.drivetrain.runDrivetrainFromCartesian(new Vector(0,-0.5),0,robot.getBotHeading());
                smartSleep(0.9);
                robot.setTargetOrientation( Math.PI/2 );
                robot.driveMode = DriveMode.ORIENT;
                smartSleep(0.9); //0.9
                robot.driveMode = DriveMode.MANUALDRIVE;
                robot.drivetrain.stop();
                break;

            case RIGHT:
                robot.setTargetOrientation( 0 );
                robot.driveMode = DriveMode.ORIENT;
                smartSleep(0.9); //0.9
                robot.driveMode = DriveMode.MANUALDRIVE;
                robot.drivetrain.runDrivetrainFromCartesian(new Vector(0.5,0), 0, robot.getBotHeading());
                smartSleep(0.9);
                robot.drivetrain.stop();
                smartSleep(0.3);
                robot.dropPixel();
                smartSleep(0.3);

                robot.drivetrain.runDrivetrainFromCartesian(new Vector(-0.5,0),0,robot.getBotHeading());
                smartSleep(0.9);
                robot.setTargetOrientation( Math.PI/2 );
                robot.driveMode = DriveMode.ORIENT;
                smartSleep(0.9); //0.9
                robot.driveMode = DriveMode.MANUALDRIVE;
                robot.drivetrain.stop();
                break;
        }

        // makes robot raise arm and drive towards board
        robot.arm.setTargetPosition(-340);
        robot.drivetrain.runDrivetrainFromCartesian(new Vector(-0.5, 0),0, robot.getBotHeading());
        smartSleep(2.5);
        robot.drivetrain.stop();

        //moves drivetrain to right position for scoring
        switch(propPosition){

            case LEFT:
                robot.drivetrain.runDrivetrainFromCartesian(new Vector(0, -0.5),0, robot.getBotHeading());
                smartSleep(0.3);
                robot.drivetrain.stop();
                break;

            case RIGHT:
                robot.drivetrain.runDrivetrainFromCartesian(new Vector(0,0.5), 0, robot.getBotHeading());
                smartSleep(0.3);
                robot.drivetrain.stop();
                break;

        }

        robot.claw.setPower(1);
        smartSleep(0.97);
        robot.claw.setPower(0);

        robot.arm.setTargetPosition(0);


        //
        switch(propPosition){

            case LEFT:
                robot.drivetrain.runDrivetrainFromCartesian(new Vector (0,0.5), 0, robot.getBotHeading());
                smartSleep(0.3);
                robot.drivetrain.stop();
                break;


            case RIGHT:
                robot.drivetrain.runDrivetrainFromCartesian(new Vector (0,-0.5), 0, robot.getBotHeading());
                smartSleep(0.3);
                robot.drivetrain.stop();
                break;
        }



    }

}