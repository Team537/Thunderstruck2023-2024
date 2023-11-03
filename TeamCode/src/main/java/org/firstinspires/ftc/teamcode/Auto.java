package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.Alliance.RED;
import static org.firstinspires.ftc.teamcode.EndPosition.CORNER;
import static org.firstinspires.ftc.teamcode.EndPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.StartPosition.BACKSTAGE;
import static org.firstinspires.ftc.teamcode.StartPosition.FRONTSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public abstract class Auto extends LinearOpMode {

    //defining a robot and allowing for opMode functions to be used in the object
    Robot robot = new Robot(this);
    ElapsedTime runtime = new ElapsedTime();

    //sleeps for s seconds using the robots internal clock
    public void smartSleep(double seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
        }
    }

    //runs auto from manual parameters
    public void runAutoFromParameters(Alliance alliance, StartPosition startPosition, EndPosition endPosition) {
        runAuto(false, alliance, startPosition, endPosition);
    }

    //runs auto from manual switches mounted on the robot
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

    public void runOpMode() {

    }

    //running the auto, except it will use parameters to determine what to do
    public void runAutoFromSensors() {
        waitForStart();
        runAuto(true, Alliance.RED, FRONTSTAGE, CORNER);
    }

    public void runAuto(boolean useSensors, Alliance alliance, StartPosition startPosition, EndPosition endPosition) {

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
        robot.drivetrain.runDrivetrainFromCartesian(0, 0.5, 0, robot.getBotHeading());
        smartSleep(1);
        robot.drivetrain.stop();
        robot.arm.setTargetPosition(-80);
        smartSleep(1);

        robot.drivetrain.runDrivetrainFromCartesian(0, 0, -0.5, robot.getBotHeading());
        smartSleep(1);
        robot.drivetrain.runDrivetrainFromCartesian(0.5, 0, 0, robot.getBotHeading());
        smartSleep(1);
        robot.drivetrain.stop();
        if (alliance == Alliance.RED) {

        } else {

        }

    }

}