package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class Auto {

    //defining a robot and allowing for opMode functions to be used in the object
    LinearOpMode opMode;
    Robot robot = new Robot();
    ElapsedTime runtime = new ElapsedTime();

    //sleeps for s seconds using the robots internal clock
    public void smartSleep(double seconds) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < seconds) ) {
        }
    }

    //runs auto from manual parameters
    public void runAutoFromParameters(Alliance alliance, StartPosition startPosition, EndPosition endPosition) {
        runAuto(false, alliance, startPosition, endPosition);
    }

    //runs auto from manual switches mounted on the robot
    public void runAutoFromSwitches() {

        Alliance alliance;
        StartPosition startPosition;
        EndPosition endPosition;

        //detecting alliance switch
        if (robot.settingSwitches.getAlliance()) {
            alliance = Alliance.RED;
        } else {
            alliance = Alliance.BLUE;
        }

        //detecting start position switch
        if (robot.settingSwitches.getStartPosition()) {
            startPosition = StartPosition.FRONTSTAGE;
        } else {
            startPosition = StartPosition.BACKSTAGE;;
        }

        //detecting end position switch
        if (robot.settingSwitches.getEndPosition()) {
            endPosition = EndPosition.CORNER;
        } else {
            endPosition = EndPosition.MIDDLE;
        }

        runAuto(false, alliance, startPosition, endPosition);
    }

    //running the auto, except it will use parameters to determine what to do
    public void runAutoFromSensors() {
        runAuto(true,Alliance.RED,StartPosition.FRONTSTAGE,EndPosition.CORNER);
    }

    public void runAuto(boolean useSensors, Alliance alliance, StartPosition startPosition, EndPosition endPosition) {

        //attaching motor functions to robot object
        robot.initializeRobot();

        //creates a simple way of inverting some of the x and rx values plugged into a motor matrix. This is because these values are usually negated when switching alliance
        //if you want a value negated when the alliance color is blue, simply multiply it by "allianceInvert"
        int allianceInvert;
        if (alliance == Alliance.RED) {
            allianceInvert = 1;
        } else {
            allianceInvert = -1;
        }

        smartSleep(1);
        robot.drivetrain.runDrivetrainFromCartesian(0, 0.5, 0,robot.getBotHeading());
        smartSleep(1);
        robot.drivetrain.stop();
        robot.arm.setTargetPosition(-80);
        smartSleep(1);

        robot.drivetrain.runDrivetrainFromCartesian(0,0,-0.5,robot.getBotHeading());
        smartSleep(1);
        robot.drivetrain.runDrivetrainFromCartesian(0.5,0,0,robot.getBotHeading());
        smartSleep(1);
        robot.drivetrain.stop();
        if (alliance == Alliance.RED) {

        } else {

        }

    }

}

enum Alliance {
    RED,
    BLUE,
}

enum StartPosition {
    FRONTSTAGE,
    BACKSTAGE,
}

enum EndPosition {
    CORNER,
    MIDDLE,
}