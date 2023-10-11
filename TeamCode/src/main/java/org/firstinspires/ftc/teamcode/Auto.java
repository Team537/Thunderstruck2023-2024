package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Auto {
    //defining motor variables
    private DcMotor lfMotor = null;
    private DcMotor rfMotor = null;
    private DcMotor lbMotor = null;
    private DcMotor rbMotor = null;

    Drivetrain drivetrain = new Drivetrain();

    //defining imu variable
    IMU imu = null;

    public void runAutoFromParameters(Alliance alliance, StartPosition startPosition, EndPosition endPosition) {
        runAuto(false, alliance, startPosition, endPosition);
    }

    public void runAutoFromSwitches() {

        Alliance alliance;
        StartPosition startPosition;
        EndPosition endPosition;

        if (true /*detect switch 1*/) {
            alliance = Alliance.RED;
        } else {
            alliance = Alliance.BLUE;
        }

        if (true /*detect switch 2*/) {
            startPosition = StartPosition.FRONTSTAGE;
        } else {
            startPosition = StartPosition.BACKSTAGE;;
        }

        if (true /*detect switch 3*/) {
            endPosition = EndPosition.CORNER;
        } else {
            endPosition = EndPosition.MIDDLE;
        }

        runAuto(false, alliance, startPosition, endPosition);
    }

    public void runAutoFromSensors() {
        runAuto(true,null,null,null);
    }

    static void runAuto(boolean useSensors, Alliance alliance, StartPosition startPosition, EndPosition endPosition) {

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