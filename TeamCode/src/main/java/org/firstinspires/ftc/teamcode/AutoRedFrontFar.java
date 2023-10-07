package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto-RED-FRONT-FAR")

public class AutoRedFrontFar extends LinearOpMode {

    @Override
    public void runOpMode() {
        new Auto().runAutoFromParameters(Alliance.RED,StartPosition.FRONTSTAGE,EndPosition.FAR);
    }

}
