package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-RED-BACK-MIDDLE")

public class AutoRedBackMiddle extends Auto {

    @Override
    public void runOpMode() {
        runAutoFromParameters(Alliance.RED,StartPosition.BACKSTAGE,EndPosition.MIDDLE);
    }

}
