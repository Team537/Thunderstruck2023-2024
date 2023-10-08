package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-BLUE-BACK-MIDDLE")

public class AutoBlueBackMiddle extends LinearOpMode {

    @Override
    public void runOpMode() {
        new Auto().runAutoFromParameters(Alliance.BLUE,StartPosition.BACKSTAGE,EndPosition.MIDDLE);
    }

}
