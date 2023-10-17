package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-BLUE-BACK-CORNER")

public class AutoBlueBackCorner extends Auto {

    @Override
    public void runOpMode() {
        runAutoFromParameters(Alliance.BLUE,StartPosition.BACKSTAGE,EndPosition.CORNER);
    }

}
