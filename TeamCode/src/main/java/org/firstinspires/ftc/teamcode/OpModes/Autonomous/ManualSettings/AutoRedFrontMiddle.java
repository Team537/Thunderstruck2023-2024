package org.firstinspires.ftc.teamcode.OpModes.Autonomous.ManualSettings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.Auto;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.EndPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.StartPosition;

@Autonomous(name="Auto-RED-FRONT-MIDDLE",group = "manual")

public class AutoRedFrontMiddle extends Auto {

    @Override

    public void runOpMode() {
        runAutoFromParameters(Alliance.RED, StartPosition.FRONTSTAGE, EndPosition.MIDDLE);
    }

}
