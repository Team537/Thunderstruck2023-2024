package org.firstinspires.ftc.teamcode.OpModes.Autonomous.ManualSettings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.Auto;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.EndPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.StartPosition;

@Autonomous(name="Auto-BLUE-FRONT-CORNER",group = "manual")

public class AutoBlueFrontCorner extends Auto {

    @Override
    public void runOpMode() {
        runAutoFromParameters(Alliance.BLUE, StartPosition.FRONTSTAGE, EndPosition.CORNER);
    }

}