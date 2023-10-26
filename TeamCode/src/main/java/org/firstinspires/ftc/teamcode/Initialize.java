package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Initialize")

public class Initialize extends LinearOpMode {

    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.initializeRobot();
    }

}
