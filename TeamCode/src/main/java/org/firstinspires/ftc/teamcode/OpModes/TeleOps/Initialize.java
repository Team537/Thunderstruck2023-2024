package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Hardware.Robot;

@TeleOp(name = "Initialize")

public class Initialize extends LinearOpMode {

    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.initializeRobot();
        waitForStart();

        while (opModeIsActive()) {
        }

    }

}
