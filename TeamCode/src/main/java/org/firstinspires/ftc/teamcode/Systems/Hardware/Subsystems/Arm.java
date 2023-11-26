package org.firstinspires.ftc.teamcode.Systems.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.MotorMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Vector;

public class Arm {

    //defining individual actuators
    public DcMotor shoulder;
    public Servo wrist;
    public CRServo claw;

    //easy to change variables
    private final int DEFAULT_ARM_UP_POSITION = -240;
    private final double DEFAULT_ARM_UP_POWER = 1.0;
    private final int DEFAULT_ARM_DOWN_POSITION = 0;
    private final double DEFAULT_ARM_DOWN_POWER = 0.8;

    private final double DEFAULT_WRIST_NEUTRAL_POSITION = 0.6;
    private final double DEFAULT_WRIST_ACTIVE_POSITION = 0.4;

    public int armUpPosition = DEFAULT_ARM_UP_POSITION;
    public double armUpPower = DEFAULT_ARM_UP_POWER;
    public int armDownPosition = DEFAULT_ARM_DOWN_POSITION;
    public double armDownPower = DEFAULT_ARM_DOWN_POWER;

    public double wristNeutralPosition = DEFAULT_WRIST_NEUTRAL_POSITION;
    public double wristActivePosition = DEFAULT_WRIST_ACTIVE_POSITION;


    public void intake() {
        claw.setPower(-1);
        wrist.setPosition(wristActivePosition);
    }

    public void discharge() {
        claw.setPower(1);
        wrist.setPosition(wristActivePosition);
    }

    public void stopClaw() {
        claw.setPower(0);
        wrist.setPosition(wristNeutralPosition);
    }

    public void armUp() {
        shoulder.setTargetPosition(armUpPosition);
        shoulder.setPower(armUpPower);
    }

    public void armDown() {
        shoulder.setTargetPosition(armDownPosition);
        shoulder.setPower(armDownPower);
    }

}
