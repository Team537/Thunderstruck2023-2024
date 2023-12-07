package org.firstinspires.ftc.teamcode.Systems.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.MotorMatrix;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.Vector;

public class Arm {

    //defining individual actuators
    public DcMotor shoulder;
    public Servo wrist;
    public CRServo claw;

    boolean armUp = false;

    //easy to change variables
    private final int DEFAULT_ARM_UP_POSITION = -230;
    private final double DEFAULT_ARM_UP_POWER = 1.0;
    private final int DEFAULT_ARM_DOWN_POSITION = 0;
    private final double DEFAULT_ARM_DOWN_POWER = 0.8;

    private final double DEFAULT_WRIST_NEUTRAL_POSITION = 0.55;//0.6;
    private final double DEFAULT_WRIST_ACTIVE_POSITION = 0.36;

    public int armUpPosition = DEFAULT_ARM_UP_POSITION;
    public double armUpPower = DEFAULT_ARM_UP_POWER;
    public int armDownPosition = DEFAULT_ARM_DOWN_POSITION;
    public double armDownPower = DEFAULT_ARM_DOWN_POWER;

    public double wristNeutralPosition = DEFAULT_WRIST_NEUTRAL_POSITION;
    public double wristActivePosition = DEFAULT_WRIST_ACTIVE_POSITION;

    public PID armPID = new PID(0.01,0.0,0.01,0.05);

    public double error;
    public double feedForward;

    public void intake() {
        claw.setPower(-1);
        wrist.setPosition(wristActivePosition);
    }

    public void discharge() {
        claw.setPower(1);
        wrist.setPosition(wristNeutralPosition);
    }

    public void stopClaw() {
        claw.setPower(0);
        wrist.setPosition(wristNeutralPosition);
    }

    public void armUp() {
        armPID.setTarget(armUpPosition,armUpPosition - shoulder.getCurrentPosition());
    }

    public void armDown() {
        armPID.setTarget(armDownPosition,armDownPosition - shoulder.getCurrentPosition());
    }

    public void updateArm() {
        error = armPID.getTarget() - shoulder.getCurrentPosition();
        feedForward = Math.signum(error);//0.5 * Math.signum(error) - 0.5 * Math.cos( (armPID.getTarget() - 70) * (Math.PI/300) );
        shoulder.setPower(armPID.calculate(error,feedForward));
    }

}
