package org.firstinspires.ftc.teamcode.Systems.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveSpeed;
import org.firstinspires.ftc.teamcode.Utilities.MotorMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Vector;

public class Drivetrain {

    //defining individual motor attributes
    public DcMotor lfMotor;
    public DcMotor rfMotor;
    public DcMotor rbMotor;
    public DcMotor lbMotor;

    private final double DEFAULT_STANDARD_DRIVE_SPEED = 1;
    private final double DEFAULT_PRECISE_DRIVE_SPEED = 0.3;
    public double standardDriveSpeed = DEFAULT_STANDARD_DRIVE_SPEED;
    public double preciseDriveSpeed = DEFAULT_PRECISE_DRIVE_SPEED;

    public double speed;

    public void setDriveSpeed(DriveSpeed driveSpeed) {
        switch (driveSpeed) {
            case STANDARD:
                speed = standardDriveSpeed;
                break;
            case PRECISE:
                speed = preciseDriveSpeed;
                break;
        }
    }

    /**
     * runs drivetrain with a MotorMatrix parameter
     * @param motorMatrix Object containing individual motor powers
     */
    public void runDrivetrain(MotorMatrix motorMatrix) {
        lfMotor.setPower(motorMatrix.lf);
        rfMotor.setPower(motorMatrix.rf);
        lbMotor.setPower(motorMatrix.lb);
        rbMotor.setPower(motorMatrix.rb);
    }

    /**
     * runs drivetrain with parameters; uses a MotorMatrix to calculate parameters
     * @param linear the linear velocity vector
     * @param rx the magnitude of the rotational vector
     * @param botHeading the orientation of the robot
     */
    public void runDrivetrainFromCartesian(Vector linear, double rx, double botHeading) {
        MotorMatrix motorMatrix = new MotorMatrix();
        motorMatrix.setMotorMatrixFromCartesian(linear,rx,botHeading,speed);
        lfMotor.setPower(motorMatrix.lf);
        rfMotor.setPower(motorMatrix.rf);
        lbMotor.setPower(motorMatrix.lb);
        rbMotor.setPower(motorMatrix.rb);
    }

    //stops all drivetrain motors
    public void stop() {
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        lbMotor.setPower(0);
        rbMotor.setPower(0);
    }

    //runs all drivetrain motors in a way so no net velocity is applied to the robot
    public void rave() {
        lfMotor.setPower(1);
        rfMotor.setPower(1);
        lbMotor.setPower(-1);
        rbMotor.setPower(-1);
    }

}
