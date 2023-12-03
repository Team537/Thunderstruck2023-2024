package org.firstinspires.ftc.teamcode.Systems.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Systems.Hardware.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Systems.Hardware.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.AutoScoringState;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.AutoStrategy;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.EndPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.FieldOfReference;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.ScoringPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.StartPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.TurningMode;
import org.firstinspires.ftc.teamcode.Utilities.TargetTurn;
import org.firstinspires.ftc.teamcode.Utilities.Vector;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveMode;

public class Robot {

    //defining attributes which will be used in the code (sensors, motors, and other variables)
    private LinearOpMode opMode;

    public Drivetrain drivetrain = new Drivetrain();
    public IMU imu;
    public Arm arm = new Arm();
    public Servo launcher;
    public CRServo dropper;
    public ColorSensor colorSensor;

    public double angleOffset;

    private Vector position = new Vector(0,0);
    private double lastLFPosition;
    private double lastRFPosition;
    private double lastRBPosition;
    private double lastLBPosition;
    private double lfPosition;
    private double rfPosition;
    private double rbPosition;
    private double lbPosition;

    private double targetOrientation = 0.5 * Math.PI;

    public DriveMode driveMode = DriveMode.MANUAL_DRIVE;
    public AutoScoringState autoScoringState = AutoScoringState.ORIENT;
    public Alliance alliance = Alliance.RED;
    public ScoringPosition scoringPosition = ScoringPosition.CENTER;
    public FieldOfReference fieldOfReference = FieldOfReference.FIELD_CENTRIC;
    public TurningMode turningMode = TurningMode.STANDARD;
    public StartPosition startPosition = StartPosition.BACKSTAGE;
    public EndPosition endPosition = EndPosition.CORNER;
    public AutoStrategy autoStrategy = AutoStrategy.SCORE;

    public final double TICKS_PER_INCH = 57.953;

    public ElapsedTime runtime = new ElapsedTime();
    public Robot(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;
    }

    /**
     * sleeps for a number seconds using the robots internal clock
     * @param seconds time in seconds
     */
    public void smartSleep(double seconds) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < seconds)) {
            this.update();
        }
    }

    /**
     * returns the bot heading of the robot
     * @return bot heading of the robot in radians
     */
    public double getBotHeading() {
        return ((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + angleOffset) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
    }

    /**
     * launches the paper drone
     */
    public void launchDrone() {
        launcher.setPosition(0);
    }

    /**
     * drops the pixel on the spike mark during autonomous
     */
    public void dropPixel() {
        dropper.setPower(-1);
    }

    /**
     * stops dropping the pixel
     */
    public void stopDropping() {
        dropper.setPower(0);
    }



    public void setTargetOrientation(double angle) {
        targetOrientation = (angle % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
    }

    public double getTargetOrientation() {
        return targetOrientation;
    }

    /**
     * attaches attributes to physical inputs and outputs, and makes sure all the motors/servos are in the correct position
     */
    public void initializeRobot() {

        //attaching the individual motors of drivetrain
        drivetrain.lfMotor = opMode.hardwareMap.get(DcMotor.class, "lf_motor");
        drivetrain.rfMotor = opMode.hardwareMap.get(DcMotor.class, "rf_motor");
        drivetrain.lbMotor = opMode.hardwareMap.get(DcMotor.class, "lb_motor");
        drivetrain.rbMotor = opMode.hardwareMap.get(DcMotor.class, "rb_motor");

        //configuring motors so they move in the right direction and set their zero power behavior correctly
        drivetrain.lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.rfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrain.lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrain.lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drivetrain.lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //attaching imu to variable and getting the gyroscope set up
        imu = opMode.hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        //getting the initial angle on the robot
        angleOffset = 0.5 * Math.PI;

        //attaching arm/launcher motors and servos
        arm.shoulder = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.wrist = opMode.hardwareMap.get(Servo.class,"wrist");
        arm.claw = opMode.hardwareMap.get(CRServo.class,"claw");
        launcher = opMode.hardwareMap.get(Servo.class,"launcher");
        dropper = opMode.hardwareMap.get(CRServo.class,"dropper");

        //configuring the arm so it is in the right mode at the right power
        arm.shoulder.setTargetPosition(0);
        arm.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.shoulder .setPower(1);

        //setting up the servos to be in the right position
        arm.wrist.setPosition(0.6);
        arm.claw.setPower(0);
        launcher.setPosition(1);
        dropper.setPower(0);

        //attaching color sensor
        colorSensor = opMode.hardwareMap.get(ColorSensor.class,"color_sensor");

        lastLFPosition = drivetrain.lfMotor.getCurrentPosition();
        lastRFPosition = drivetrain.rfMotor.getCurrentPosition();
        lastRBPosition = drivetrain.rbMotor.getCurrentPosition();
        lastLBPosition = drivetrain.lbMotor.getCurrentPosition();

    } //end method initialize()

    /**
     * sets the robot's mode
     * @param driveMode mode the robot will be set to
     */
    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
        this.drivetrain.stop();

        switch (driveMode) {
            case AUTO_SCORE:
                autoScoringState = AutoScoringState.ORIENT;
                this.arm.shoulder.setPower(1);
                this.drivetrain.lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case MANUAL_DRIVE:
            case ORIENT:
                //in case the power was changed via emergency brake, set power back on for the shoulder
                this.arm.shoulder.setPower(1);
                this.drivetrain.lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
        }

        this.update();

    } //end method setDriveMode()

    /**
     * returns the robot's current drive mode
     * @return current drive mode of the robot
     */
    public DriveMode getDriveMode() {
        return this.driveMode;
    }

    /**
     * updates the coordinates of the robot
     */
    public void update() {
        lfPosition = drivetrain.lfMotor.getCurrentPosition();
        rfPosition = drivetrain.rfMotor.getCurrentPosition();
        rbPosition = drivetrain.rbMotor.getCurrentPosition();
        lbPosition = drivetrain.lbMotor.getCurrentPosition();
        double angle = this.getBotHeading();
        Vector lfVector = Vector.rotate( Vector.multiply( new Vector(Math.sqrt(2) / 2, -Math.sqrt(2) / 2), lfPosition - lastLFPosition ), angle);
        Vector rfVector = Vector.rotate( Vector.multiply( new Vector(Math.sqrt(2) / 2, Math.sqrt(2) / 2), rfPosition - lastRFPosition ), angle);
        Vector rbVector = Vector.rotate( Vector.multiply( new Vector(Math.sqrt(2) / 2, -Math.sqrt(2) / 2), rbPosition - lastRBPosition ), angle);
        Vector lbVector = Vector.rotate( Vector.multiply( new Vector(Math.sqrt(2) / 2, Math.sqrt(2) / 2), lbPosition - lastLBPosition ), angle);
        Vector totalVector = Vector.add( Vector.add(lfVector,rfVector), Vector.add(rbVector,lbVector) );
        position = Vector.add(position,Vector.divide(totalVector,TICKS_PER_INCH));
        lastLFPosition = lfPosition;
        lastRFPosition = rfPosition;
        lastRBPosition = rbPosition;
        lastLBPosition = lbPosition;


        switch(this.driveMode) {

            case ORIENT:
                this.drivetrain.runDrivetrainFromCartesian(new Vector(0,0), TargetTurn.getTurn(this.getBotHeading(),this.targetOrientation),getBotHeading());
                break;

            case EMERGENCY_BRAKE:
                this.drivetrain.lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.drivetrain.stop();
                this.arm.shoulder.setPower(0);
                this.arm.claw.setPower(0);
                this.dropper.setPower(0);
                break;
        }
    } //end method update()

    /**
     * gets the position of the robot
     * @return vector of the robot's offset in ticks
     */
    public Vector getPosition() {
        return position;
    }

}
