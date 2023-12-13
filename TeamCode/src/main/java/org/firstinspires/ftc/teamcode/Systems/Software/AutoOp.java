package org.firstinspires.ftc.teamcode.Systems.Software;

import org.firstinspires.ftc.teamcode.Systems.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.AutoScoringState;
import org.firstinspires.ftc.teamcode.Utilities.TargetTurn;
import org.firstinspires.ftc.teamcode.Utilities.Vector;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AutoOp {

    public Robot robot;
    public AutoScoringState autoScoringState = AutoScoringState.BEGIN;

    boolean foundTag = false;
    boolean foundPrimary = false;
    Vector tagOffset = new Vector(0,0);
    int tagID = 0;
    int primaryTagID = 0;

    private double armUpTimestamp = 0;
    private double positionTimestamp = 0;
    private double dischargeTimestamp = 0;
    private double armDownTimestamp = 0;
    private double lastTagFoundTimestamp = 0;

    private double maxTagWaitTime = 0;

    public AutoOp(Robot robot) {
        this.robot = robot;
    }

    //sets up scoring and primary tags while spinning to the correct orientation before aligning
    private void firstOrient() {

        //find the primary tag id based on the scoring state and alliance
        switch (robot.scoringPosition) {
            case LEFT:
                primaryTagID = (robot.alliance == Alliance.BLUE) ? 1 : 4;
                break;
            case MIDDLE:
                primaryTagID = (robot.alliance == Alliance.BLUE) ? 2 : 5;
                break;
            case RIGHT:
                primaryTagID = (robot.alliance == Alliance.BLUE) ? 3 : 6;
                break;
        }

        //setting the arm up and marking the timestamp
        robot.arm.armUp();
        armUpTimestamp = robot.runtime.seconds();

    }
    private boolean orient(double turn) {
        robot.drivetrain.runDrivetrainFromCartesian(new Vector(0,0),turn,1.5 * Math.PI);
        if ( Math.abs( turn ) < 0.05) {
            robot.drivetrain.stop();
            if (robot.arm.shoulder.getCurrentPosition() < -200) {
                return true;
            }
        }
        return false;
    }

    //aligns robot with tag
    private boolean align(double turn) {

        findTag();
        Vector linear = new Vector(0,0);

        robot.opMode.telemetry.addData("id",tagID);
        robot.opMode.telemetry.addData("tag offset",tagOffset.string());
        robot.opMode.telemetry.addData("found tag",foundTag);

        if (foundTag) {

            lastTagFoundTimestamp = robot.runtime.seconds();

            //drives slower and closer if its found the target tag, else it drives farther away and faster
            if (foundPrimary) {

                //finishes state if it detects the right tag in the right spot
                if (Math.abs(tagOffset.x) < 0.3 && tagOffset.y < 10) {
                    robot.drivetrain.stop();
                    return true;
                }

                //changes wait times depending on if it has aligned itself correctly
                if (Math.abs(tagOffset.x) < 0.3) {
                    maxTagWaitTime = Math.min(0.3, Math.min(Math.abs(tagOffset.x)/30,tagOffset.y/30));
                } else {
                    maxTagWaitTime = Math.min(0.3, tagOffset.y/10);
                }

                //drives closer to tag
                if (tagOffset.y < 5) {
                    linear.y = 0;
                } else {
                    linear.y = 0.3; //0.3
                }

                //aligns left or right
                if (tagOffset.x > 0.3) {
                    linear.x = 0.15; //0.15
                } else if (tagOffset.x < -0.3) {
                    linear.x = -0.15; //-0.15
                } else {
                    linear.x = 0;
                }

            } else {

                maxTagWaitTime = 0.4;

                //drives backwards for more visibility
                if (tagOffset.y > 40) {
                    linear.y = 0;
                } else {
                    linear.y = -0.15; //-0.15
                }

                //drives left or right depending on if the id is less than or greater than the primary one
                if (tagID > primaryTagID) {
                    linear.x = -0.4; //0.3
                } else {
                    linear.x = 0.4; //0.3
                }

            }

        }

        //stops drivetrain if it hasn't detected a tag in a while
        if (robot.runtime.seconds() > lastTagFoundTimestamp + maxTagWaitTime) {
            if (robot.runtime.seconds() > lastTagFoundTimestamp + maxTagWaitTime + 1) {
                linear = new Vector(0,-0.2);
            } else {
                linear = new Vector(0,0);
            }
        }

        //updates drivetrain
        robot.drivetrain.runDrivetrainFromCartesian(linear,turn,1.5 * Math.PI);

        return false;
    }

    //drives the robot a little closer
    private void firstPosition() {
        positionTimestamp = robot.runtime.seconds();
    }
    private boolean position(double turn) {
        robot.drivetrain.runDrivetrainFromCartesian(new Vector(0,0.2),turn,1.5 * Math.PI);
        if (robot.runtime.seconds() > positionTimestamp + (tagOffset.y / 6)) {
            robot.drivetrain.stop();
            return true;
        }
        return false;
    }

    //discharges pixel
    private void firstDischarge() {
        robot.arm.discharge();
        dischargeTimestamp = robot.runtime.seconds();
    }
    private boolean discharge() {
        if (robot.runtime.seconds() > dischargeTimestamp + 2) {
            robot.arm.stopClaw();
            return true;
        }
        return false;
    }

    //drops the arm down
    private void firstFall() {
        robot.arm.armDown();
        armDownTimestamp = robot.runtime.seconds();
    }
    private boolean fall() {
        if (robot.runtime.seconds() > armDownTimestamp + 3) {
            return true;
        }
        return false;
    }

    //detects an april tag (prioritizing the primary april tag)
    private void findTag() {
        List<AprilTagDetection> tags = robot.tagProcessor.getDetections();
        foundTag = false;
        foundPrimary = false;
        for (AprilTagDetection tag : tags) {

            //detects any tag
            boolean useTag = false;
            if (!foundPrimary) {
                if (robot.alliance == Alliance.BLUE) {
                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        useTag = true;
                    }
                } else {
                    if (tag.id == 4 || tag.id == 5 || tag.id == 6) {
                        useTag = true;
                    }
                }
            }

            //updates tag variables and sets any primary tag
            if (useTag) {
                if (tag.id == primaryTagID) {
                    foundPrimary = true;
                }
                foundTag = true;
                tagOffset = new Vector(tag.ftcPose.x,tag.ftcPose.y);
                tagID = tag.id;
            }

        }
    }

    //returns the turning value the robot needs to get to
    private double getTurn() {
        return TargetTurn.getTurn(robot.getBotHeading(),(robot.alliance == Alliance.BLUE) ? 0 : Math.PI);
    }

    //resets auto scoring
    public void resetAutoScore() {
        autoScoringState = AutoScoringState.BEGIN;
    }

    //automatically scores a pre-loaded pixel
    public void autoScore() {

        robot.opMode.telemetry.addData("Auto Scoring State:",autoScoringState);

        switch (autoScoringState) {

            //this state is initializes the auto scoring
            case BEGIN:
                firstOrient();
                autoScoringState = AutoScoringState.ORIENT;
                break;

            //orients the robot so it is facing the right direction
            case ORIENT:

                if (orient(getTurn())) {
                    autoScoringState = AutoScoringState.ALIGN;
                }
                break;

            //drives linearly so that it is a certain distance away from the target april tag
            case ALIGN:

                if (align(getTurn())) {
                    firstPosition();
                    autoScoringState = AutoScoringState.POSITION;
                }
                break;

            //slowly drives closer to the tag and waits for the arm to go completely up
            case POSITION:

                if (position(getTurn())) {
                    firstDischarge();
                    autoScoringState = AutoScoringState.DISCHARGE;
                }
                break;

            //discharges pixel
            case DISCHARGE:

                if (discharge()) {
                    firstFall();
                    autoScoringState = AutoScoringState.FALL;
                }
                break;

            //drops arm
            case FALL:

                if (fall()) {
                    autoScoringState = AutoScoringState.END;
                }
                break;
        }

    }



}