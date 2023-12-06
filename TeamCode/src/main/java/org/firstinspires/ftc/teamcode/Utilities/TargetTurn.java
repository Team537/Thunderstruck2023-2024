package org.firstinspires.ftc.teamcode.Utilities;

public class TargetTurn {

    public static double getTurn(double orientation,double target) {
        Vector orientationVector = Vector.fromPolar(1,orientation);
        Vector targetVector = Vector.fromPolar(1,target);
        if (Vector.dot(orientationVector,targetVector) > 0) {
            return Vector.cross(orientationVector, targetVector);
        } else {
            if (Vector.cross(orientationVector, targetVector) > 0) {
                return 1;
            } else {
                return -1;
            }
        }
    }

    public static double getDistance(double orientation,double target) {
        Vector orientationVector = Vector.fromPolar(1,orientation);
        Vector targetVector = Vector.fromPolar(1,target);
        return Vector.cross(orientationVector, targetVector);
    }

}
