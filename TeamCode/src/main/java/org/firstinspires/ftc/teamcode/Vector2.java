package org.firstinspires.ftc.teamcode;

public class Vector2 {
    double X;
    double Y;

    public Vector2 (double x, double y) {
        this.X = x;
        this.Y = y;
    }

    static Vector2 add(Vector2 a,Vector2 b) {
        return new Vector2(a.X + b.X,a.Y + b.Y);
    }


}
