package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    public double kp;
    public double ki;
    public double kd;

    public double feedForwardRatio;

    public double lastError = 0;
    public double lastTime = 0;

    public double integral;
    public double derivative;
    public double out;

    double target;

    public ElapsedTime runtime = new ElapsedTime();

    public PID(double kp, double ki, double kd, double feedForwardRatio) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.feedForwardRatio = feedForwardRatio;
    }

    public void setTarget(double target, double error) {
        this.target = target;
        this.lastTime = runtime.seconds();
        this.lastError = error;
        integral = 0;
    }

    public double calculate(double error, double feedForward) {

        double time = runtime.seconds();

        derivative = (error - lastError) / (time - lastTime);

        integral = integral + (error * (time - lastTime));

        out = (kp * error) + (ki * integral) + (kd * derivative);

        lastError = error;
        lastTime = time;

        return (feedForwardRatio * feedForward) + ((1 - feedForwardRatio) * Math.tanh(out));

    }

    public double getTarget() {
        return target;
    }

}
