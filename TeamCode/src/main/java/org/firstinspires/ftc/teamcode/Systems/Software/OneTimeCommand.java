package org.firstinspires.ftc.teamcode.Systems.Software;

public class OneTimeCommand extends CommandBase {

    //the runnable to be run exactly once upon initialization
    Runnable runnable;

    public OneTimeCommand(Runnable runnable) {
        this.runnable = runnable;
    }

    public void initialize() {
        runnable.run();
    }

    public boolean isFinished() {
        return true;
    }

}
