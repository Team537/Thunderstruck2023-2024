package org.firstinspires.ftc.teamcode.Systems.Software;


public class RunCommand extends CommandBase {

    //the runnable which will run on the commands execution
    Runnable runnable;

    public RunCommand(Runnable runnable) {
        this.runnable = runnable;
    }

    public void execute() {
        runnable.run();
    }

}
