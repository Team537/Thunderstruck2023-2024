package org.firstinspires.ftc.teamcode.Systems.Software;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelRaceCommandGroup extends CommandBase {

    boolean raceEnded = false;

    ArrayList<CommandBase> commandList;

    public ParallelRaceCommandGroup(CommandBase... commandList) {
        this.commandList = new ArrayList<CommandBase>(Arrays.asList(commandList));
    }

    public void initialize() {

        //initializing all commands
        for (CommandBase command : commandList) {
            command.initialize();
        }
    }

    public void execute() {

        //executing all commands
        for (CommandBase command : commandList) {
            command.execute();
        }
    }

    public boolean isFinished() {

        //search for a command that has finished
        if (raceEnded == false) {
            for (CommandBase command : commandList) {
                if (command.isFinished()) {
                    raceEnded = true;
                    break;
                }
            }

            //if the race has now ended, run the finished on all commands
            if (raceEnded) {
                for (CommandBase command : commandList) {
                    command.onFinished();
                }
            }
        }

        //if the race has finished, return true; otherwise, return false
        return raceEnded;

    }

}
