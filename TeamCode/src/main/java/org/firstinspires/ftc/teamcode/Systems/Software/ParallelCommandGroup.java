package org.firstinspires.ftc.teamcode.Systems.Software;

import java.util.ArrayList;
import java.util.Arrays;

public class ParallelCommandGroup extends CommandBase {

    ArrayList<CommandBase> commandList;

    public ParallelCommandGroup(CommandBase... commandList) {
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

        //removing all finished commands
        for (CommandBase command : commandList) {
            if (command.isFinished()) {
                command.onFinished();
                commandList.remove(command);
            }
        }

        //if all commands are removed/finished, the group will return as finished
        if (commandList.size() == 0) {
            return true;
        } else {
            return false;
        }

    }

}
