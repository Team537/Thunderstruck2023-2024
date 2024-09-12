package org.firstinspires.ftc.teamcode.Systems.Software;

import java.util.LinkedHashMap;
import java.util.function.Supplier;

public class SelectableCommandGroup extends CommandBase {

    LinkedHashMap<String,CommandBase> commandList;
    LinkedHashMap<String,Supplier<String>> pointerList;
    String pointer;
    String pendingPointer = "";
    boolean pointerNeedsUpdating = false;
    boolean currentCommandHasInitialized = false;

    public SelectableCommandGroup(LinkedHashMap<String,CommandBase> commandList, LinkedHashMap<String,Supplier<String>> pointerList, String initialKey) {
        if (commandList.size() != pointerList.size()) {
            throw new IllegalArgumentException("Command Lists and Pointer List sizes do not match");
        }
        this.commandList = commandList;
        this.pointerList = pointerList;
        this.pointer = initialKey;
    }

    public void initialize() {

        if (commandList.size() != 0) {
            commandList.get(pointer).initialize();
            currentCommandHasInitialized = true;
        }

    }

    public void execute() {

        CommandBase currentCommand = commandList.get(pointer);

        //if the selected command hasn't been initialized, initialize it
        if (!currentCommandHasInitialized) {
            currentCommand.initialize();
            currentCommandHasInitialized = true;
        }
        currentCommand.execute();

    }

    //if the current command finishes, determine the next command by suppliers in the pointer list
    //if the pointer returns [-1], then the command group ends
    //otherwise, reinitialize the new selected command
    public boolean isFinished() {

        CommandBase currentCommand = commandList.get(pointer);
        if (currentCommand.isFinished() == true) {

            //runs the onFinished command once it has finished
            currentCommand.onFinished();

            //the new pointer (defined as separate variable in case a directive is passed via the "&" character)
            String newPointer = pointerList.get(pointer).get();

            //get the index of the current pointer
            Object[] keyArray = commandList.keySet().toArray();
            int pointerIndex = 0;
            for (int i = 0; i < keyArray.length; i++) {
                if (pointer == keyArray[i].toString()) {
                    pointerIndex = i;
                    break;
                }
            }

            //if the new pointer begins with "&", then it will have a special function
            if (newPointer.charAt(0) == '&') {
                String directive = newPointer.substring(1);

                //do the directive based on the text
                switch  (directive.charAt(0)) {

                    //ends the command group
                    case 'e':
                        return true;

                    //jumps to a specific index in the command hashmap
                    case 't':
                        int index = Integer.parseInt(directive.substring(1));
                        pointer = commandList.keySet().toArray()[index].toString();
                        break;

                    //jumps forward one command
                    case 'n':
                        //use the mod function to "loop" back to the start if it goes past the end
                        index = Math.floorMod(pointerIndex + 1,keyArray.length);
                        pointer = commandList.keySet().toArray()[index].toString();
                        break;

                    //jumps forward one command
                    case 'b':
                        //use the mod function to "loop" the code to "loop" to the end if it goes past the start
                        index = pointerIndex - 1;
                        pointer = commandList.keySet().toArray()[index].toString();
                        break;

                    //jumps forward one command
                    case 's':
                        //use the mod function to "loop" around if it skips past the threshold
                        index = Math.floorMod(pointerIndex + Integer.parseInt(directive.substring(1)),keyArray.length);
                        pointer = commandList.keySet().toArray()[index].toString();
                        break;

                    //repeats the current command
                    case 'r':
                        break;

                }
            } else {
                pointer = newPointer;
            }

            currentCommandHasInitialized = false;

        }

        if (pointerNeedsUpdating) {
            pointerNeedsUpdating = false;
            pointer = pendingPointer;
            currentCommandHasInitialized = false;
        }

        return false;

    }

    //sets the pointer (will only update after checking if the current pointer is finished)
    public void setPointer(String pointer) {
        this.pointer = pointer;
        pointerNeedsUpdating = true;
    }

}