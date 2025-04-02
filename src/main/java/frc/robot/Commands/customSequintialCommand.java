package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class customSequintialCommand extends Command {
    Command currentCommand;
    int commandIndex;
    Command[] commands;

    public customSequintialCommand(Command[] commands){
        this.commands = commands;
        commandIndex = 0;
        currentCommand = commands[0];
    }

    @Override
    public void execute() {
        currentCommand.execute();
        if (currentCommand.isFinished()){
            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommand.isFinished() & commandIndex == commands.length ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            currentCommand.end(interrupted);
            return;
        }
        currentCommand.end(interrupted);
        if (commandIndex < commands.length-1){
            commandIndex++;
            currentCommand = commands[commandIndex];
        }
    }
}
