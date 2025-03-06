package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.trackSubsytem;

public class ElevatorCommands {
    static trackSubsytem Elevator;

    public static Command getMoveCommand(Double pos){
        return new InstantCommand(
            () -> Elevator.setDesiredPosition(pos)
            , Elevator
        );
    }

    public static Command pos1 = getMoveCommand(0d);
    public static Command pos2 = getMoveCommand(0d);
    public static Command pos3 = getMoveCommand(0d);
    public static Command pos4 = getMoveCommand(0d);

    public static Command elevatorUp=
        new InstantCommand(
            () -> Elevator.driveAbsoluteSpeed(1)
            , Elevator
        );

    public static Command elevatorDown=
        new InstantCommand(
            () -> Elevator.driveAbsoluteSpeed(-1)
            , Elevator
        );

    public static Command elevatorToOrigin= 
        new RunCommand(
            () -> Elevator.drive(0)
            , Elevator
        );

    public static Command driveToOrigin = new driveToOrigin(Elevator);
    public static Command driveToPresetPosition = new driveToPresetPosition(Elevator);
}   
