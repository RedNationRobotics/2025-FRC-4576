package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.trackSubsytem;

public class ElevatorCommands {
    trackSubsytem Elevator;

    public ElevatorCommands(trackSubsytem Elevator) {
        this.Elevator = Elevator;
    }

    public Command getMoveCommand(Double pos){
        return new InstantCommand(
            () -> Elevator.setDesiredPosition(pos)
            , Elevator
        );
    }

    public Command pos1 = getMoveCommand(0d);
    public Command pos2 = getMoveCommand(0d);
    public Command pos3 = getMoveCommand(0d);
    public Command pos4 = getMoveCommand(0d);

    public Command elevatorUp=
        new InstantCommand(
            () -> Elevator.driveAbsoluteSpeed(1)
            , Elevator
        );

    public Command elevatorDown=
        new InstantCommand(
            () -> Elevator.driveAbsoluteSpeed(-1)
            , Elevator
        );

    public Command elevatorToOrigin= 
        new RunCommand(
            () -> Elevator.drive(0)
            , Elevator
        );

    public Command driveToOrigin = new driveToOrigin(Elevator);
    public Command driveToPresetPosition = new driveToPresetPosition(Elevator);
}   
