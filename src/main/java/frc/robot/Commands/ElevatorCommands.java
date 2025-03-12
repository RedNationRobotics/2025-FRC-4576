package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.trackSubsytem;

public class ElevatorCommands {
    trackSubsytem Elevator;
    public Command pos1;
    public Command pos2;
    public Command pos3;
    public Command pos4;

    public ElevatorCommands(trackSubsytem Elevator) {
        this.Elevator = Elevator;
        pos1 = getMoveCommand(0d);
        pos2 = getMoveCommand(0d);
        pos3 = getMoveCommand(0d);
        pos4 = getMoveCommand(0d);

        elevatorUp = 
        new InstantCommand(
            () -> {Elevator.driveAbsoluteSpeed(1);}
            , Elevator
        );

        elevatorDown =
        new InstantCommand(
            () -> {Elevator.driveAbsoluteSpeed(-1);}
            , Elevator
        );

        elevatorStop = 
        new RunCommand(
            () -> {Elevator.drive(0);}
            , Elevator
        );

        driveToOrigin = new driveToOrigin(Elevator);
        driveToPresetPosition = new driveToPresetPosition(Elevator);
    }

    public InstantCommand getMoveCommand(Double pos){
        return new InstantCommand(
            () -> {Elevator.setDesiredPosition(pos);}
            , Elevator
        );
    }

    public InstantCommand elevatorUp;

    public InstantCommand elevatorDown;

    public RunCommand elevatorStop;
    
    public Command driveToOrigin;
    public Command driveToPresetPosition;

}   
