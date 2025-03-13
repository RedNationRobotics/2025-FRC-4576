package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.trackSubsytem;

public class ElevatorCommands {
    private trackSubsytem Elevator;

    public ElevatorCommands(trackSubsytem Elevator) {
        this.Elevator = Elevator;

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
            () -> {Elevator.stop();}
            , Elevator
        );

        driveToOrigin = new driveToOrigin(Elevator);
        driveToPresetPosition = new driveToPresetPosition(Elevator);
    }

    public InstantCommand getSetPosCommand(Double pos){
        return new InstantCommand(
            () -> {
                Elevator.setDesiredPosition(pos);
            }
            , Elevator
        );
    }

    public InstantCommand elevatorUp;
    public InstantCommand elevatorDown;
    public RunCommand elevatorStop;
    public Command driveToOrigin;
    public Command driveToPresetPosition;

}   
