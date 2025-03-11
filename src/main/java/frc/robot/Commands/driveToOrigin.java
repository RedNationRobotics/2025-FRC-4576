package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.trackSubsytem;

public class driveToOrigin extends Command {

    private final trackSubsytem Elevator;
    public driveToOrigin(trackSubsytem Elevator){
        this.Elevator = Elevator;
    }

    @Override
    public void execute(){
        Elevator.driveAbsoluteSpeed(-1);;
    }

    @Override
    public void end(boolean interrupted){
        Elevator.drive(0);
    }

    @Override
    public boolean isFinished(){
        return Elevator.isAtOrigin();
    }

}