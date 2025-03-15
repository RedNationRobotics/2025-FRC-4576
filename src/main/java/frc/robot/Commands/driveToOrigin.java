package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.trackSubsytem;

public class driveToOrigin extends Command {

    private final trackSubsytem Elevator;
    public driveToOrigin(trackSubsytem Elevator){
        this.Elevator = Elevator;
        addRequirements(Elevator);
    }

    @Override
    public void execute(){
        Elevator.setDesiredPosition(0);
        Elevator.driveAbsoluteSpeed(-.3);
        if(isFinished()) end(false);
    }

    @Override
    public void end(boolean interrupted){
        Elevator.stop();
    }

    @Override
    public boolean isFinished(){
        return Elevator.isAtOrigin();
    }

}