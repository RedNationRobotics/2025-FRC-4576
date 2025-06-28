package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.trackSubsytem;

public class ElevatorToPosition extends Command {

    private final trackSubsytem Elevator;
    private final double targetPos;
    public ElevatorToPosition(trackSubsytem Elevator, double pos){
        this.Elevator = Elevator;
        this.addRequirements(Elevator);
        this.targetPos = pos;
    }

    @Override
    public void execute(){
        //Elevator.setDesiredPosition(targetPos);
        //Elevator.driveToDesiredPosition();
        Elevator.driveToPosition(targetPos);
        if (isFinished()) end(false);
    }

    @Override
    public void end(boolean interrupted){
        Elevator.stop();
    }

    @Override
    public boolean isFinished(){
        return Elevator.isAtPosition(targetPos);
    }

}
