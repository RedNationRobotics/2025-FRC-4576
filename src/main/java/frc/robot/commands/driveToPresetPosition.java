package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.trackSubsytem;

public class driveToPresetPosition extends Command {

    private final trackSubsytem Elevator;
    public driveToPresetPosition(trackSubsytem Elevator){
        this.Elevator = Elevator;
    }

    @Override
    public void execute(){
        Elevator.driveToDesiredPosition();
    }

    @Override
    public void end(boolean interrupted){
        Elevator.drive(0);
    }

    @Override
    public boolean isFinished(){
        return Elevator.isAtPosition();
    }

}
