package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.trackSubsytem;

public class driveToPresetPosition extends Command {

    private final trackSubsytem Elevator;
    public driveToPresetPosition(trackSubsytem Elevator){
        this.Elevator = Elevator;
        this.addRequirements(Elevator);
    }

    @Override
    public void execute(){
        Elevator.driveToDesiredPosition();
        if (isFinished()) end(false);
    }

    @Override
    public void end(boolean interrupted){
        Elevator.stop();
    }

    @Override
    public boolean isFinished(){
        return Elevator.isAtPresetPosition();
    }

}
