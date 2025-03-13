package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Libaries.LimelightHelpers;

public class Limelight_LineupLeft_CMD extends Command{

    

    public Limelight_LineupLeft_CMD(){
        addRequirements(Constants.subsystems.robotDrive);
    }
    
    @Override
    public void execute() {
        if (LimelightHelpers.getTX("limelight") == 0) return;
        LimelightHelpers.setFiducial3DOffset("limelight", 0, -.2, 0);
        Double rot = 0d;
        if (LimelightHelpers.getTX("limelight") > 0 ){
            rot = LimelightHelpers.getTX("limelight")/(23*5);
        } else if (LimelightHelpers.getTX("limelight") < 0 ){
            rot = LimelightHelpers.getTX("limelight")/(23*5);
        }
        Constants.subsystems.robotDrive.drive(0, 0, rot);
    }
}
