package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Libaries.LimelightHelpers;

public class Limelight_LineupRight_CMD extends Command{

    

    public Limelight_LineupRight_CMD(){
        addRequirements(Constants.subsystems.robotDrive);
    }
    
    @Override
    public void execute() {
        LimelightHelpers.setPipelineIndex("limelight", 1);
        if (LimelightHelpers.getTX("limelight") == 0) 
        {
            Constants.subsystems.robotDrive.absoluteDrive(0,.1, 0);
            return;
        }
        double rot = 0;
        double drive = 0;
        if (LimelightHelpers.getTX("limelight") > 1 ){
            drive = 1;
            rot = Math.signum(LimelightHelpers.getTX("limelight"));
        } else if (LimelightHelpers.getTX("limelight") < -1 ){
            rot = Math.signum(LimelightHelpers.getTX("limelight"));
            drive = -1;
        }
        Constants.subsystems.robotDrive.absoluteDrive(0,drive *.1, rot*.1);
    }
}
