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
        LimelightHelpers.setPipelineIndex("limelight", 1);
        if (LimelightHelpers.getTX("limelight") == 0) 
        {
            Constants.subsystems.robotDrive.absoluteDrive(0,-.1, 0);
            return;
        }
        double drive = 0;
        //LimelightHelpers.setFiducial3DOffset("limelight", 0, -.2, 0);
        Double rot = 0d;
        if (LimelightHelpers.getTX("limelight") > 3 ){
            rot = Math.signum(LimelightHelpers.getTX("limelight"));
            drive = -1;
        } else if (LimelightHelpers.getTX("limelight") < -3 ){
            rot = Math.signum(LimelightHelpers.getTX("limelight"));
            drive = 1;
        }

        Constants.subsystems.robotDrive.absoluteDrive(0,drive*.1, rot*.1);
    }
}
