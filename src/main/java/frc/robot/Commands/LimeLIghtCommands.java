package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Libaries.LimelightHelpers;

public class LimeLIghtCommands {
    public static InstantCommand estimatePoseFromLimelight = new InstantCommand(
        () -> {
            doEstimatePoseByLimelight();
        }
    );

    public static void doEstimatePoseByLimelight(){
        //if limelight not seeing target then return
        if (LimelightHelpers.getTV("limelight") == false) return;
        
        var alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) return;
        if(alliance.get() == DriverStation.Alliance.Red){
            Constants.odometry.setFullPosition(LimelightHelpers.getBotPose2d_wpiRed("limelight"));
            SmartDashboard.putNumber("limelight says roations", LimelightHelpers.getBotPose2d_wpiRed("limelight").getRotation().getDegrees());
        } else {
            Constants.odometry.setFullPosition(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
            SmartDashboard.putNumber("limelight says roations", LimelightHelpers.getBotPose2d_wpiBlue("limelight").getRotation().getDegrees());
        }
    }

}
