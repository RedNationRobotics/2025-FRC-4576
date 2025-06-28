package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Libaries.LimelightHelpers;

public class LimeLIghtCommands {
    public static InstantCommand estimatePoseFromLimelight = new InstantCommand(
        () -> {
            System.out.println("AAAG");
            doEstimatePoseByLimelight();
        }
    );

    public static void doEstimatePoseByLimelight(){
        System.out.println("Getting pose?");
        //if limelight not seeing target then return
        System.out.println(LimelightHelpers.getTX("limelight")  );
        SmartDashboard.putBoolean("See target?", LimelightHelpers.getTV("limelight"));
        if (LimelightHelpers.getTV("limelight") == false) return;
        System.out.println(DriverStation.getAlliance());

        var alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            System.out.println("no aliance");
            return;
        }

        if(alliance.get() == DriverStation.Alliance.Red){
            Constants.odometry.setFullPosition(LimelightHelpers.getBotPoseEstimate_wpiRed("limelight").pose);
            SmartDashboard.putString("Aliance", "Red");
            SmartDashboard.putNumber("limelight says roations", LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose.getRotation().getDegrees());
        } else {
            Constants.odometry.setFullPosition(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose);
            SmartDashboard.putString("Aliance", "Blue");
            SmartDashboard.putNumber("limelight says roations", LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose.getRotation().getDegrees());
        }
    }

}
