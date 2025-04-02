package frc.robot.Commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Libaries.LimelightHelpers;

public class LimeLIghtCommands {
    static Map<Integer, Pose2d> leftPoses = new HashMap<Integer, Pose2d>();
    static Map<Integer, Pose2d> rightPoses = new HashMap<Integer, Pose2d>();

    public static boolean doPositioning = true;

    public static void setupDictionary(){
        
        leftPoses.put(17, new Pose2d(3.7,2.96,Rotation2d.fromDegrees(62)));
        leftPoses.put(18, new Pose2d(3.3,4.18,Rotation2d.fromDegrees(-1.04)));
        rightPoses.put(17, new Pose2d(3.97,2.85,Rotation2d.fromDegrees(55)));
        rightPoses.put(18, new Pose2d(3.23,3.83,Rotation2d.fromDegrees(-1.8)));
    }

    public static InstantCommand estimatePoseFromLimelight = new InstantCommand(
        () -> {
            doEstimatePoseByLimelight();
        }
    );

    public static void doEstimatePoseByLimelight(){
        if (!doPositioning) return;
        LimelightHelpers.setPipelineIndex("limelight", 0);
        //if limelight not seeing target then return
        SmartDashboard.putBoolean("See target?", LimelightHelpers.getTV("limelight"));
        if (LimelightHelpers.getTV("limelight") == false) return;

        var alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return;
        }

        if(alliance.get() == DriverStation.Alliance.Red){
            Constants.odometry.setFullPosition(LimelightHelpers.getBotPoseEstimate_wpiRed("limelight").pose);
        } else {
            Constants.odometry.setFullPosition(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose);
        }

        Constants.subsystems.left.setTarget(leftPoses.get((int)LimelightHelpers.getFiducialID("limelight")));
        Constants.subsystems.right.setTarget(rightPoses.get((int)LimelightHelpers.getFiducialID("limelight")));
    }

}
