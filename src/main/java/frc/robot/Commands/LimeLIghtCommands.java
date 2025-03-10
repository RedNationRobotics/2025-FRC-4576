package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Libaries.LimelightHelpers;

public class LimeLIghtCommands {
    public static InstantCommand estimatePoseFromLimelight = new InstantCommand(
        () -> {
            Constants.odometry.setFullPosition(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
            SmartDashboard.putNumber("limelight says roations", LimelightHelpers.getBotPose2d_wpiBlue("limelight").getRotation().getDegrees());
        }
    );
}
