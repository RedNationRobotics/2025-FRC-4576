package frc.robot.Commands;

import java.util.regex.MatchResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class SmallDriveAdjustment  extends Command {

    public Pose2d targetPosition;
    private final double maxSpeedMultiplier = .1;
    private final double maxTurnMultiplier = .1;

    public SmallDriveAdjustment(Pose2d targetPosition){
        this.targetPosition = targetPosition;
    }

    public SmallDriveAdjustment(){
        this.targetPosition = null;
    }

    @Override
    public void execute(){
        if (targetPosition == null){
            end(false);
            return;
        }
        double diffX = targetPosition.getX() - Constants.odometry.getPosition().getX();
        double diffY = targetPosition.getY() - Constants.odometry.getPosition().getY();
        double diffrot = targetPosition.getRotation().minus(Constants.odometry.getPosition().getRotation()).getRadians();
        
        diffX = MathUtil.applyDeadband(diffX, .03);
        diffY = MathUtil.applyDeadband(diffY, .03);
        diffrot = MathUtil.applyDeadband(diffrot, Units.degreesToRadians(.8));

        diffX /= Constants.SWERVE_MOTORS.maxSpeedMPS;
        diffY /= Constants.SWERVE_MOTORS.maxSpeedMPS;
        diffrot /= Constants.SWERVE_MOTORS.maxAngularSpeed;
        
        diffX = Math.max(Math.abs(diffX), .01) * Math.signum(diffX);
        diffY = Math.max(Math.abs(diffY), .01) * Math.signum(diffY);
        diffrot = Math.max(Math.abs(diffrot), .1) * Math.signum(diffrot);
        
        diffX = MathUtil.clamp(diffX, -1, 1);
        diffY = MathUtil.clamp(diffY, -1, 1);
        diffrot = MathUtil.clamp(diffrot, -1, 1);

        Constants.subsystems.robotDrive.fieldDrive(-diffX, -diffY, -diffrot);
    }

    @Override
    public void end(boolean interrupted){
        Constants.subsystems.robotDrive.absoluteDrive(0, 0, 0);
    }

    public void setTarget(Pose2d pose){
        targetPosition = pose;
    }

    @Override
    public boolean isFinished(){
        if (targetPosition == null) return true;
        double diffX = targetPosition.getX() - Constants.odometry.getPosition().getX();
        double diffY = targetPosition.getY() - Constants.odometry.getPosition().getY();
        double diffrot = targetPosition.getRotation().minus(Constants.odometry.getPosition().getRotation()).getDegrees();

        return 
        Math.abs(diffX) < .03 &&
        Math.abs(diffY) < .03 &&
        Math.abs(diffrot) < 1;
    }
}