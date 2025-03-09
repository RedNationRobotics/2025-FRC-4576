package frc.robot.Subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class swerve_drive extends SubsystemBase{
    public swerve_drive(){

    }

    public void drive(double XSpeed, double ZSpeed, double rotationAmt){
        double targetXSpeed;
        double targetZSpeed;
        double targetrotSpeed;

        targetXSpeed = XSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetZSpeed = ZSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetrotSpeed = -rotationAmt * Constants.SWERVE_MOTORS.maxAngularSpeed;


        ChassisSpeeds SwerveMotorDesiredStates;
        SwerveMotorDesiredStates = ChassisSpeeds.fromFieldRelativeSpeeds(targetXSpeed, targetZSpeed, targetrotSpeed, Constants.gyro.main_Gyro.getRotation2d());
        //SwerveMotorDesiredStates = ChassisSpeeds.fromRobotRelativeSpeeds(targetXSpeed, targetZSpeed, targetrotSpeed, Constants.gyro.main_Gyro.getRotation2d());


        SwerveModuleState[] desiredStates = Constants.SWERVE_MOTORS.swerveKinematics.toSwerveModuleStates(SwerveMotorDesiredStates);

        Constants.SWERVE_MOTORS.Swerve_BL.setDesiredState(desiredStates[0]);
        Constants.SWERVE_MOTORS.Swerve_FL.setDesiredState(desiredStates[1]);
        Constants.SWERVE_MOTORS.Swerve_BR.setDesiredState(desiredStates[2]);
        Constants.SWERVE_MOTORS.Swerve_FR.setDesiredState(desiredStates[3]);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        Constants.odometry.updatePosition();

        SmartDashboard.putNumber("PoseX", Constants.odometry.getPosition().getX());
        SmartDashboard.putNumber("PoseY", Constants.odometry.getPosition().getY());
        SmartDashboard.putNumber("Rotation", Constants.gyro.main_Gyro.getRotation2d().getRadians());
    }
}