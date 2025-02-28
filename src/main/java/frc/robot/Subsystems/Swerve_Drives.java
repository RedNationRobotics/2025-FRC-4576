package frc.robot.Subsystems;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve_Drives extends SubsystemBase{
    public Swerve_Drives(){

    }

    ///
    /// 
    /// 
    /// Speeds are from 0 to 1
    public void drive(double XSpeed, double ZSpeed, double rotationAmt){
        double targetXSpeed;
        double targetZSpeed;
        double targetrotSpeed;

        targetXSpeed = XSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetZSpeed = ZSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetrotSpeed = rotationAmt * Constants.SWERVE_MOTORS.maxAngularSpeed;

        ChassisSpeeds swerveMotorDesiredStates;
        swerveMotorDesiredStates = ChassisSpeeds.fromFieldRelativeSpeeds(targetXSpeed, targetZSpeed, rotationAmt, Constants.gyro.main_Gyro.getRotation2d());

        SwerveModuleState[] desiredStates = Constants.SWERVE_MOTORS.swerveKinematics.toSwerveModuleStates(swerveMotorDesiredStates);

    Constants.SWERVE_MOTORS.Swerve_BL.setDesiredState(desiredStates[0]);
    Constants.SWERVE_MOTORS.Swerve_BR.setDesiredState(desiredStates[1]);
    Constants.SWERVE_MOTORS.Swerve_FL.setDesiredState(desiredStates[2]);        
    Constants.SWERVE_MOTORS.Swerve_FR.setDesiredState(desiredStates[3]);

// Constants.SWERVE_MOTORS.Swerve_BL.setDesiredState()
    }
}