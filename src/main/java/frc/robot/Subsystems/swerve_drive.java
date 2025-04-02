package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.LimeLIghtCommands;
import frc.robot.Libaries.LimelightHelpers;

public class swerve_drive extends SubsystemBase{
    private boolean doFieldRelativeDrive = true;
    private boolean braked = false;


    public swerve_drive(){
        
    }

    public void drive(double XSpeed, double ZSpeed, double rotationAmt){
      if (braked) return;
      if (doFieldRelativeDrive){
        fieldDrive(XSpeed, ZSpeed, rotationAmt);
      } else {
        absoluteDrive(XSpeed, ZSpeed, rotationAmt);
      }
    }

    public void flipDrive(){
      doFieldRelativeDrive = ! doFieldRelativeDrive;
    }

    public void fieldDrive(double XSpeed, double ZSpeed, double rotationAmt){
        double targetXSpeed;
        double targetZSpeed;
        double targetrotSpeed;

        targetXSpeed = XSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetZSpeed = ZSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetrotSpeed = rotationAmt * Constants.SWERVE_MOTORS.maxAngularSpeed;

        //targetXSpeed = Constants.SWERVE_MOTORS.accLimiter.calculate(targetXSpeed);
        //targetZSpeed = Constants.SWERVE_MOTORS.accLimiter.calculate(targetZSpeed);
        //targetrotSpeed = Constants.SWERVE_MOTORS.accLimiter.calculate(targetrotSpeed);

        SmartDashboard.putNumber("desired speed percent", XSpeed);
        ChassisSpeeds SwerveMotorDesiredStates;
        SwerveMotorDesiredStates = ChassisSpeeds.fromFieldRelativeSpeeds(-targetXSpeed, -targetZSpeed, -targetrotSpeed, Constants.gyro.main_Gyro.getRotation2d());

        SwerveModuleState[] desiredStates = Constants.SWERVE_MOTORS.swerveKinematics.toSwerveModuleStates(SwerveMotorDesiredStates);

        setSpeeds(desiredStates);
    }

    public void absoluteDrive(double XSpeed, double ZSpeed, double rotationAmt){
        double targetXSpeed;
        double targetZSpeed;
        double targetrotSpeed;

        targetXSpeed = XSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetZSpeed = ZSpeed * Constants.SWERVE_MOTORS.maxSpeedMPS;
        targetrotSpeed = rotationAmt * Constants.SWERVE_MOTORS.maxAngularSpeed;

        //targetXSpeed = Constants.SWERVE_MOTORS.accLimiter.calculate(targetXSpeed);
        //targetZSpeed = Constants.SWERVE_MOTORS.accLimiter.calculate(targetZSpeed);
        //targetrotSpeed = Constants.SWERVE_MOTORS.accLimiter.calculate(targetrotSpeed);

        ChassisSpeeds SwerveMotorDesiredStates;
        SwerveMotorDesiredStates = ChassisSpeeds.fromRobotRelativeSpeeds(-targetXSpeed, -targetZSpeed, -targetrotSpeed, Rotation2d.kZero);

        SwerveModuleState[] desiredStates = Constants.SWERVE_MOTORS.swerveKinematics.toSwerveModuleStates(SwerveMotorDesiredStates);

        setSpeeds(desiredStates);
    }

    public void autoDrive(ChassisSpeeds robotSpeeds, DriveFeedforwards optional){
      //robotSpeeds.vxMetersPerSecond *=-1;  
      //robotSpeeds.vyMetersPerSecond *=-1;  
      //robotSpeeds.omegaRadiansPerSecond *=-1;  
      robotSpeeds.omegaRadiansPerSecond *=0;  
      SwerveModuleState[] desiredStates = Constants.SWERVE_MOTORS.swerveKinematics.toSwerveModuleStates(robotSpeeds);
      setSpeeds(desiredStates);
    }
    private final Field2d m_field = new Field2d();
        
    @Override
    public void periodic() {
        super.periodic();
        Constants.odometry.updatePosition();

        SmartDashboard.putNumber("PoseX", Constants.odometry.getPosition().getX());
        SmartDashboard.putNumber("PoseY", Constants.odometry.getPosition().getY());
        SmartDashboard.putString("Rotation", Constants.gyro.main_Gyro.getRotation2d().toString());
        SmartDashboard.putNumber("LimelightID", LimelightHelpers.getFiducialID("limelight"));
        SmartDashboard.putString("PATHPLANNER POSE", AutoBuilder.getCurrentPose().toString());

        // Do this in either robot periodic or subsystem periodic

        m_field.setRobotPose(Constants.odometry.getPosition());
        SmartDashboard.putData("Field", m_field);
        LimeLIghtCommands.doEstimatePoseByLimelight();
    }

    public void brake() {
      Constants.SWERVE_MOTORS.Swerve_BL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(1.75*Math.PI)));
      Constants.SWERVE_MOTORS.Swerve_BR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(.25*Math.PI)));
      Constants.SWERVE_MOTORS.Swerve_FL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(.25*Math.PI)));
      Constants.SWERVE_MOTORS.Swerve_FR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-.25*Math.PI)));
      braked = true;
    }

    public void unbrake(){
      braked = false;
    }

    public void setSpeeds(SwerveModuleState[] states){
      Constants.SWERVE_MOTORS.Swerve_BL.setDesiredState(states[0]);
        Constants.SWERVE_MOTORS.Swerve_FL.setDesiredState(states[1]);
        Constants.SWERVE_MOTORS.Swerve_BR.setDesiredState(states[2]);
        Constants.SWERVE_MOTORS.Swerve_FR.setDesiredState(states[3]);
    }
}