package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Modules.MAXSwerveModule;
import frc.robot.Modules.customEncoder;
import frc.robot.Subsystems.pathPlanner;
import frc.robot.Subsystems.swerve_drive;
import frc.robot.Subsystems.trackSubsytem;

public class Constants {
    
    public class subsystems {
        public static swerve_drive robotDrive = new swerve_drive();
        public static pathPlanner Path_subsystem = new pathPlanner();
    }

    public class intake {
        public static SparkMax intakeMotor = new SparkMax(11, MotorType.kBrushless);
    }

    public class verticalElevator {
        public static customEncoder verticalEncoder = new customEncoder(new DutyCycleEncoder(0));
        public static SparkFlex verticalMotor = new SparkFlex(12, MotorType.kBrushless);
        public static DigitalInput verticalBase = new DigitalInput(1);
        public static trackSubsytem verticalElevator = new trackSubsytem(.7, 2, Constants.verticalElevator.verticalMotor, Constants.verticalElevator.verticalEncoder, verticalBase, 0.06);//.06 hold speed for elevator
    }

    public class horizontalElevator {
        public static customEncoder horizontalEncoder = new customEncoder(new DutyCycleEncoder(2));
        public static SparkMax horizontalMotor = new SparkMax(10, MotorType.kBrushless);
        public static DigitalInput horizontalBase = new DigitalInput(3);
        public static trackSubsytem horizontalElevator = new trackSubsytem(.7, 2, Constants.horizontalElevator.horizontalMotor, Constants.horizontalElevator.horizontalEncoder, horizontalBase, 0);//0 hold speed for elevator
    }

    public class paths {
        public static SendableChooser<Command> autoChooser;
    }
    
    public class controllers {
        public static CommandXboxController driveController = new CommandXboxController(0);
        public static CommandXboxController operatorController = new CommandXboxController(1);
    }

    public class SWERVE_MOTORS {
        public static final int Swerve_Drive_BL_ID = 3;
        public static final int Swerve_Turn_BL_ID = 4;
        public static final double Swerve_BL_Rad = 0+(Math.PI);

        public static final int Swerve_Drive_FL_ID = 7;
        public static final int Swerve_Turn_FL_ID = 8;
        public static final double Swerve_FL_Rad = 1.57+(Math.PI); //* Math.PI;

        public static final int Swerve_Drive_BR_ID = 1;
        public static final int Swerve_Turn_BR_ID = 2;
        public static final double Swerve_BR_Rad = 4.71+(Math.PI); //* Math.PI;

        public static final int Swerve_Drive_FR_ID = 5;
        public static final int Swerve_Turn_FR_ID = 6;
        public static final double Swerve_FR_Rad = -3.14+(Math.PI); //* Math.PI;

        public static final MAXSwerveModule Swerve_BL = new MAXSwerveModule(Swerve_Drive_BL_ID, Swerve_Turn_BL_ID, Swerve_BL_Rad);
        public static final MAXSwerveModule Swerve_FL = new MAXSwerveModule(Swerve_Drive_FL_ID, Swerve_Turn_FL_ID, Swerve_FL_Rad);
        public static final MAXSwerveModule Swerve_BR = new MAXSwerveModule(Swerve_Drive_BR_ID, Swerve_Turn_BR_ID, Swerve_BR_Rad);
        public static final MAXSwerveModule Swerve_FR = new MAXSwerveModule(Swerve_Drive_FR_ID, Swerve_Turn_FR_ID, Swerve_FR_Rad);

        public static final double maxSpeedMPS = 6.36;
        public static final double maxAngularSpeed = .1 * Math.PI;
        private static final double disX = 23.5/2;
        private static final double disY = 27.5/2;

        // Constant for the positions of the Swerve Motors
        //public static final Translation2d Swerve_Drive_BL_Loc = new Translation2d(-disX, disY);
        //public static final Translation2d Swerve_Drive_FL_Loc = new Translation2d(disX, disY);
        //public static final Translation2d Swerve_Drive_BR_Loc = new Translation2d(-disX, -disY);
        //public static final Translation2d Swerve_Drive_FR_Loc = new Translation2d(disX, -disY);

        public static final Translation2d Swerve_Drive_BL_Loc = new Translation2d(-disX, disY);
        public static final Translation2d Swerve_Drive_FL_Loc = new Translation2d(disX, disY);
        public static final Translation2d Swerve_Drive_BR_Loc = new Translation2d(-disX, -disY);
        public static final Translation2d Swerve_Drive_FR_Loc = new Translation2d(disX, -disY);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            Swerve_Drive_BL_Loc, Swerve_Drive_FL_Loc, Swerve_Drive_BR_Loc, Swerve_Drive_FR_Loc
            );
        
        private static SwerveModulePosition[] MOTOR_POSITIONS(){
            return new SwerveModulePosition[] {
                SWERVE_MOTORS.Swerve_BL.getPosition(),
                SWERVE_MOTORS.Swerve_FL.getPosition(),
                SWERVE_MOTORS.Swerve_BR.getPosition(),
                SWERVE_MOTORS.Swerve_FR.getPosition()
            };
        }
        
        private static SwerveModuleState[] MOTOR_STATES(){
            return new SwerveModuleState[] {
                SWERVE_MOTORS.Swerve_BL.getState(),
                SWERVE_MOTORS.Swerve_FL.getState(),
                SWERVE_MOTORS.Swerve_BR.getState(),
                SWERVE_MOTORS.Swerve_FR.getState()
            };
        }

        public static SlewRateLimiter accLimiter = new SlewRateLimiter(.5);
    }

    public static final class gyro{
        private static final int PIGEON2_ID = 0;

        public static final Pigeon2 main_Gyro = new Pigeon2(PIGEON2_ID);
    }

    public static final class odometry{
        public static final SwerveDriveOdometry swerve_Odemetry = new SwerveDriveOdometry(SWERVE_MOTORS.swerveKinematics, gyro.main_Gyro.getRotation2d(), Constants.SWERVE_MOTORS.MOTOR_POSITIONS());
        
        public static void updatePosition(){
            swerve_Odemetry.update(
                Rotation2d.fromDegrees(gyro.main_Gyro.getYaw().getValueAsDouble()),
                Constants.SWERVE_MOTORS.MOTOR_POSITIONS()
                );
        }
        
        ///Returns pose2d in meters reference
        public static Pose2d getPosition(){
            return swerve_Odemetry.getPoseMeters();
        }

        //Sets the robot pose to the desired pose
        public static void setPositionLocation(Pose2d newPose){
            swerve_Odemetry.resetPosition(gyro.main_Gyro.getRotation2d(), Constants.SWERVE_MOTORS.MOTOR_POSITIONS(), newPose);
        }

        public static void setFullPosition(Pose2d newPose){
            swerve_Odemetry.resetPosition(newPose.getRotation(), Constants.SWERVE_MOTORS.MOTOR_POSITIONS(), newPose);
            Constants.gyro.main_Gyro.setYaw(newPose.getRotation().getDegrees());
        }

        public static ChassisSpeeds botRelativeSpeeds() {
            return Constants.SWERVE_MOTORS.swerveKinematics.toChassisSpeeds(Constants.SWERVE_MOTORS.MOTOR_STATES());
        }

    }

    public static final class ModuleConstants {
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kFreeSpeedRpm = 5767;
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 12; // originally 12
        public static final int kDrivingMotorSpurTeeth = 20; // originally set as number, not const

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
      
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * kDrivingMotorSpurTeeth) / (kDrivingMotorPinionTeeth * kDrivingMotorSpurTeeth);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }
}
