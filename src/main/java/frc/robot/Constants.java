package frc.robot;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Modules.MAXSwerveModule;



public class Constants {
    public class SWERVE_MOTORS {

        
        public static final int Swerve_Drive_BL_ID = 0;
        public static final int Swerve_Turn_BL_ID = 0;
        public static double Swerve_BL_Rad = 0;

        public static final int Swerve_Drive_FL_ID = 0;
        public static final int Swerve_Turn_FL_ID = 0;
        public static double Swerve_FL_Rad = 0;

        public static final int Swerve_Drive_BR_ID = 0;
        public static final int Swerve_Turn_BR_ID = 0;
        public static double Swerve_BR_Rad = 0;

        public static final int Swerve_Drive_FR_ID = 0;
        public static final int Swerve_Turn_FR_ID = 0;
        public static double Swerve_FR_Rad = 0;

        //Kit Bot's Location on a 2d map "Translation2d" and "Serve_Ind-Wheel_Loc" 
        public static final Translation2d Swerve_BL_Location = new Translation2d (0,0);
        public static final Translation2d Swerve_BR_Location = new Translation2d (0,0);
        public static final Translation2d Swerve_FL_Location = new Translation2d (0,0);
        public static final Translation2d Swerve_FR_Location = new Translation2d (0,0);   

        //Kit Bots 
        public static final MAXSwerveModule Swerve_BL = new MAXSwerveModule(Swerve_Drive_BL_ID, Swerve_Turn_BL_ID, Swerve_BL_Rad);
        public static final MAXSwerveModule Swerve_FL = new MAXSwerveModule(Swerve_Drive_FL_ID, Swerve_Turn_FL_ID, Swerve_FL_Rad);
        public static final MAXSwerveModule Swerve_BR = new MAXSwerveModule(Swerve_Drive_BR_ID, Swerve_Turn_BR_ID, Swerve_BR_Rad);
        public static final MAXSwerveModule Swerve_FR = new MAXSwerveModule(Swerve_Drive_FR_ID, Swerve_Turn_FR_ID, Swerve_FR_Rad);

        public static final double maxSpeedMPS = 6.36;
        public static final double maxAngularSpeed = 2 * Math.PI;
        
        //
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            Swerve_BL_Location, Swerve_BR_Location, Swerve_FL_Location, Swerve_FR_Location
        );
    }

    public static final class gyro{
        private static final int PIGEON2_ID = 0;

        public static final Pigeon2 main_Gyro = new Pigeon2(PIGEON2_ID);
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
