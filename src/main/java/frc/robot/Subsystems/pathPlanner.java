package frc.robot.Subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pathPlanner extends SubsystemBase{
    public pathPlanner(){
    }

    public Command getPathFollowCommand(){
        
      Pathfinding.setPathfinder(new LocalADStar());
      PathPlannerPath alignment = null;
      try {
        alignment = PathPlannerPath.fromPathFile("New Path");
      } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
      }
      // Since we are using a holonomic drivetrain, the rotation component of this pose
      // represents the goal holonomic rotation
      //Pose2d targetPose = new Pose2d(alignment.getAllPathPoints().get(1).position,Rotation2d.kZero);
      //// Create the constraints to use while pathfinding
      //PathConstraints constraints = new PathConstraints(
      //  3.0, 4.0,
      //  Units.degreesToRadians(540), Units.degreesToRadians(720));
      //// Since AutoBuilder is configured, we can use it to build pathfinding commands
      //Command pathfindingCommand = AutoBuilder.pathfindToPose(
      //  targetPose,
      //  constraints,
      //  0.0 // Goal end velocity in meters/sec
      //);
      //return pathfindingCommand;

      return Constants.paths.autoChooser.getSelected();
    }

    public void setupDashboard(){
      
    }

    public void setupAutoBuilder(){
            // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = null;
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }
    
        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> Constants.odometry.getPosition(), // Robot pose supplier
                (x) -> Constants.odometry.setFullPosition(x), // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Constants.odometry.botRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> Constants.subsystems.robotDrive.absoluteDrive(-speeds.vxMetersPerSecond/Constants.SWERVE_MOTORS.maxSpeedMPS,-speeds.vyMetersPerSecond/Constants.SWERVE_MOTORS.maxSpeedMPS, speeds.omegaRadiansPerSecond/Constants.SWERVE_MOTORS.maxAngularSpeed), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
}
