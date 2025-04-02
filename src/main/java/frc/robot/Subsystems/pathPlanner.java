package frc.robot.Subsystems;

import java.util.Map;
import java.util.TreeMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Libaries.LimelightHelpers;

public class pathPlanner extends SubsystemBase{
  
  private static final Pose2d[] lineupPoses = {
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//BLUE WEST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//BLUE WEST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//BLUE SOUTHWEST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//BLUE SOUTHWEST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//BLUE SOUTHEAST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//BLUE SOUTHEAST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//BLUE EAST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//BLUE EAST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//BLUE NORTHEAST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//BLUE NORTHEAST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//BLUE NORTHWEST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//BLUE NORTHWEST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//RED WEST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//RED WEST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//RED SOUTHWEST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//RED SOUTHWEST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//RED SOUTHEAST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//RED SOUTHEAST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//RED EAST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//RED EAST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//RED NORTHEAST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75)),//RED NORTHEAST RIGHT
    new Pose2d(3.05, 3.825, Rotation2d.fromDegrees(-18.75)),//RED NORTHWEST LEFT
    new Pose2d(3.05, 4.175, Rotation2d.fromDegrees(-18.75))//RED NORTHWEST RIGHT
  };
  //ID to pose lookup
  //key
  //pose index
  private static final Map<Integer, Integer> targetToPose = new TreeMap<Integer,Integer>(){{
    put(6, 0);//BLUE WEST
    put(6, 2);//BLUE SOUTHWEST
    put(6, 4);//BLUE SOUTHEAST
    put(6, 6);//BLUE EAST
    put(6, 8);//BLUE NORTHEAST
    put(6, 10);//BLUE NORTHWEST
    put(6, 0);//RED WEST
    put(6, 2);//RED SOUTHWEST
    put(6, 4);//RED SOUTHEAST
    put(6, 6);//RED EAST
    put(6, 8);//RED NORTHEAST
    put(6, 10);//RED NORTHWEST
  }};

  private Pose2d GetLineupPose(int targetID, boolean lineRight){
    int lineupIndex = targetToPose.get(targetID) + (lineRight?1:0);
    return lineupPoses[lineupIndex];
  }
  
  ///getLineUpCommand
  /// 
  public Command lineUpCommand(boolean lineRight){
    int targetID = 0; //Get limelight viewed id
    targetID = (int)LimelightHelpers.getFiducialID("limelight");
    //Instead of limelight, maybe get the closest pose?
    if (!targetToPose.containsKey(targetID)) return new Command(){};
    
    Pathfinding.setPathfinder(new LocalADStar());
    Pose2d targetPose = GetLineupPose(targetID, lineRight);
    //Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));
    //Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose,
      constraints,
      0.0 // Goal end velocity in meters/sec
    );
    return pathfindingCommand;
  }

  public pathPlanner(){
  }

  public Command getPathFollowCommand(){
    return Constants.paths.autoChooser.getSelected();
  }

  public Command goToPositionCommand(){
    Pose2d target =  new Pose2d(
      SmartDashboard.getNumber("END POINT X", 0), 
      SmartDashboard.getNumber("END POINT Y", 0), 
      Rotation2d.fromDegrees(SmartDashboard.getNumber("END POINT ROT", 0))
      //Constants.gyro.main_Gyro.getRotation2d()
      //Constants.odometry.getPosition().getRotation()
      );
    Command r = AutoBuilder.pathfindToPose(
      target, 
      new PathConstraints(.3, 1, .1, Units.degreesToRadians(0.1)),
      0);
    System.out.println(r.isFinished());
    return r;
  }

  public void setupDashboard(){
    UsbCamera c = CameraServer.startAutomaticCapture();
    c.setVideoMode(PixelFormat.kMJPEG, 640, 480, 10);
    CameraServer.addCamera(c);

    SmartDashboard.putData(Constants.paths.autoChooser);
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
      () -> {return Constants.odometry.getPosition();}, // Robot pose supplier
      (x) -> {Constants.odometry.setFullPosition(x);}, // Method to reset odometry (will be called if your auto has a starting pose)
      () -> {return Constants.odometry.botRelativeSpeeds();}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      //(speeds, feedforwards) -> Constants.subsystems.robotDrive.absoluteDrive(-speeds.vxMetersPerSecond/Constants.SWERVE_MOTORS.maxSpeedMPS,-speeds.vyMetersPerSecond/Constants.SWERVE_MOTORS.maxSpeedMPS, speeds.omegaRadiansPerSecond/Constants.SWERVE_MOTORS.maxAngularSpeed), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      (speeds, feedforwards) -> {Constants.subsystems.robotDrive.autoDrive(speeds, feedforwards);}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(0, 0, 0), // Translation PID constants
              new PIDConstants(1, 0, 0) // Rotation PID constants
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
    Constants.paths.autoChooser = AutoBuilder.buildAutoChooser();
  }
}
