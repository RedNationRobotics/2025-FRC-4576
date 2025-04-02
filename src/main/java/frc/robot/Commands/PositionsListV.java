package frc.robot.Commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.pathPlanner;

public class PositionsListV {
    public static int index = 0;
    public static ElevatorCommands evVCommands = new ElevatorCommands(Constants.verticalElevator.verticalElevator);
    public static SequentialCommandGroup curCommand= new SequentialCommandGroup(
      new driveToOrigin(Constants.horizontalElevator.horizontalElevator),
      new driveToOrigin(Constants.verticalElevator.verticalElevator)
    );

    public static String[] PosNames = new String[]{
      "Coral Station",
      "Algae 1",
      "Coral 1",
      "Algae 2",
      "Coral 2",
      "Coral 3"
    };

    public static double[] poseHeights = new double[]{
      .16,.53,1.5,2.20,3.25,6.1
    };

    public static double[] poseOut = new double[]{
      .02, 2.4, 2.4, 2.4, 2.4, 2.4
    };

    public static void setName(){
      SmartDashboard.putString("SelectedPosition", PosNames[index]);
      SmartDashboard.putNumber("Height", poseHeights[index]);
    }

    public static void registerCommands(){
      for (int i = 0; i< PosNames.length; i++){
        System.out.println(i + " " + PosNames[i]);
        NamedCommands.registerCommand(PosNames[i],
          new SequentialCommandGroup(
            new InstantCommand(()-> {System.out.println("AUTONOMOUS");},Constants.verticalElevator.verticalElevator),
            new ElevatorToPosition(Constants.horizontalElevator.horizontalElevator, 1.5),
            new ElevatorToPosition(Constants.verticalElevator.verticalElevator, poseHeights[i]),
            new ElevatorToPosition(Constants.horizontalElevator.horizontalElevator, 2.4)
          )
        );
      }
      NamedCommands.registerCommand("OUT", new InstantCommand(
        () -> {Constants.intake.intakeMotor.set(1);}, new pathPlanner()
      ));
      NamedCommands.registerCommand("IN", new InstantCommand(
        () -> {Constants.intake.intakeMotor.set(-1);}, new pathPlanner()
      ));
      NamedCommands.registerCommand("STOPINTAKE", new InstantCommand(
        () -> {Constants.intake.intakeMotor.set(0);}, new pathPlanner()
      ));
      NamedCommands.registerCommand("ZERO ELEVATORS", new SequentialCommandGroup(
        new ElevatorToPosition(Constants.horizontalElevator.horizontalElevator, 1.5),
        new driveToOrigin(Constants.verticalElevator.verticalElevator),
        new driveToOrigin(Constants.horizontalElevator.horizontalElevator)       
      ));
    }

    public static void up(){
      index +=1;
      index %= poseHeights.length;
      setName();
      setPosition();
    }

    public static void down(){
      index-=1;
      if (index < 0) index = poseHeights.length-1;
      setName();
      setPosition();
    }

    public static void setPosition() {
      Constants.verticalElevator.verticalElevator.setDesiredPosition(poseHeights[index]);
      Constants.horizontalElevator.horizontalElevator.setDesiredPosition(poseOut[index]);
    }
}
