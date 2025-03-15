package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class PositionsListV {
    public static int index = 0;
    public static ElevatorCommands evVCommands = new ElevatorCommands(Constants.verticalElevator.verticalElevator);
    public static String[] PosNames = new String[]{
      "Zero",
      "Coral Station",
      "Algae 1",
      "Coral 1",
      "Algae 2",
      "Coral 2",
      "Coral 3"
    };

    public static double[] poseHeights = new double[]{
      0.01,.125,.83,1.75,2.68,3.4,6.15
    };

    public static Command[] TidePositions = new Command[]{
      evVCommands.getSetPosCommand(0.01),  //Zero
      evVCommands.getSetPosCommand(.125),   //Coral Station
      evVCommands.getSetPosCommand(.83),  //Algae 1
      evVCommands.getSetPosCommand(1.75), //Coral 1
      evVCommands.getSetPosCommand(2.68), //Algae 2
      evVCommands.getSetPosCommand(3.4), //Coral 2
      evVCommands.getSetPosCommand(6.15) //Coral 3

    };

    public static void setName(){
      SmartDashboard.putString("SelectedPosition", PosNames[index]);
      SmartDashboard.putNumber("Height", poseHeights[index]);
    }

    public static Command getCommand(){
      return TidePositions[index];
    }

    public static void up(){
      index +=1;
      index %= TidePositions.length;
      setName();
    }

    public static void down(){
      index-=1;
      if (index < 0) index = TidePositions.length-1;
      setName();
    }

    public static void setPosition() {
      Constants.verticalElevator.verticalElevator.setDesiredPosition(poseHeights[index]);
    }
}
