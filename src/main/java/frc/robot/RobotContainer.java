// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Libraries.LimelightHelpers;
import frc.robot.commands.ElevatorCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //#region controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_secondController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //#endregion

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Defines button actions
   */
  private void configureBindings() {

    //#region DRIVER2 elevator position setting
    //m_secondController.a().whileTrue(ElevatorCommands.pos1);
    //m_secondController.b().whileTrue(ElevatorCommands.pos2);
    //m_secondController.x().whileTrue(ElevatorCommands.pos3);
    //m_secondController.y().whileTrue(ElevatorCommands.pos4);
    //#endregion

    //#region DRIVER2 elevator arm controls
      //Open arm
      //Close arm
      //forward
      //backward
      //positions
    //#endregion

    //#region DRIVER1 elevator driver controls
    //m_driverController.a().whileTrue(ElevatorCommands.driveToPresetPosition);
    //m_driverController.leftBumper().whileTrue(ElevatorCommands.elevatorUp);
    //m_driverController.rightBumper().whileTrue(ElevatorCommands.elevatorDown);
    //#endregion

    //#region DRIVER1 drive controls
      //Joystick drive Simple joystick drive
      //speed switch SET SPEED TO DIFFERENT
      //Emergency stop SET WHEELS TO X
      //Vision lockon MOVE TO DESIRED VISION TARGET
    //#endregion

  }

  private void configureLimelight(){
    
  }

  private void initSmartDashboard() {
    SmartDashboard.putString("Test value"," SOME STRING HERE ");
  }

  private void setSubSystemCommands() {
    //Drive subsystem to default drive always
    //Elvator up and down possibly
    //Arm forwards and backwards possibly
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    throw new NoSuchMethodError("Not yet implemented");
    //Get auto from 
  }
}
