// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.ElevatorCommands;
import frc.robot.Commands.LimeLIghtCommands;
import frc.robot.Commands.Limelight_LineupLeft_CMD;
import frc.robot.Commands.Limelight_LineupRight_CMD;
import frc.robot.Commands.PositionsListV;
import frc.robot.Libaries.LimelightHelpers;
import frc.robot.Subsystems.pathPlanner;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.MathUtil;

public class RobotContainer  {
  
  

  public RobotContainer() {
    configureBindings();
    initSmartDashboard();
    Constants.subsystems.Path_subsystem.setupAutoBuilder();
    Constants.subsystems.Path_subsystem.setupDashboard();
    
    ElevatorCommands evHCommands = new ElevatorCommands(Constants.horizontalElevator.horizontalElevator);
    HttpCamera limelightFeed = new HttpCamera("limelight", "http://10.45.76.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    CameraServer.addCamera(limelightFeed);
  }

  private void configureBindings() {

    Constants.controllers.driveController.start().onTrue(LimeLIghtCommands.estimatePoseFromLimelight);
    Constants.controllers.driveController.leftBumper().onTrue(new InstantCommand(() -> Constants.subsystems.robotDrive.flipDrive()));
    Constants.controllers.driveController.rightBumper().whileTrue(new RunCommand(() -> Constants.subsystems.robotDrive.brake()));
    Constants.controllers.driveController.rightBumper().onFalse(new RunCommand(() -> Constants.subsystems.robotDrive.unbrake()));

    Constants.controllers.driveController.povRight().onTrue(Constants.subsystems.Path_subsystem.lineUpCommand(true));
    Constants.controllers.driveController.povLeft().onTrue(Constants.subsystems.Path_subsystem.lineUpCommand(false));

    pathPlanner bsSubsystem = new pathPlanner();

    Constants.controllers.operatorController.povUp().onTrue(
      new InstantCommand(() -> {PositionsListV.up();System.out.println("up");}, bsSubsystem)
      );
    Constants.controllers.operatorController.povDown().onTrue(
      new InstantCommand(() -> {PositionsListV.down();}, bsSubsystem)
      );
    Constants.controllers.operatorController.a().onTrue(
      (new InstantCommand(() -> {PositionsListV.setPosition();}, bsSubsystem))
      );

    ElevatorCommands evVCommands = new ElevatorCommands(Constants.verticalElevator.verticalElevator);
    Constants.verticalElevator.verticalElevator.setDefaultCommand(evVCommands.driveToPresetPosition);
    
    ElevatorCommands evHCommands = new ElevatorCommands(Constants.horizontalElevator.horizontalElevator);
    Constants.controllers.operatorController.x().onTrue(evHCommands.getSetPosCommand(2.0));
    Constants.controllers.operatorController.y().onTrue(evHCommands.getSetPosCommand(1.0));
    Constants.controllers.operatorController.b().onTrue(evHCommands.driveToOrigin);

    Constants.horizontalElevator.horizontalElevator.setDefaultCommand(evHCommands.driveToPresetPosition);

    Constants.controllers.operatorController.rightBumper().whileFalse(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(0);}, bsSubsystem
    ));
    Constants.controllers.operatorController.leftBumper().whileFalse(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(0);}, bsSubsystem
    ));
    Constants.controllers.operatorController.rightBumper().whileTrue(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(1);}, bsSubsystem
    ));
    Constants.controllers.operatorController.leftBumper().whileTrue(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(-1);}, bsSubsystem
    ));

    Limelight_LineupRight_CMD right = new Limelight_LineupRight_CMD();
    Limelight_LineupLeft_CMD left = new Limelight_LineupLeft_CMD();
    Constants.controllers.driveController.povLeft().whileTrue(left);
    Constants.controllers.driveController.povRight().whileTrue(right);
  }

  private void initSmartDashboard(){
    SmartDashboard.putString("Test", "Some String Here");
    SmartDashboard.putNumber( "YUUU",1200);
    //HttpCamera httpCamera = new HttpCamera("Limelight", "10.45.76.11:5800");
    //Shuffleboard.getTab("Tab").add(httpCamera);
  }

  public Command getAutonomousCommand() {
    return Constants.subsystems.Path_subsystem.getPathFollowCommand();
  }

  public void getLimelightValues(){
    SmartDashboard.putNumber("Target Angle X", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("Target Angle Y", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("Area", LimelightHelpers.getTA("limelight"));
  }

  public Command getTelaopCommand() {
    return new RunCommand(() -> Constants.subsystems.robotDrive.drive(MathUtil.applyDeadband(MathUtil.clamp(Constants.controllers.driveController.getLeftY(), -1, 1),.1), MathUtil.applyDeadband(MathUtil.clamp(Constants.controllers.driveController.getLeftX(), -1, 1),.1), MathUtil.applyDeadband(MathUtil.clamp(Constants.controllers.driveController.getRightX(), -1, 1),.1)), Constants.subsystems.robotDrive);
  }
}
