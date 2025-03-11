// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.ElevatorCommands;
import frc.robot.Commands.LimeLIghtCommands;
import frc.robot.Commands.Motor;
import frc.robot.Constants.intake;
import frc.robot.Libaries.LimelightHelpers;
import edu.wpi.first.math.MathUtil;

public class RobotContainer  {
  
  

  public RobotContainer() {
    configureBindings();
    initSmartDashboard();
    Constants.subsystems.Path_subsystem.setupAutoBuilder();
    Constants.subsystems.Path_subsystem.setupDashboard();
  }

  private void configureBindings() {

    DigitalInput end = new DigitalInput(0);
    DigitalInput orgin = new DigitalInput(1);
    TalonSRX talon = new TalonSRX(17);
    VictorSPX victor = new VictorSPX(0);
    victor.set(ControlMode.PercentOutput, 1);
    Constants.controllers.driveController.a().onTrue(new Motor(end, orgin, talon));
    //Constants.controllers.driveController.a().onTrue(new RunCommand(() -> System.out.println("A pressed"), Swerve_Constants.subsystems.robotDrives()));

    Constants.controllers.driveController.start().onTrue(LimeLIghtCommands.estimatePoseFromLimelight);
    Constants.controllers.driveController.leftBumper().onTrue(new InstantCommand(() -> Constants.subsystems.robotDrive.flipDrive()));
    Constants.controllers.driveController.rightBumper().whileTrue(new RunCommand(() -> Constants.subsystems.robotDrive.brake()));
    Constants.controllers.driveController.rightBumper().onFalse(new RunCommand(() -> Constants.subsystems.robotDrive.unbrake()));

    Constants.controllers.driveController.povRight().onTrue(Constants.subsystems.Path_subsystem.lineUpCommand(true));
    Constants.controllers.driveController.povLeft().onTrue(Constants.subsystems.Path_subsystem.lineUpCommand(false));

    //constants.controls.driveController.
    Constants.controllers.driveController.rightTrigger().whileTrue(Constants.horizontalElevator.evCommands.elevatorUp);
    Constants.controllers.driveController.leftTrigger().whileTrue(Constants.verticalElevator.evCommands.elevatorDown);

    //Constants.controllers.assistantController.rightTrigger().whileTrue(new InstantCommand(()->{Constants.intake.intakeMotor.set(1);}));
    //Constants.controllers.assistantController.leftTrigger().whileTrue(new InstantCommand(()->{Constants.intake.intakeMotor.set(-1);}));
    
    
    //Constants.controllers.assistantController.rightTrigger().onFalse(
    //  new InstantCommand(
    //    //Modulo?
    //    //Why no modulo to reset to 0?
    //  ()->{
    //    Constants.controllers.assistantController.leftBumper().getAsBoolean() ?
    //    return : Constants.intake.intakeMotor.set(0);
    //  })
    //);
    //ROBOT ELEVATOR POSITIONS
    //Constants.controllers.assistantController.a().onTrue(Constants.subsystems.dri)
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
