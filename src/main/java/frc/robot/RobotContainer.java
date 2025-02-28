// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.NormalDrive;
import frc.robot.Subsystems.swerve_drive;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


  XboxController driveController = new XboxController(0);
  public Command getTelaopCommand(swerve_drive drive) {
    //return new NormalDrive(driveController, drive);
    return new RunCommand(() -> drive.drive(MathUtil.applyDeadband(MathUtil.clamp(driveController.getLeftY(), -0.5, 0.5),.1), MathUtil.applyDeadband(MathUtil.clamp(driveController.getLeftX(), -0.5, 0.5),.1), MathUtil.applyDeadband(MathUtil.clamp(driveController.getRightX(), -1, 1),.1)), drive);
  }
}


