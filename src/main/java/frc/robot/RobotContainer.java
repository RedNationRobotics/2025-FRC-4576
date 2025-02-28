// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Motor;
import frc.robot.Libaries.LimelightHelpers;
import frc.robot.Subsystems.Swerve_Drives;

public class RobotContainer  {
  
  
  
  CommandXboxController driverController = new CommandXboxController(0);
  Swerve_Drives drive = new Swerve_Drives();
  
  public RobotContainer() {
    configureBindings();
    initSmartDashboard();
  }

  private void configureBindings() {

    DigitalInput end = new DigitalInput(0);
    DigitalInput orgin = new DigitalInput(1);
    TalonSRX talon = new TalonSRX(17);
    VictorSPX victor = new VictorSPX(0);
    victor.set(ControlMode.PercentOutput, 1);
    driverController.a().onTrue(new Motor(end, orgin, talon));
    //driverController.a().onTrue(new RunCommand(() -> System.out.println("A pressed"), Swerve_Drives()));
  }

  private void configureLimelight() {

  }

  private void initSmartDashboard(){
    SmartDashboard.putString("Test", "Some String Here");
    SmartDashboard.putNumber( "YUUU",1200);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void getLimelightValues(){
    SmartDashboard.putNumber("Target Angle X", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("Target Angle Y", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("Area", LimelightHelpers.getTA("limelight"));

  }

  //public Command getTeleopCommand(Object robot_drive){
  //  return new RunCommand(()->robot_drive.drive(driverController.getLeftY(), driverController.getLeftX(), driverController.getRightX()), robot_drive);
  //}
}



