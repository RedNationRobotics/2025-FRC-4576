// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Motor;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Subsystems.swerve_drive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleopCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    // This resets the motors to 0 (It should at least, not 100% sure yet)
    Constants.SWERVE_MOTORS.Swerve_BL.resetEncoders();
    Constants.SWERVE_MOTORS.Swerve_FL.resetEncoders();
    Constants.SWERVE_MOTORS.Swerve_BR.resetEncoders();
    Constants.SWERVE_MOTORS.Swerve_FR.resetEncoders();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.getLimelightValues();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

TalonSRX motor = new TalonSRX(17);

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    m_teleopCommand = m_robotContainer.getTelaopCommand();
    Constants.subsystems.robotDrive.setDefaultCommand((m_teleopCommand));
    
    //XboxController c = new XboxController(0);
    //Trigger button = new Trigger(() -> Constants.controllers.driveController.getAButtonPressed());
    //SWERVE DRIVE WILL BE REPLACE WITH YOUR SUBSYSTEM :)
    //button.onTrue(new RunCommand(()-> motor.set(TalonSRXControlMode.PercentOutput, 1), drive));
    //Constants.controllers.driveController.a().onTrue(new RunCommand(()-> motor.set(TalonSRXControlMode.PercentOutput, 1), drive));
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("teleopPeriodic", kDefaultPeriod);
    SmartDashboard.putNumber("Gyro Rot:", Constants.gyro.main_Gyro.getRotation2d().getDegrees());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    //TalonSRX motor = new TalonSRX(17);
    TalonSRX motor = new TalonSRX(17);
    motor.set(TalonSRXControlMode.PercentOutput, 1);
    //motor.set(TalonSRXControlMode.Velocity, 12);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
