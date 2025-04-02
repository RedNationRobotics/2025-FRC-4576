// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.ElevatorCommands;
import frc.robot.Commands.LimeLIghtCommands;
import frc.robot.Commands.driveToOrigin;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

    Constants.gyro.main_Gyro.setYaw(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(new driveToOrigin(Constants.horizontalElevator.horizontalElevator));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    LimeLIghtCommands.doPositioning = false;
    Constants.subsystems.robotDrive.removeDefaultCommand();

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
    LimeLIghtCommands.doPositioning = true;
    CommandScheduler.getInstance().schedule(new driveToOrigin(Constants.horizontalElevator.horizontalElevator));
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_teleopCommand = m_robotContainer.getTelaopCommand();
    Constants.subsystems.robotDrive.setDefaultCommand((m_teleopCommand));
  }

  @Override
  public void teleopPeriodic() {
    
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
