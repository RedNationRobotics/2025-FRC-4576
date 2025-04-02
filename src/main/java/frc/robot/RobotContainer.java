// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ElevatorCommands;
import frc.robot.Commands.ElevatorToPosition;
import frc.robot.Commands.LimeLIghtCommands;
import frc.robot.Commands.Limelight_LineupLeft_CMD;
import frc.robot.Commands.Limelight_LineupRight_CMD;
import frc.robot.Commands.PositionsListV;
import frc.robot.Commands.driveToOrigin;
import frc.robot.Commands.driveToPresetPosition;
import frc.robot.Libaries.LimelightHelpers;
import frc.robot.Subsystems.pathPlanner;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;

public class RobotContainer  {
  
  public ConditionalCommand safeMove;

  private static double driveSpeedSlow = .2;
  private double curDriveSpeed;

  //LED STRIPS
  //23 21 LEFT
  //25 21 RIGHT

  public RobotContainer() {
    AddressableLED m_led =  new AddressableLED(0);
    m_led.setLength(90);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(90);
    AddressableLEDBufferView back = buffer.createView(23, 64);

    //RED AND GREEN CHANNELS ARE BACKWARDS
    LEDPattern color =  (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? LEDPattern.solid(Color.kGreen) : LEDPattern.solid(Color.kBlue);
    LEDPattern sponsor = LEDPattern.solid(Color.kWhite);

    color.applyTo(buffer);
    sponsor.applyTo(back);

    m_led.setData(buffer);

    m_led.start();


    configureBindings();
    initSmartDashboard();

    PositionsListV.registerCommands();
    Constants.subsystems.Path_subsystem.setupAutoBuilder();
    Constants.subsystems.Path_subsystem.setupDashboard();

    LimeLIghtCommands.setupDictionary();
    curDriveSpeed = 1;
  }

  private void configureBindings() {

    Constants.controllers.driveController.start().onTrue(LimeLIghtCommands.estimatePoseFromLimelight);
    Constants.controllers.driveController.leftBumper().onTrue(new InstantCommand(() -> Constants.subsystems.robotDrive.flipDrive()));
    Constants.controllers.driveController.rightBumper().whileTrue(new RunCommand(() -> Constants.subsystems.robotDrive.brake()));
    Constants.controllers.driveController.rightBumper().onFalse(new RunCommand(() -> Constants.subsystems.robotDrive.unbrake()));

    //Constants.controllers.driveController.povRight().onTrue(Constants.subsystems.Path_subsystem.lineUpCommand(true));
    //Constants.controllers.driveController.povLeft().onTrue(Constants.subsystems.Path_subsystem.lineUpCommand(false));

    Constants.controllers.driveController.back().onTrue(new InstantCommand(() -> {
      curDriveSpeed = (curDriveSpeed<.9) ? 1 : driveSpeedSlow;
    }));

    pathPlanner bsSubsystem = new pathPlanner();

    Constants.controllers.operatorController.povUp().onTrue(
      new InstantCommand(() -> {PositionsListV.up();System.out.println("up");}, bsSubsystem)
      );
    Constants.controllers.operatorController.povDown().onTrue(
      new InstantCommand(() -> {PositionsListV.down();}, bsSubsystem)
      );
    

    //Safe elevator height controls
    //ElevatorCommands evVCommands = new ElevatorCommands(Constants.verticalElevator.verticalElevator);
    //safeMove = new ConditionalCommand(
    //  evVCommands.driveToPresetPosition,
    //  evVCommands.elevatorStop
    //  , () -> {return Constants.horizontalElevator.horizontalElevator.encoder.getValue()>1.4;}
    //);
    
    ElevatorCommands evHCommands = new ElevatorCommands(Constants.horizontalElevator.horizontalElevator);
    //Constants.controllers.operatorController.x().onTrue(new ElevatorToPosition(Constants.horizontalElevator.horizontalElevator, 2.2));
    //Constants.controllers.operatorController.y().onTrue(new ElevatorToPosition(Constants.horizontalElevator.horizontalElevator, 1));
    Constants.controllers.operatorController.b().onTrue(
        new SequentialCommandGroup(
          new ElevatorToPosition(Constants.horizontalElevator.horizontalElevator, 1.5),
          new driveToOrigin(Constants.verticalElevator.verticalElevator),
          evHCommands.driveToOrigin
        )
      );

    Constants.controllers.operatorController.rightBumper().onFalse(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(0);}, bsSubsystem
    ));
    Constants.controllers.operatorController.leftBumper().onFalse(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(0);}, bsSubsystem
    ));
    Constants.controllers.operatorController.rightBumper().whileTrue(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(1);}, bsSubsystem
    ));
    Constants.controllers.operatorController.leftBumper().whileTrue(new InstantCommand(
      () -> {Constants.intake.intakeMotor.set(-1);}, bsSubsystem
    ));

    //Constants.controllers.driveController.povLeft().whileTrue(Constants.subsystems.left);
    //Constants.controllers.driveController.povRight().whileTrue(Constants.subsystems.right);
    Constants.controllers.driveController.povLeft().whileTrue(new Limelight_LineupLeft_CMD());
    Constants.controllers.driveController.povRight().whileTrue(new Limelight_LineupRight_CMD());

    //ON START move to position
    SequentialCommandGroup scoreUp = new SequentialCommandGroup(
      new ElevatorToPosition(Constants.horizontalElevator.horizontalElevator, 1.5),
      new driveToPresetPosition(Constants.verticalElevator.verticalElevator),
      new driveToPresetPosition(Constants.horizontalElevator.horizontalElevator)
    );
    Constants.controllers.operatorController.a().onTrue(scoreUp);
  }

  private void initSmartDashboard(){
    //Camera
    HttpCamera httpCamera = new HttpCamera("limelight", "http://10.45.76.201:5800/");
    CameraServer.startAutomaticCapture(httpCamera);

    //
  }

  public Command getAutonomousCommand() {
    return Constants.subsystems.Path_subsystem.getPathFollowCommand();
  }

  public Command getTelaopCommand() {
    return new RunCommand(() -> Constants.subsystems.robotDrive.drive(MathUtil.applyDeadband(MathUtil.clamp(Constants.controllers.driveController.getLeftY(), -1, 1),.1) * curDriveSpeed, MathUtil.applyDeadband(MathUtil.clamp(Constants.controllers.driveController.getLeftX(), -1, 1),.1)*curDriveSpeed, MathUtil.applyDeadband(MathUtil.clamp(Constants.controllers.driveController.getRightX(), -1, 1),.1)*Math.pow(curDriveSpeed,1.0)), Constants.subsystems.robotDrive);
  }
}
