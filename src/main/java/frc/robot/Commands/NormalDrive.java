package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Libaries.LimelightHelpers;
import frc.robot.Subsystems.swerve_drive;

public class NormalDrive extends Command {
    private final XboxController driveController;
    private final swerve_drive drive;

   /*
    *private double curXSpeedfast = 0.0;
    *private double curZSpeedfast = 0.0;
    *
    *private double curXSpeedslow = 0.0;
    *private double curZSpeedslow = 0.0;
    */ 

    private double curXSpeed = 0.0;
    private double curZSpeed = 0.0;

    private static final double TRANSLATION_RAMP = 0.01;
    private static final double ROTATION_SCALE = 0.6;
    private static final double DEADBAND = 0.05;

    private double speedMod = 1;
    private double roationMod = 1;

    public NormalDrive(XboxController driveController, swerve_drive drive) {

        this.driveController = driveController;
        this.drive = drive;

        addRequirements(drive);
    }
    @Override
    public void initialize() {
        System.out.println("NormalDrive Command Initialized");
        SmartDashboard.putNumber("Drive Speed Multiplier", speedMod);
        SmartDashboard.putNumber("Rotation Speed Multiplier", roationMod);



    }

    @Override
    public void execute() {
        speedMod = SmartDashboard.getNumber("Drive Speed Multiplier", 1 );
        speedMod = SmartDashboard.getNumber("Rotation Speed Multiplier", 1 );

        double wantedSpeedX = -MathUtil.applyDeadband(driveController.getLeftY(), DEADBAND) * speedMod;
        double wantedSpeedZ = -MathUtil.applyDeadband(driveController.getLeftX(), DEADBAND) * speedMod;
        double wantedRotAngle = -MathUtil.applyDeadband(driveController.getRightX(), DEADBAND) * ROTATION_SCALE * roationMod;

        curXSpeed += (wantedSpeedX - curXSpeed) * TRANSLATION_RAMP;
        curZSpeed += (wantedSpeedZ - curZSpeed) * TRANSLATION_RAMP;

        curXSpeed = MathUtil.clamp(curXSpeed, -1.0, 1.0);
        curZSpeed = MathUtil.clamp(curZSpeed, -1.0, 1.0);


        double curRotAngle = wantedRotAngle;

        System.out.println("CurX: " + curXSpeed + " | CurZ: " + curZSpeed + " | CurRot: " + curRotAngle);


        drive.drive(curXSpeed, curZSpeed, curRotAngle);
        //double wantedSlowSpeedX = -MathUtil.applyDeadband(driveController.getLeftY(), DEADBAND);
        //double wantedSlowSpeedZ = -MathUtil.applyDeadband(driveController.getLeftX(), DEADBAND);
        //double wantedSlowRotAngle = -MathUtil.applyDeadband(driveController.getRightX(), DEADBAND) * ROTATION_SCALE;
//
        //double wantedFastSpeedx = -MathUtil.applyDeadband(driveController.getLeftY(), DEADBAND)*speedMod;
        //double wantedFastSpeedz = -MathUtil.applyDeadband(driveController.getLeftX(), DEADBAND);
        //double wantedFastRotAngle = -MathUtil.applyDeadband(driveController.getRightX(), DEADBAND) * ROTATION_SCALE;
//
        //curXSpeedslow += (wantedSlowSpeedX - curXSpeedslow) * TRANSLATION_RAMP;
        //curZSpeedslow += (wantedSlowSpeedZ - curZSpeedslow) * TRANSLATION_RAMP;
        //
        //curXSpeedfast += (wantedFastSpeedX - curXSpeedslow) * TRANSLATION_RAMP;
        //curXSpeedfast += (wantedFastSpeedZ - curZSpeedslow) * TRANSLATION_RAMP;
//
        //curXSpeedslow = MathUtil.clamp(curXSpeedslow, -0.5, 0.5);
        //curZSpeedslow = MathUtil.clamp(curZSpeedslow, -0.5, 0.5);
//
        //curXSpeedfast = MathUtil.clamp(curXSpeedfast, -1.0, 1.0);
        //curZSpeedfast = MathUtil.clamp(curZSpeedfast, -1.0, 1.0);
//
//
//

        //double curRotAngle = wantedSlowRotAngle;

        //System.out.println("CurX: " + curXSpeed + " | CurZ: " + curZSpeed + " | CurRot: " + curRotAngle);


        //drive.drive(curXSpeedslow, curXSpeedfast, curZSpeedslow, curXSpeedfast, curRotAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("NormalDrive Command Ended");
        drive.drive(0, 0, 0);
    }
}
