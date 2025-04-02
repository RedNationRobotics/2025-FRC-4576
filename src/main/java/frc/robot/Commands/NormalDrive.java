package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.swerve_drive;

public class NormalDrive extends Command {
    private final XboxController driveController;
    private final swerve_drive drive;

    private double curXSpeed = 0.0;
    private double curZSpeed = 0.0;

    private static final double TRANSLATION_RAMP = 0.01;
    private static final double ROTATION_SCALE = 0.6;
    private static final double DEADBAND = 0.05;

    private boolean doFieldRelativeDrive = true;

    public NormalDrive(XboxController driveController, swerve_drive drive) {

        this.driveController = driveController;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double wantedSpeedX = -MathUtil.applyDeadband(driveController.getLeftY(), DEADBAND);
        double wantedSpeedZ = -MathUtil.applyDeadband(driveController.getLeftX(), DEADBAND);
        double wantedRotAngle = -MathUtil.applyDeadband(driveController.getRightX(), DEADBAND) * ROTATION_SCALE;

        curXSpeed += (wantedSpeedX - curXSpeed) * TRANSLATION_RAMP;
        curZSpeed += (wantedSpeedZ - curZSpeed) * TRANSLATION_RAMP;

        curXSpeed = MathUtil.clamp(curXSpeed, -1.0, 1.0);
        curZSpeed = MathUtil.clamp(curZSpeed, -1.0, 1.0);

        double curRotAngle = wantedRotAngle;
        if (doFieldRelativeDrive){
            drive.drive(curXSpeed, curZSpeed, curRotAngle);
        } else {
            drive.absoluteDrive(curXSpeed, curZSpeed, curRotAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }
}
