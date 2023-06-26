package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightInterface;


public class DriveToTargetCommand extends CommandBase {
    private double targetLimelightTY;
    private Drive drive;
    private PIDController driveController;
    private LimelightInterface limelightInterface;

    private double minSpeed = 0.05;

    public DriveToTargetCommand(double targetLimelightTY, Drive drive, LimelightInterface limelightInterface) {
        this.targetLimelightTY = targetLimelightTY;
        this.drive = drive;
        this.limelightInterface = limelightInterface;

        driveController = new PIDController(0.01, 0, 0);

        driveController.setTolerance(0.3);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        driveController.reset();
        driveController.setSetpoint(targetLimelightTY);
    }

    @Override
    public void execute() {
        double output = driveController.calculate(limelightInterface.getTY());
        output = Math.copySign(minSpeed, output) + output;

        drive.drivePercent(output, output);
    }

    @Override
    public boolean isFinished() {
        return driveController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}