package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveForwardCommand extends CommandBase {
    private Drive drive;
    private static PIDController leftController = new PIDController(0.3, 0, 0);
    private static PIDController rightController = new PIDController(0.3, 0, 0);
    private double desiredDistance;
    private double leftTarget;
    private double rightTarget;

    private double minSpeed = 0.1;

    public DriveForwardCommand(Drive drive, double desiredDistanceMeters) {
        this.drive = drive;
        this.desiredDistance = desiredDistanceMeters;

        leftController.setTolerance(0.03);
        rightController.setTolerance(0.03);
    }

    @Override
    public void initialize() {
        leftTarget = drive.getLeftDistance() + desiredDistance;
        rightTarget = drive.getRightDistance() + desiredDistance;

        leftController.reset();
        rightController.reset();

        leftController.setSetpoint(leftTarget);
        rightController.setSetpoint(rightTarget);
    }

    @Override
    public void execute() {
        double leftOutput = Math.copySign(minSpeed, desiredDistance) + leftController.calculate(drive.getLeftDistance());
        double rightOutput = Math.copySign(minSpeed, desiredDistance) + rightController.calculate(drive.getRightDistance());

        drive.drivePercent(leftOutput, rightOutput);
    }

    @Override
    public boolean isFinished() {
        return leftController.atSetpoint() && rightController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
