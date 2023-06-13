package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimToTargetCommand extends CommandBase{
    private Drive drive;
    private PIDController angleController;

    private double minSpeed = 0.05;

    public AimToTargetCommand(Drive drive) {
        this.drive = drive;

        angleController = new PIDController(0.15, 0, 0);

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(0.1));
    }

    @Override
    public void initialize() {
        angleController.reset();
        angleController.setSetpoint(0);
    }

    @Override
    public void execute() {
        double output = angleController.calculate(drive.getPose().getRotation().getRadians());
        output = Math.copySign(minSpeed, output) + output;

        drive.drivePercent(-output, output);
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}