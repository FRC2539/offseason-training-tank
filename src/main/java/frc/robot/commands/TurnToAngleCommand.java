package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnToAngleCommand extends CommandBase {
    private Drive drive;
    private static PIDController angleController = new PIDController(0.15, 0, 0);
    private double desiredAngle;

    private double minSpeed = 0.05;

    public TurnToAngleCommand(Drive drive, double desiredAngleDegrees) {
        this.drive = drive;
        this.desiredAngle = Units.degreesToRadians(desiredAngleDegrees);

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(0.1));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        angleController.reset();
        angleController.setSetpoint(desiredAngle);
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
