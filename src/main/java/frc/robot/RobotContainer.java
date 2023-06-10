package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;
import frc.robot.LimelightInterface;

public class RobotContainer {
	Joystick leftJoystick = new Joystick(0);
	Joystick rightJoystick = new Joystick(1);

    private Drive drive;
    private LimelightInterface limelightInterface;

    public RobotContainer() {
        drive = new Drive();

        configureBindings();
	}

    private void configureBindings() {
        drive.setDefaultCommand(
                drive.driveCommand(this::getLeftAxis, this::getRightAxis));
    }

	private double getLeftAxis() {
		return -deadband(leftJoystick.getY(), 0.05);
	}

	private double getRightAxis() {
		return -deadband(rightJoystick.getX(), 0.05);
	}

	private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance) return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private double getLimelightTX() {
        return limelightInterface.getTX();
    }

    private double getLimelightTY() {
        return limelightInterface.getTY();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
