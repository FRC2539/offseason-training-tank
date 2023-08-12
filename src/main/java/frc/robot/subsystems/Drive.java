package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
    private CANSparkMax leftLeaderMotor;
    private CANSparkMax leftFollowerMotor;
    private CANSparkMax rightLeaderMotor;
    private CANSparkMax rightFollowerMotor;

    public Drive() {
        leftLeaderMotor = new CANSparkMax(4, MotorType.kBrushless);
        leftFollowerMotor = new CANSparkMax(3, MotorType.kBrushless);
        rightLeaderMotor = new CANSparkMax(1, MotorType.kBrushless);
        rightFollowerMotor = new CANSparkMax(2, MotorType.kBrushless);

        leftLeaderMotor.restoreFactoryDefaults();
        rightLeaderMotor.restoreFactoryDefaults();
        leftFollowerMotor.restoreFactoryDefaults();
        rightFollowerMotor.restoreFactoryDefaults();

        leftLeaderMotor.setInverted(false);
        rightLeaderMotor.setInverted(true);
        leftFollowerMotor.follow(leftLeaderMotor, false);
        rightFollowerMotor.follow(rightLeaderMotor, false);

        leftLeaderMotor.enableVoltageCompensation(12.0);
        rightLeaderMotor.enableVoltageCompensation(12.0);
        leftLeaderMotor.setSmartCurrentLimit(30);
        rightLeaderMotor.setSmartCurrentLimit(30);

        leftFollowerMotor.setIdleMode(IdleMode.kCoast);
        rightFollowerMotor.setIdleMode(IdleMode.kCoast);
        leftLeaderMotor.setIdleMode(IdleMode.kCoast);
        rightLeaderMotor.setIdleMode(IdleMode.kCoast);

        leftLeaderMotor.burnFlash();
        rightLeaderMotor.burnFlash();
        leftFollowerMotor.burnFlash();
        rightFollowerMotor.burnFlash();
    }

    public Command driveCommand(DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
        return run(() -> {
            driveArcade(leftAxis.getAsDouble(), rightAxis.getAsDouble());
        });
    }

    public void drivePercent(double leftSpeed, double rightSpeed) {
        leftLeaderMotor.setVoltage(leftSpeed * 12.0);
        rightLeaderMotor.setVoltage(rightSpeed * 12.0);
    }

    public void driveArcade(double xSpeed, double zRotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);

        drivePercent(speeds.left, speeds.right);
    }

    public void driveCurvature(double xSpeed, double zRotation) {
        var speeds = DifferentialDrive.curvatureDriveIK(xSpeed, zRotation, true);

        drivePercent(speeds.left, speeds.right);
    }

    public void stop() {
        leftLeaderMotor.stopMotor();
        rightLeaderMotor.stopMotor();
    }
}
