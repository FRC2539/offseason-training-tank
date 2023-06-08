package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
    public static final double wheelRadius = Units.inchesToMeters(6.0) / 2;
    public static final double trackWidth = Units.inchesToMeters(26.0);
    public static final double gearRatio = 10.7;

    private CANSparkMax leftLeaderMotor;
    private CANSparkMax leftFollowerMotor;
    private CANSparkMax rightLeaderMotor;
    private CANSparkMax rightFollowerMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private AHRS gyro;

    private DifferentialDriveOdometry odometry;
    public DifferentialDriveKinematics kinematics;

    private DoubleArrayPublisher posePublisher;

    public Drive() {
        gyro = new AHRS();

        leftLeaderMotor = new CANSparkMax(4, MotorType.kBrushless);
        leftFollowerMotor = new CANSparkMax(3, MotorType.kBrushless);
        rightLeaderMotor = new CANSparkMax(1, MotorType.kBrushless);
        rightFollowerMotor = new CANSparkMax(2, MotorType.kBrushless);

        leftEncoder = leftLeaderMotor.getEncoder();
        rightEncoder = rightLeaderMotor.getEncoder();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

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
        leftLeaderMotor.setIdleMode(IdleMode.kBrake);
        rightLeaderMotor.setIdleMode(IdleMode.kBrake);

        leftLeaderMotor.burnFlash();
        rightLeaderMotor.burnFlash();
        leftFollowerMotor.burnFlash();
        rightFollowerMotor.burnFlash();

        odometry = new DifferentialDriveOdometry(getGyroRotation(), 0, 0, new Pose2d());
        kinematics = new DifferentialDriveKinematics(trackWidth);

        posePublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("/Drive/pose").getEntry(new double[] {});
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(getGyroRotation(), getLeftDistance(), getRightDistance(), pose2d);
    }

    public double getLeftDistance() {
        return Units.rotationsToRadians(leftEncoder.getPosition() / gearRatio) * wheelRadius;
    }

    public double getRightDistance() {
        return Units.rotationsToRadians(rightEncoder.getPosition() / gearRatio) * wheelRadius;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / gearRatio) * wheelRadius,
                Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / gearRatio) * wheelRadius);
    }

    public Command driveCommand(DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
        return run(() -> driveCurvature(leftAxis.getAsDouble(), rightAxis.getAsDouble()));
    }

    public void driveVoltage(double leftVoltage, double rightVoltage) {
        leftLeaderMotor.setVoltage(leftVoltage);
        rightLeaderMotor.setVoltage(rightVoltage);
    }

    public void drivePercent(double leftSpeed, double rightSpeed) {
        leftLeaderMotor.setVoltage(leftSpeed * 12.0);
        rightLeaderMotor.setVoltage(rightSpeed * 12.0);
    }

    public void driveArcade(double xSpeed, double zRotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, false);

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

    @Override
    public void periodic() {
        var pose = odometry.update(getGyroRotation(), getLeftDistance(), getRightDistance());

        // Send the pose data for telemetry visualization
        posePublisher.accept(new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
    }
}
