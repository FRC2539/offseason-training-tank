package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private DigitalInput proximitySensor;
    private CANSparkMax motor;
    private Joystick joystick;
    private Trigger button;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        proximitySensor = new DigitalInput(0);
        motor = new CANSparkMax(1, MotorType.kBrushless);
        joystick = new Joystick(0);
        button = new JoystickButton(joystick, 2);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        // Lab 1
        // Disable the motor once we see something
        if (!proximitySensor.get()) {
            motor.stopMotor();
        } else {
            motor.set(0.1);
        }

        // Lab 2
        // Run the motor when the button is pressed
        // if (button.getAsBoolean()) {
        // 	motor.set(0.1);
        // } else {
        // 	motor.stopMotor();
        // }

        // Lab 3
        // Control the motor's speed based on a joystick axis
        // motor.set(joystick.getY());
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
