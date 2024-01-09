package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.Constants.ControllerConstants;

public class FalconTest extends SubsystemBase{
    private final ThrustmasterJoystick leftDriveController =
    new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    
    private TalonFX falconTest = new TalonFX(0);
    
    public Command setFalconVoltage() {
        return runOnce(
            () -> {
              falconTest.setVoltage(getLeftControllerXAxis());
            });
    }

    public double getLeftControllerXAxis() {
        return leftDriveController.getXAxis().getRaw();
    }
    
    @Override
    public void periodic() {
        setFalconVoltage();
    }
}
