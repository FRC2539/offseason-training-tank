package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightInterface {

    public double getTX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
}

