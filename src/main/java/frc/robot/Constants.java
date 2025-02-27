package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class DriverConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.05;
    }
    public static class SwerveConstants {
        public static final double maxSpeed = Units.feetToMeters(15);
    }
}
