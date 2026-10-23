package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class DriverConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.05;
    }

    public static class SwerveConstants {
        public static final double maxSpeed = Units.feetToMeters(15);
        public static final double slowSpeed = Units.feetToMeters(7.5);
    }

    public static class PivotConstants {
        public static final int pivotMotorCan = 1; // make sure cans are correct
        public static final int currentLimit = 40;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0; // tune thsese
        public static final double gearRatio = 100; // make sure this is correct

        public static final double minAngle = 0; // degrees
        public static final double maxAngle = 90; // idk max
    }
    
    public static class ShooterConstants {
        public static final int shooterMotorCan = 2;
        public static final int currentLimit = 40;

        public static final double shootRpm = 3000;    // tune this
        public static final double intakeRpm = -1500;
    }
}
