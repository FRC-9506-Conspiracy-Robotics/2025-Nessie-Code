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

    public static class CanId {
        public static final int intakeMotorCan = 9;
        public static final int intakeFollowerCan = 10;
        public static final int wristMotorCan = 11;
        public static final int elbowMotorCan = 12;
        public static final int elevatorMotorCan = 13;
        public static final int elevatorFollowerCan = 14;
    }

    //measurements in inches
    public static class ElevatorConstants {
        public static final double minExtension = 0;
        public static final double maxExtension = 72;
        public static final double l2Setpoint = 24;
        public static final double l3Setpoint = 40;
        public static final double l4Setpoint = 54;
        public static final double intakeSetpoint = 12;
    }

    //measurements in degrees, where rotating up/counterclockwise is positive
    public static class EndEffectorConstants {
        public static final double startingAngle = 60;
        public static final double restingAngle = 0;
        public static final double clearanceAngle = 25;
        public static final double intakeAngle = 20;
        public static final double wristHorizontalAngle = 0;
        public static final double wristVerticalAngle = 90;
    }
}
