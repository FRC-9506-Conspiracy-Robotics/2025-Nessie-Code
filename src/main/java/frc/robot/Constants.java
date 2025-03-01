package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

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

    public static class ElevatorConstants {
        public static final double kElevatorGearing = 25;
        public static final double kSprocketTeeth = 22;
        public static final double kSprocketPitch = 1.76;
        public static final double kCarriageMass = 20;
        public static final double minExtension = 0;
        public static final double maxExtension = 28; //for stage 1
        public static final double l2Setpoint = 24;
        public static final double l3Setpoint = 40;
        public static final double l4Setpoint = 54;
        public static final double intakeSetpoint = 12;

        public static final double kElevatorKp = 0.1;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 1;

        public static final double kElevatorkS = 0.0;
        public static final double kElevatorkV = 20;
        public static final double kElevatorkA = 0.01;
        public static final double kElevatorkG = 0.25;

        public static final double kMaxVelocity = 20;
        public static final double kMaxAccel = 500;
        public static final int kElevatorMaxCurrent = 40;
        public static final double kElevatorRampRate = 0.1;
        public static final double kElevatorTolerance = 1/32;
        public static final double kScoreDrop = 6;
    }

    //measurements in degrees, where rotating up/counterclockwise is positive
    public static class EndEffectorConstants {
        public static final Angle startingAngle = Degrees.of(60);
        public static final Angle restingAngle = Degrees.of(0);
        public static final Angle clearanceAngle = Degrees.of(25);
        public static final Angle intakeAngle = Degrees.of(20);
        public static final Angle wristHorizontalAngle = Degrees.of(0);
        public static final Angle wristVerticalAngle = Degrees.of(90);
    }
}
