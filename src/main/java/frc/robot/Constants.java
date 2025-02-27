package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

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
        public static final double kSprocketPitch = Units.inchesToMeters(1.76);
        public static final double kCarriageMass = 20;
        public static final double minExtension = Units.inchesToMeters(0);
        public static final double maxExtension = Units.inchesToMeters((72 - 16) / 2);
        public static final Distance l2Setpoint = Inches.of(24);
        public static final Distance l3Setpoint = Inches.of(40);
        public static final Distance l4Setpoint = Inches.of(54);
        public static final Distance intakeSetpoint = Inches.of(12);
        public static final double kElevatorKp = 30;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 10;
        public static final double kElevatorkS = 0.2;
        public static final double kElevatorkV = 21.8;
        public static final double kElevatorkA = 0.14;
        public static final double kElevatorkG = 0.87;
        public static final double kMaxVelocity = Meters.of(1).per(Second).in(MetersPerSecond);
        public static final double kMaxAccel = Meters.of(0.5).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final int kElevatorMaxCurrent = 40;
        public static final double kElevatorRampRate = 0.1;
        public static final Distance kElevatorTolerance = Inches.of(1/32);
        public static final Distance kScoreDrop = Inches.of(6);
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
