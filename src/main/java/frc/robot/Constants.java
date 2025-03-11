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
        //motion constants
        public static final double kElevatorGearing = 25;
        public static final double kSprocketPitch = 1.76;
        //extension constants
        public static final double minExtension = 0;
        public static final double maxExtension = 56;
        public static final double l2Setpoint = 24;
        public static final double l3Setpoint = 40;
        public static final double l4Setpoint = 55;
        public static final double intakeSetpoint = 12;
        //pid constants
        public static final double kElevatorKp = 0.25;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 0.0;
        //feedforward constants
        public static final double kElevatorkS = 0.25;
        public static final double kElevatorkV = 0.25;
        public static final double kElevatorkA = 0.00;
        public static final double kElevatorkG = 0.25;
        //motion limits
        public static final double kMaxVelocity = 30;
        public static final double kMaxAccel = 100;
        public static final int kElevatorMaxCurrent = 40;
        public static final double kElevatorRampRate = 0.1;
        //other constraints
        public static final double kElevatorTolerance = 1/16;
        public static final double kScoreDrop = 6;
    }

    //measurements in degrees, where rotating up/counterclockwise is positive
    public static class EndEffectorConstants {
        public static final double kIntakeSpeed = 0.2;
        public static final double startingAngle = Units.degreesToRadians(90);
        public static final double restingAngle = Units.degreesToRadians(35);
        public static final double intakeAngle = Units.degreesToRadians(80);
        public static final double kElbowGearing = 125;

        public static final double kMaxElbowVelocity = 1;
        public static final double kMaxElbowAccel = 1;
        public static final int kElbowCurrentLimit = 40;
        public static final double kElbowRampRate = 0.25;
        
        public static final double kElbowKp = 1;
        public static final double kElbowKi = 0;
        public static final double kElbowKd = 0;
        
        public static final double kElbowkS = 0.0;
        public static final double kElbowkV = 3.0;
        public static final double kElbowkA = 0.0;
        public static final double kElbowkG = 0.1;

        public static final double wristHorizontalAngle = 0;
        public static final double wristVerticalAngle = 90;
        public static final double kWristGearing = 16;

        public static final double kMaxWristVelocity = 2;
        public static final double kMaxWristAccel = 4;
        public static final int kWristCurrentLimit = 40;
        public static final double kWristRampRate = 0.1;

        public static final double kWristKp = 1;
        public static final double kWristKi = 0;
        public static final double kWristKd = 0;
        
        public static final double kWristkS = 0.5;
        public static final double kWristkV = 0.15;
        public static final double kWristkA = 0.0;
        public static final double kWristkG = 0;
    }
}
