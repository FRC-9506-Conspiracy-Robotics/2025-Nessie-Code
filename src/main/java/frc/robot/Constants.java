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
        public static final double kIntakeSpeed = 0.2;
        public static final double startingAngle = Units.degreesToRadians(90);
        public static final double restingAngle = Units.degreesToRadians(30);
        public static final double clearanceAngle = Units.degreesToRadians(55);
        public static final double intakeAngle = Units.degreesToRadians(50);
        public static final double kElbowGearing = 25;

        public static final double kMaxElbowVelocity = 1;
        public static final double kMaxElbowAccel = 1;
        public static final int kElbowCurrentLimit = 40;
        public static final double kElbowRampRate = 0.25;
        
        public static final double kElbowKp = 1;
        public static final double kElbowKi = 0;
        public static final double kElbowKd = 0.5;
        
        public static final double kElbowkS = 0.0;
        public static final double kElbowkV = 0.5;
        public static final double kElbowkA = 0.04;
        public static final double kElbowkG = 1;

        public static final double wristHorizontalAngle = 0;
        public static final double wristVerticalAngle = 90;
        public static final double kWristGearing = 16;

        public static final double kMaxWristVelocity = 1;
        public static final double kMaxWristAccel = 1;
        public static final int kWristCurrentLimit = 40;
        public static final double kWristRampRate = 0.1;

        public static final double kWristKp = 1;
        public static final double kWristKi = 0;
        public static final double kWristKd = 0.5;
        
        public static final double kWristkS = 0.0;
        public static final double kWristkV = 0.5;
        public static final double kWristkA = 0.0;
        public static final double kWristkG = 0.0;



        
        
    }
}
