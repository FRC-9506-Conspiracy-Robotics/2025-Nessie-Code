package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorCan, MotorType.kBrushless);
    private final SparkClosedLoopController pid = shooterMotor.getClosedLoopController();
    private final RelativeEncoder encoder = shooterMotor.getEncoder();
    private double targetRpm = 0;

    public ShooterSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.currentLimit)
            .inverted(false);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0001, 0, 0);
        config.closedLoop.feedForward
            .kV(1.0 / 473.0);

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVelocity(double rpm) {
        targetRpm = rpm;
        pid.setSetpoint(rpm, ControlType.kVelocity);
    }

    public boolean atSpeed() {
        return Math.abs(encoder.getVelocity() - targetRpm) < 100;
    }

    public void setShooterSpeed(double rpm) {
        setVelocity(rpm);
    }

    public void stop() {
        shooterMotor.stopMotor();
    }

    public Command setShooterSpeedCommand(double rpm) {
        return startEnd(() -> setShooterSpeed(rpm), this::stop);
    }
}