package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor = new SparkMax(PivotConstants.pivotMotorCan, MotorType.kBrushless);
    private final SparkClosedLoopController pid = pivotMotor.getClosedLoopController();
    private final RelativeEncoder encoder = pivotMotor.getEncoder();
    private double targetAngle = 0;

    public PivotSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(PivotConstants.currentLimit)
            .inverted(false);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD)
            .outputRange(-0.5, 0.5); // limits speed make it -1, 1 for 100%
        config.softLimit
            .forwardSoftLimit(PivotConstants.maxAngle)
            .reverseSoftLimit(PivotConstants.minAngle)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);
        config.encoder.positionConversionFactor(360.0 / PivotConstants.gearRatio);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(PivotConstants.minAngle);
    }

    public void setPivotPosition(boolean up) {
        double target;
        if (up) {
            target = PivotConstants.maxAngle;
        } else {
            target = PivotConstants.minAngle;
        }
        targetAngle = target;
        pid.setSetpoint(target, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public boolean atTarget() {
        return Math.abs(encoder.getPosition() - targetAngle) < 1;
    }

    public void stop() {
        pivotMotor.stopMotor();
    }

    public Command setPivotPositionCommand(boolean up) {
    return runOnce(() -> setPivotPosition(up))
        .andThen(Commands.waitUntil(this::atTarget))
        .withTimeout(2.5); // don't hang forever if it can't reach the target
    }
}