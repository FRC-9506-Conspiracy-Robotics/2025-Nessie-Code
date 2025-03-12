package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.EndEffectorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(CanId.intakeMotorCan, MotorType.kBrushless);
    private final SparkMax intakeFollower = new SparkMax(CanId.intakeFollowerCan, MotorType.kBrushless);

    public IntakeSubsystem() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
            intakeConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(EndEffectorConstants.kWristCurrentLimit)
            .closedLoopRampRate(0);
            
            intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();
            intakeFollowerConfig.idleMode(IdleMode.kBrake)
            .follow(CanId.intakeMotorCan, true);

            intakeFollower.configure(intakeFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runIntake() {
        return startEnd(
            () -> {
                intakeMotor.set(EndEffectorConstants.kIntakeSpeed);
            },
            () -> {
                intakeMotor.set(0);
            }
        );
    }

    public Command reverseIntake() {
        return startEnd(
            () -> {
                intakeMotor.set(-EndEffectorConstants.kIntakeSpeed);
            },
            () -> {
                intakeMotor.set(0);
            }
        );
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    public Command stopIntake() {
        return run(() -> stop());
    }
}
