package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(
        CanId.climberMotorCan, MotorType.kBrushless
    );

    public Climber() {
        SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
        climberMotorConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit)
        .closedLoopRampRate(0)
        .inverted(false);

        climberMotor.configure(
            climberMotorConfig, 
            ResetMode.kNoResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    public Command climb() {
        return runEnd(
            () -> {
                climberMotor.set(1.0);
            },
            () -> {
                climberMotor.set(0.0);
            }
        );
    }

    public Command deploy() {
        return runEnd(
            () -> {
                climberMotor.set(-0.25);
            },
            () -> {
                climberMotor.set(0.0);
            }
        );
    }
}
