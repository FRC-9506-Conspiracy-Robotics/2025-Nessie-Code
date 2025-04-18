package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.EndEffectorConstants;

public class ElbowSubsystem extends SubsystemBase{
    private final SparkMax elbowMotor = new SparkMax(CanId.elbowMotorCan, MotorType.kBrushless);
    private final RelativeEncoder elbowEncoder = elbowMotor.getEncoder();
    private double targetAngleRad = 90 * Math.PI / 180.0;

    private final ProfiledPIDController elbowPid =
        new ProfiledPIDController(
            EndEffectorConstants.kElbowKp,
            EndEffectorConstants.kElbowKi,
            EndEffectorConstants.kElbowKd,
            new Constraints(
                EndEffectorConstants.kMaxElbowVelocity,
                EndEffectorConstants.kMaxElbowAccel
            )
        );
    
    private final ArmFeedforward elbowFeed =
        new ArmFeedforward(
            EndEffectorConstants.kElbowkS,
            EndEffectorConstants.kElbowkG,
            EndEffectorConstants.kElbowkV
        );
    
    public ElbowSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(EndEffectorConstants.kElbowCurrentLimit)
        .closedLoopRampRate(0) 
        .inverted(false);

        elbowMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getElbowAngleRad() {
        return (elbowEncoder.getPosition() / EndEffectorConstants.kElbowGearing) * Math.PI * 2;
    }

    public double getElbowVelocityRadPerSec() {
        return(elbowEncoder.getVelocity() / EndEffectorConstants.kElbowGearing) * Math.PI * 2 / 60;
    }

    public void setZero() {
        System.out.println("Setting encoder position");
        elbowEncoder.setPosition(0.25 * EndEffectorConstants.kElbowGearing);
        //targetAngleRad = getElbowAngleRad();
        System.out.println("New position " + getElbowAngleRad() * 180 / Math.PI);
    }

    public Command fixSetpoint() {
        return runOnce(() -> {
            elbowPid.reset(getElbowAngleRad());
            targetAngleRad = getElbowAngleRad();
        });
    }

    public Command resetZero() {
        return runOnce(() -> setZero());
    }

    public void elbowGoToAngle(double angleRad) {
        double voltsOut = MathUtil.clamp(
            elbowPid.calculate(getElbowAngleRad(), angleRad)
            + elbowFeed.calculateWithVelocities(
                getElbowAngleRad(), elbowPid.getSetpoint().velocity, elbowPid.getSetpoint().velocity),
            -12,
            12
        );
        SmartDashboard.putNumber("Elbow target position", angleRad * 180 / Math.PI);
        elbowMotor.setVoltage(voltsOut);
    }

    public Command setElbowAngle(double angleInRad) {
        return runOnce(
            () -> {
                 targetAngleRad = angleInRad;
            }
        );
    }

    public void stop() {
        elbowMotor.set(0.0);
    }

    public Command stopElbow() {
        return run(() -> stop());
    }

    private double holdAngle = 0;
    public Command holdElbow() {
        return startRun(
            () -> {
                holdAngle = getElbowAngleRad();
                elbowPid.reset(holdAngle);
            },
            () -> {
                setElbowAngle(holdAngle);
                System.out.println("Holding: " + holdAngle);
                System.out.println("Position: " + getElbowAngleRad());
            }
        );
    }
    
    public Command block() {
        return run(() -> {elbowGoToAngle(targetAngleRad);});
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elbow current position", getElbowAngleRad() * 180.0 / Math.PI);
        elbowGoToAngle(targetAngleRad);
    }
}
