package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.EndEffectorConstants;

public class ClawSubsystem extends SubsystemBase{
    private final SparkMax intakeMotor = new SparkMax(CanId.intakeMotorCan, MotorType.kBrushless);
    private final SparkMax intakeFollower = new SparkMax(CanId.intakeFollowerCan, MotorType.kBrushless);

    private final SparkMax wristMotor = new SparkMax(CanId.wristMotorCan, MotorType.kBrushless);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

    private final ProfiledPIDController wristPid = 
        new ProfiledPIDController(
            EndEffectorConstants.kWristKp,
            EndEffectorConstants.kWristKi,
            EndEffectorConstants.kWristKd,
            new Constraints(
                EndEffectorConstants.kMaxWristVelocity,
                EndEffectorConstants.kMaxWristAccel
            )
        );

    private final ArmFeedforward wristFeed =
        new ArmFeedforward(
            EndEffectorConstants.kWristkS,
            EndEffectorConstants.kWristkG,
            EndEffectorConstants.kWristkV
        );
    
    public ClawSubsystem() {

        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(EndEffectorConstants.kWristCurrentLimit)
        .closedLoopRampRate(0)
        .inverted(false);

        wristMotor.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

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

    // TODO: @Alberto
    // Make a new command to reverse the intake

    public void stop() {
        intakeMotor.set(0.0);
    }

    public Command stopIntake() {
        return run(() -> stop());
    }

    public void setZero() {
        wristEncoder.setPosition(EndEffectorConstants.wristVerticalAngle);
    }

    public void wristGoToAngle(double angleRad) {
        double voltsOut = MathUtil.clamp(
            wristPid.calculate(getWristAngleRad(), angleRad)
            + wristFeed.calculateWithVelocities(getWristAngleRad(), wristPid.getSetpoint().velocity, wristPid.getSetpoint().velocity),
            -12,
            12
        );

        wristMotor.setVoltage(voltsOut);
    }

    public double getWristAngleRad() {
        return (wristEncoder.getPosition() / EndEffectorConstants.kWristGearing) * Math.PI * 2;
    }
    
    public double getWristVelocityRadPerSec() {
        return(wristEncoder.getVelocity() / EndEffectorConstants.kWristGearing) * Math.PI * 2 / 60;
    }

    public Command setWristAngle(double angleInRad) {
        return run(() -> {
                wristGoToAngle(angleInRad);
                System.out.println("Wrist Position: " + getWristAngleRad());
            }
        );
    }

    private double holdAngle = 0;
    public Command holdClaw() {
        return startRun(
            () -> {
                holdAngle = getWristAngleRad();
                wristPid.reset(holdAngle);
            },
            () -> {
                setWristAngle(holdAngle);
                System.out.println("Holding: " + holdAngle);
                System.out.println("Position: " + getWristAngleRad());
            }
        );
    }
}
