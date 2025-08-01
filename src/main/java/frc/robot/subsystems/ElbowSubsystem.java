package frc.robot.subsystems;

import javax.print.attribute.standard.PrinterIsAcceptingJobs;

import com.ctre.phoenix.motorcontrol.can.BaseMotorControllerConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanId;
import frc.robot.Constants.EndEffectorConstants;

public class ElbowSubsystem extends SubsystemBase{
    private final SparkMax elbowMotor = new SparkMax(CanId.elbowMotorCan, MotorType.kBrushless);
    private final RelativeEncoder elbowEncoder = elbowMotor.getEncoder();

    private final DigitalInput bottomLimitSwitch = new DigitalInput(1);
    private boolean homedStatus = false;
    private boolean previousLimitVal = false;

    private double targetAngle; //radians
    private int currentStage = 0;
    private double commandedPower = 0.0;
    private final int bottomStage = 0;
    private final int topStage = 3;
    private final double[] stageAngles = {
        EndEffectorConstants.fullyVertical,
        EndEffectorConstants.intakeAngle,
        EndEffectorConstants.fullyHorizontal,
        EndEffectorConstants.floorIntakeAngle
    };

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
    
    Trigger bottomTrigger = new Trigger(() -> bottomLimitSwitch.get()).debounce(0.1);

    public ElbowSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(EndEffectorConstants.kElbowCurrentLimit)
        .closedLoopRampRate(0) 
        .inverted(false);

        elbowMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getElbowAngleRad() {
        return (elbowEncoder.getPosition() / EndEffectorConstants.kElbowGearing) 
        * Math.PI * 2;
    }

    public void handleBottomLimit() {
        commandedPower = 0.0;
        elbowEncoder.setPosition(Units.radiansToRotations(EndEffectorConstants.fullyVertical) * EndEffectorConstants.kElbowGearing);
        homedStatus = true;
        targetAngle = EndEffectorConstants.fullyVertical;
        currentStage = 0;
        elbowPid.reset(EndEffectorConstants.fullyVertical);
        System.out.println("Elbow homed");
    }

    public void handleOutOfBounds() {
        homedStatus = false;
        commandedPower = 0.0;
        homeElbow();
    }

    public void homeElbow() {
        commandedPower = 0.0;
        if (bottomLimitSwitch.get()) {
            handleBottomLimit();
        }
    }

    public boolean isHomed() {
        return homedStatus;
    }

    public void setAngleRad(double rad) {
        if (!homedStatus && rad > 0) {
            System.out.println("WARNING: Elbow not homed");
            System.out.println("Current elbow position: " + getElbowAngleRad());
            System.out.println("Limit switch status " + bottomTrigger.getAsBoolean());
            return;
        }

        targetAngle = MathUtil.clamp(rad, EndEffectorConstants.floorIntakeAngle, EndEffectorConstants.fullyVertical);
    }

    public Command goToCurrentStage() {
        return runOnce(
            () -> {setAngleRad(stageAngles[currentStage]);}
        );
    }

    public Command goToStage(int stage) {
        return runOnce(() -> {currentStage = stage;});
    }

    public double getCurrentStage() {
        return currentStage;
    }

    public Command goToBottom() {
        return runOnce(() -> {setAngleRad(EndEffectorConstants.floorIntakeAngle);});
    }

    public boolean bottomLimitRising() {
        boolean currentVal = bottomTrigger.getAsBoolean();
        if (currentVal && !previousLimitVal) {
            previousLimitVal = currentVal;
            return true;
        }
        previousLimitVal = currentVal;
        return false;
    }

    public void updateElbowTelemetry() {
        SmartDashboard.putNumber("Elbow Angle", getElbowAngleRad());
        SmartDashboard.putNumber("Elbow Target Angle", targetAngle);
        SmartDashboard.putNumber("Elbow commanded power", commandedPower);
        SmartDashboard.putNumber("Current stage", getCurrentStage());
        SmartDashboard.putBoolean("Elbow homed?", isHomed());
        SmartDashboard.putBoolean("Elbow Bottom Limit", bottomLimitSwitch.get());
        SmartDashboard.putNumber("elbow position setpoint", elbowPid.getSetpoint().position);
        SmartDashboard.putNumber("elbow velocity setpoint", elbowPid.getSetpoint().velocity);
    }
    
    @Override
    public void periodic() {
        if (bottomLimitRising()) {
            handleBottomLimit();
        }

        if ((getElbowAngleRad() > EndEffectorConstants.fullyVertical) || (getElbowAngleRad() <= EndEffectorConstants.floorIntakeAngle - (20.0 * Math.PI / 180.0))) {
            handleOutOfBounds();
        }

        setAngleRad(stageAngles[currentStage]);
        if (isHomed()) {
            double voltsOut = MathUtil.clamp(
                elbowPid.calculate(getElbowAngleRad(), targetAngle) +
                elbowFeed.calculate(
                    targetAngle,
                    elbowPid.getSetpoint().velocity
                ),
                -12, 12 
            );
            commandedPower = voltsOut;
        } else {
            commandedPower = 1.2;
        }
        elbowMotor.set(commandedPower / 12.0);
        updateElbowTelemetry();
    }
}
