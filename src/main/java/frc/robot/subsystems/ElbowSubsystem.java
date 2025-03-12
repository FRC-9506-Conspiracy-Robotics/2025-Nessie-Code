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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanId;
import frc.robot.Constants.EndEffectorConstants;

public class ElbowSubsystem extends SubsystemBase{
    private final SparkMax elbowMotor = new SparkMax(CanId.elbowMotorCan, MotorType.kBrushless);
    private final RelativeEncoder elbowEncoder = elbowMotor.getEncoder();

    private final DigitalInput elbowLimitSwitch = new DigitalInput(1);
    private boolean homedStatus = false;
    private boolean previousLimitVal = false;

    private double targetAngle;
    private int currentPosition;
    private final double[] elbowAngles = {
        EndEffectorConstants.startingAngle,
        EndEffectorConstants.intakeAngle,
        EndEffectorConstants.restingAngle
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

    Trigger elbowHomeTrigger = new Trigger(() -> elbowLimitSwitch.get()).debounce(0.1);
    
    public ElbowSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(EndEffectorConstants.kElbowCurrentLimit)
        .closedLoopRampRate(0) 
        .inverted(false);

        elbowMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getElbowAngleRad() {
        return Units.rotationsToRadians(elbowEncoder.getPosition() / EndEffectorConstants.kElbowGearing);
    }

    /*public void setZero() {
        System.out.println("Setting encoder position");
        elbowEncoder.setPosition(0.25 * EndEffectorConstants.kElbowGearing);
        System.out.println("New position " + getElbowAngleRad() * 180 / Math.PI);
    }

    public Command resetZero() {
        return runOnce(() -> setZero());
    }*/

    public Command goToStartingPos() {
        return runOnce(() -> {setElbowAngle(elbowAngles[0]);});
    }

    public Command goToIntakePos() {
        return runOnce(() -> {setElbowAngle(elbowAngles[1]);});
    }

    public Command goToRestingPos() {
        return runOnce(() -> {setElbowAngle(elbowAngles[2]);});
    }

    public void elbowGoToAngle(double angleRad) {
        double voltsOut = MathUtil.clamp(
            elbowPid.calculate(getElbowAngleRad(), angleRad)
            + elbowFeed.calculate(
                getElbowAngleRad(),
                elbowPid.getSetpoint().velocity),
            -12,
            12
        );
        elbowMotor.setVoltage(voltsOut);
    }

    public void handleElbowLimit() {
        elbowMotor.set(0.0);
        elbowEncoder.setPosition(Units.radiansToRotations(EndEffectorConstants.startingAngle));
        homedStatus = true;
        targetAngle = EndEffectorConstants.startingAngle;
        elbowPid.reset(EndEffectorConstants.startingAngle);
        System.out.println("Elbow homed");
    }

    public void handleOutOfBounds() {
        homedStatus = false;
        elbowMotor.set(0.0);
        homeElbow();
    }

    public void homeElbow() {
        elbowMotor.set(-0.1);
        if (elbowLimitRising()) {
            handleElbowLimit();
        }
    }

    public boolean isHomed() {
        return homedStatus;
    }

    public void setElbowAngle(double angleInRad) {
        if (!homedStatus && angleInRad > 0) {
            System.out.println("WARNING: Elbow not homed");
            System.out.println("Current elbow position" + getElbowAngleRad());
            System.out.println("Elbow Limit Switch Status" + elbowHomeTrigger.getAsBoolean());
            return;
        }
        targetAngle = MathUtil.clamp(angleInRad, EndEffectorConstants.restingAngle, EndEffectorConstants.startingAngle);
    }

    /*private double holdAngle = 0;
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
    }*/

    public boolean elbowLimitRising() {
        boolean currLimitVal = elbowHomeTrigger.getAsBoolean();
        if (currLimitVal && !previousLimitVal) {
            previousLimitVal = currLimitVal;
            return true;
        }
        previousLimitVal = currLimitVal;
        return false;
    }

    public void updateElbowTelemetry() {
        SmartDashboard.putNumber("Elbow Angle", Units.radiansToDegrees(getElbowAngleRad()));
        SmartDashboard.putBoolean("Elbow homed?", isHomed());
        SmartDashboard.putBoolean("Elbow Limit", elbowLimitSwitch.get());
    }

    @Override
    public void periodic() {
        if (elbowLimitRising()) {
            handleElbowLimit();
        }

        if((getElbowAngleRad() >= EndEffectorConstants.startingAngle)) {
            handleOutOfBounds();
        }

        setElbowAngle(elbowAngles[currentPosition]);
        if(isHomed()) {
            double voltsOut = MathUtil.clamp(
                elbowPid.calculate(getElbowAngleRad(), targetAngle)
                + elbowFeed.calculate(
                    getElbowAngleRad(),
                    elbowPid.getSetpoint().velocity),
                -12,
                12
            );
            elbowMotor.setVoltage(voltsOut);
        }
        updateElbowTelemetry();
    }
}