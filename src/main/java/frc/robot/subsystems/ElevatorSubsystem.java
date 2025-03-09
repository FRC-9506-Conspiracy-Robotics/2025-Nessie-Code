package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanId;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    //intialize motors
    private final SparkMax elevatorMotor = new SparkMax(CanId.elevatorMotorCan, MotorType.kBrushless);
    private final SparkMax elevatorFollower = new SparkMax(CanId.elevatorFollowerCan, MotorType.kBrushless);
    
    private final DigitalInput bottomLimitSwitch;
    private boolean homedStatus = false;

    private double currentTarget = ElevatorConstants.minExtension;
    private double currentPosition;
    private double targetPosition;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;

    private int currentFloor = 0;
    private final int bottomFloor = 0;
    private final int topFloor = 3;
    private final double[] floorHeights = {
        ElevatorConstants.minExtension, 
        ElevatorConstants.l2Setpoint, 
        ElevatorConstants.l3Setpoint, 
        ElevatorConstants.l4Setpoint
    };

    //intitalize relative encoder based on the main motor
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private final ProfiledPIDController elevatorPid = 
        new ProfiledPIDController(
            ElevatorConstants.kElevatorKp,
            ElevatorConstants.kElevatorKi,
            ElevatorConstants.kElevatorKd,
            new Constraints(
                ElevatorConstants.kMaxVelocity,
                ElevatorConstants.kMaxAccel
            )
        );

    private final ElevatorFeedforward elevatorFeed = 
        new ElevatorFeedforward(
            ElevatorConstants.kElevatorkS,
            ElevatorConstants.kElevatorkG,
            ElevatorConstants.kElevatorkV,
            ElevatorConstants.kElevatorkA
        );


    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.kElevatorMaxCurrent)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate);

        elevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.kElevatorMaxCurrent)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
        .follow(elevatorMotor, true);

        elevatorFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        bottomLimitSwitch = new DigitalInput(0);

        elevatorPid.setTolerance(ElevatorConstants.kElevatorTolerance);
    }

    public double getHeightInches() {
        return (elevatorEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
        * (2 * Math.PI * ElevatorConstants.kSprocketPitch);
    }

    public double getVelocityInchesPerSecond() {
        return ((elevatorEncoder.getVelocity() / 60) / ElevatorConstants.kElevatorGearing)
        * (2 * Math.PI * ElevatorConstants.kSprocketPitch);
    }

    public void handleBottomLimit() {
        elevatorMotor.set(0.0);
        elevatorEncoder.setPosition(0.0);
        homedStatus = true;
        targetPosition = 0.0;
        currentState = new TrapezoidProfile.State(ElevatorConstants.minExtension, 0);
        goalState = new TrapezoidProfile.State(ElevatorConstants.minExtension, 0);
        elevatorPid.reset(currentState);
    }

    private double calculateFeedForward(TrapezoidProfile.State state) {
        return ElevatorConstants.kElevatorkS * Math.signum(state.velocity) +
            ElevatorConstants.kElevatorkG +
            ElevatorConstants.kElevatorkV * state.velocity;
    }

    public void setHeightInches(double height) {
        if (!homedStatus && height > 0) {
            System.out.println("WARNING: Elevator not homed");
        }
    }

    public void reachHeight(double height) {
        double voltsOut = MathUtil.clamp(
            elevatorPid.calculate(getHeightInches(), height) +
            elevatorFeed.calculateWithVelocities(
                elevatorPid.getSetpoint().velocity,
                elevatorPid.getSetpoint().velocity
            ),
            -12, 12 
        );

        elevatorMotor.setVoltage(voltsOut);
    }

    public Command incrementFloor() {
        return runOnce(() -> {
                if (currentFloor < topFloor) {
                  currentFloor++;
                }
            }
        );
    }

    public Command decrementFloor() {
        return runOnce(() -> {
                if (currentFloor > bottomFloor) {
                    currentFloor--;
                }
            }
        );
    }

    public Command goToCurrentFloor() {
        return runOnce(
            () -> {reachHeight(floorHeights[currentFloor]);});
    }

    public Command goToFloor(int floor) {
        return runOnce(() -> {reachHeight(floorHeights[floor]);});
    }

    public Command goToBottom() {
        return runOnce(() -> {reachHeight(0.0);});
    }

    public 

    @Override
    public void periodic() {
        currentPosition = getHeightInches();


    }
} 