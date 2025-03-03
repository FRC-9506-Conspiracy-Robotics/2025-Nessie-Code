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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
    private int currentFloor = 0;
    private final int bottomFloor = 0;
    private final int topFloor = 4;
    private final double[] floorHeights = {0.0, 3.0, 12.0, 15.0, 18.0};
    private final DigitalInput elevatorLimitSwitch = new DigitalInput(0);

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
    
    public final Trigger elevatorLimitTrigger = new Trigger(elevatorLimitSwitch::get);

    public final Trigger minStop = 
        new Trigger(() -> MathUtil.isNear(
            getHeightInches(),
            ElevatorConstants.minExtension,
            1
            )
        );
    
    public final Trigger maxStop = 
        new Trigger(() -> MathUtil.isNear(
            getHeightInches(),
            ElevatorConstants.maxExtension,
            0.1
            )
        );

    public Command minStopWarning() {
        return run(() -> {
            System.out.println("ELEVATOR NEAR BOTTOM");
        });
    }

    public Command maxStopCommand() {
        return run(() -> {
            stopElevator();
            System.out.println("ELEVATOR NEAR TOP");
        });
    }

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
    }

    public double getHeightInches() {
        return 
        (elevatorEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
        * (2 * Math.PI * ElevatorConstants.kSprocketPitch);
    }

    public double getVelocityInchesPerSecond() {
        return ((elevatorEncoder.getVelocity() / 60) / ElevatorConstants.kElevatorGearing)
        * (2 * Math.PI * ElevatorConstants.kSprocketPitch);
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

    public Command setHeight(double height) {
        return run(() -> {
            reachHeight(height);
            System.out.println("Elevator Position: " + getHeightInches());
            System.out.println("Velocity SP is " + elevatorPid.getSetpoint().velocity);
        });
    }

    public void stop() {
        elevatorMotor.set(0.0);
    }

    public Command stopElevator() {
        return runOnce(() -> {
            stop();
            System.out.println("Stopping.");
        });
    }

    public void zero() {
        elevatorEncoder.setPosition(0.0);
    }

    public Command zeroElevator() {
        return runOnce(() -> {
            zero();
            System.out.println("Zeroing.");
        });
    }

    public boolean aroundHeight(double height, double tolerance) {
        return MathUtil.isNear(
            height, 
            getHeightInches(), 
            ElevatorConstants.kElevatorTolerance
        );
    }

    private double holdPoint = 0;
    public Command holdPosition() {
        return startRun(
            () -> {
                holdPoint = getHeightInches();
                elevatorPid.reset(holdPoint);
            },
            () -> {
                reachHeight(holdPoint);
                System.out.println("Holding: " + holdPoint);
                System.out.println("Elevator Position: " + getHeightInches());
            }
        );
    }

    public Command goUpOneFloor() {
        return run(() -> {
            if (currentFloor < topFloor) {
                currentFloor++;
            }
            reachHeight(floorHeights[currentFloor]);
            System.out.println("Going up.");
        });
    }

    public Command goDownOneFloor() {
        return run(() -> {
            if (currentFloor > bottomFloor) {
                currentFloor--;
            }
            reachHeight(floorHeights[currentFloor]);
            System.out.println("Going down.");
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Floor", currentFloor);
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putBoolean("Bottom Limit", elevatorLimitTrigger.getAsBoolean());
    }
}