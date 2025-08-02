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
    
    private final DigitalInput bottomLimitSwitch = new DigitalInput(0);
    private boolean homedStatus = false;
    private boolean previousLimitVal = false;

    private double targetPosition;
    private int currentFloor = 0;
    private final int bottomFloor = 0;
    private final int topFloor = 4;
    private final double[] floorHeights = {
        ElevatorConstants.minExtension,
        ElevatorConstants.floorInakeSetpoint,
        ElevatorConstants.intakeSetpoint, 
        //ElevatorConstants.l2Setpoint, 
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

    Trigger bottomTrigger = new Trigger(() -> bottomLimitSwitch.get()).debounce(0.1);

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

        elevatorPid.setTolerance(ElevatorConstants.kElevatorTolerance);
    }

    //calculated by taking the number of rotations of the motor and divding it by the gear ratio
    //then multiply by the circumference of the sprocket using the pitch
    //then times 2 and plus 1 to account for cascade and the second stage
    public double getHeightInches() {
        return (elevatorEncoder.getPosition() / ElevatorConstants.kElevatorGearing)
        * (Math.PI * ElevatorConstants.kSprocketPitch) * 2 + 1;
    }

    //called on the rising edge of the botoom limit switch
    public void handleBottomLimit() {
        elevatorMotor.set(0.0);
        elevatorEncoder.setPosition(0.0);
        homedStatus = true;
        targetPosition = 0.0;
        currentFloor = 0;
        elevatorPid.reset(0.0);
        System.out.println("Elevator homed");
    }

    //called when the elevator either travels to a position out of the min max extension
    public void handleOutOfBounds() {
        homedStatus = false;
        elevatorMotor.set(0.0);
        homeElevator();
    }

    public void homeElevator() {
        elevatorMotor.set(-0.1);
        if (bottomLimitSwitch.get()) {
            handleBottomLimit();
        }
    }

    public boolean isHomed() {
        return homedStatus;
    }

    public void setHeightInches(double height) {
        if (!homedStatus && height > 0) {
            System.out.println("WARNING: Elevator not homed");
            System.out.println("Current elevator position: " + getHeightInches());
            System.out.println("Limit switch status " + bottomTrigger.getAsBoolean());
            return;
        }

        //ensures target position is within the max and min extension
        targetPosition = MathUtil.clamp(height, ElevatorConstants.minExtension, ElevatorConstants.maxExtension);
    }

    //command to increase floor number if it is less than the top floor
    public Command incrementFloor() {
        return runOnce(() -> {
                if (currentFloor < topFloor) {
                  currentFloor++;
                }
            }
        );
    }

    //command to decrease floor number if it is greater than the bottom floor
    public Command decrementFloor() {
        return runOnce(() -> {
                if (currentFloor > bottomFloor) {
                    currentFloor--;
                }
            }
        );
    }

    //command to set the target height to the current floor setpoint
    public Command goToCurrentFloor() {
        return runOnce(
            () -> {setHeightInches(floorHeights[currentFloor]);}
        );
    }

    //command to set the target height to the desired floor
    public Command goToFloor(int floor) {
        return runOnce(() -> {currentFloor = floor;});
    }

    public double getCurrentFloor() {
        return currentFloor;
    }

    //command to go to the bottom of the elevator
    public Command goToBottom() {
        return runOnce(() -> {setHeightInches(ElevatorConstants.minExtension);});
    }

    //checks if the limit switch press is on the rising edge
    public boolean bottomLimitRising() {
        boolean currLimitVal = bottomTrigger.getAsBoolean();
        if (currLimitVal && !previousLimitVal) {
            previousLimitVal = currLimitVal;
            return true;
        }
        previousLimitVal = currLimitVal;
        return false;
    }

    public void updateElevatorTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Current Floor", getCurrentFloor());
        SmartDashboard.putBoolean("Homed?", isHomed());
        SmartDashboard.putBoolean("Bottom Limit", bottomLimitSwitch.get());
        SmartDashboard.putNumber("elevator position setpoint", elevatorPid.getSetpoint().position);
    }

    @Override 
    public void periodic() {

        //handle bottom limit if the limit switch is pressed
        if (bottomLimitRising()) {
            handleBottomLimit();
        }

        //handles out of bounds if the current height is not within max and min extension
        if ((getHeightInches() >= ElevatorConstants.maxExtension) || (getHeightInches() <= ElevatorConstants.minExtension)) {
            handleOutOfBounds();
        }

        //only runs elevator if it is homed by bottom limit switch
        //no hold position needed because targetPosition achieves the same effect when in periodic()
        setHeightInches(floorHeights[currentFloor]);
        if (isHomed()) {
            double voltsOut = MathUtil.clamp(
                elevatorPid.calculate(getHeightInches(), targetPosition) +
                elevatorFeed.calculate(
                    elevatorPid.getSetpoint().velocity
                ),
                -12, 12 
            );
            elevatorMotor.setVoltage(voltsOut);
        }
        updateElevatorTelemetry();
    }
} 