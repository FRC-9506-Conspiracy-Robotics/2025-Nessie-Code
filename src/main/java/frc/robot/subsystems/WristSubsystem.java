package frc.robot.subsystems;

import com.reduxrobotics.canand.CanandSettingsManager.ResultCode;
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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanId;
import frc.robot.Constants.EndEffectorConstants;

public class WristSubsystem extends SubsystemBase{
    private final SparkMax wristMotor = new SparkMax(CanId.wristMotorCan, MotorType.kBrushless);
    private double targetRad;

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
            EndEffectorConstants.kWristkV,
            EndEffectorConstants.kWristkA
        );

    public WristSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(EndEffectorConstants.kWristCurrentLimit)
        .closedLoopRampRate(EndEffectorConstants.kWristRampRate);

        wristMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
        wristPid.setTolerance(Units.degreesToRadians(1));
    }

    public double getAngleRads() {
        return Units.rotationsToRadians(wristEncoder.getPosition() / EndEffectorConstants.kWristGearing);
    }

    public void setAngleRad(double Radian) {
        targetRad = Radian;//MathUtil.clamp(Radian, EndEffectorConstants.wristHorizontalAngle, EndEffectorConstants.wristVerticalAngle + 0.001);
    }

    public Command setTargetRad(double Radian) {
        return runOnce(() -> {setAngleRad(Radian);
        System.out.println("set angle" + Radian);});
    }

    @Override
    public void periodic() {
        setAngleRad(targetRad);
        double voltsOut = MathUtil.clamp(
                wristPid.calculate(getAngleRads(), targetRad) +
                wristFeed.calculate(
                    getAngleRads(),
                    wristPid.getSetpoint().velocity
                ),
                -12, 12 
            );
        wristMotor.setVoltage(voltsOut);
        SmartDashboard.putNumber("Wirst vOut", voltsOut);
    }
}