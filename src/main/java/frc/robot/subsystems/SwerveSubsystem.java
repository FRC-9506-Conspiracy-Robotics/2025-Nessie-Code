package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
    //create swerve object
    private final SwerveDrive swerveDrive;
    public SwerveSubsystem(File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        //Attempts to initialize swerve drive using the given filepath throws error if files cannot be found
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                Constants.SwerveConstants.maxSpeed, 
                new Pose2d(
                    new Translation2d(
                        Meter.of(1), 
                        Meter.of(4)
                    ), 
                    Rotation2d.fromDegrees(0)
                )
            );
            } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); 
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //corrects for skew that occurs as angular velocity increases, tune this
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); //enable to resync encoders when swerve is not moving
    }

    //constructs swerve drive using parameter configs (field centric or not, controller or keyboard, etc)
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(
            driveCfg, 
            controllerCfg, 
            Constants.SwerveConstants.maxSpeed, 
            new Pose2d(
                new Translation2d(
                    Meter.of(2), 
                    Meter.of(0)
                ), 
            Rotation2d.fromDegrees(0)
            )
        );
        setupPathPlanner();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    //loads auto config from pathplanner
    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            final boolean enableFeedForward = true;
            AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (enableFeedForward) {
                        swerveDrive.drive(
                            speedsRobotRelative,
                            swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces()
                        );
                    } else {
                        swerveDrive.setChassisSpeeds(speedsRobotRelative);
                    }
                },
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
        PathfindingCommand.warmupCommand().schedule();
    }

    //creates path using pathplanner
    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    //drives to point on field using pathplanner pathfinding
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(),
            4.0,
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720)
        );
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0)
        );
    }
    
    //characterizes the drive motors using sysid
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                new Config(),
                this,
                swerveDrive,
                12,
                true
            ),
            3.0, 5.0, 3.0
        );
    }

    //samething but with angle motors
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                new Config(),
                this,
                swerveDrive
            ),
            3.0, 5.0, 3.0
        );
    }

    //centers modules to zero
    public Command centerModulesCommand() {
        //first creates an array as a list of all the swerve modules, then sets each angle to 0.0
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
    }
    
    //replaces swerve module feedforwards with new one
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    //moves the robot
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
        return run(
            () -> {
                swerveDrive.driveFieldOriented(velocity.get());
            }
        );
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    //moves robot based on translation and angle
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity()), 0.8
                ),
                Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(), true, false
            );
        });
    }

    //drives a certain distance and stops using pose
    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
        return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0))).until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters);
    }
}    

