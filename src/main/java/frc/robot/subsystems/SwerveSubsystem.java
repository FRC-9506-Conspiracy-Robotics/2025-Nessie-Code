package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Vision.Cameras;
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
    private Vision vision;
    public SwerveSubsystem(File directory) {
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        //Attempts to initialize swerve drive using the given filepath throws error if files cannot be found
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                Constants.SwerveConstants.maxSpeed,
                startingPose
            );
            } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); 
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //corrects for skew that occurs as angular velocity increases, tune this
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); //enable to resync encoders when swerve is not moving
        setupPhotonVision();
        swerveDrive.stopOdometryThread();
        setupPathPlanner();
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyro));
    }

    public void setupPhotonVision() {
        vision = new Vision(swerveDrive::getPose, swerveDrive.field);
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
        swerveDrive.updateOdometry();
        vision.updatePoseEstimation(swerveDrive);
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

    public Command aimAtTarget(Cameras camera) {

        return run(() -> {
            Optional<PhotonPipelineResult> resultO = camera.getBestResult();
            if (resultO.isPresent()) {
                var result = resultO.get();
                if (result.hasTargets()) {
                    drive(getTargetSpeeds(0,
                                0,
                                Rotation2d.fromDegrees(result.getBestTarget()
                                                             .getYaw()))); // Not sure if this will work, more math may be required.
                }
            }
        });
    }

    //drives to point on field using pathplanner pathfinding
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(),
            6.0,
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(810)
        );
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0)
        );
    }

    /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
        throws IOException, ParseException {
            SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            swerveDrive.getMaximumChassisAngularVelocity());
            AtomicReference<SwerveSetpoint> prevSetpoint
            = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                   swerveDrive.getStates(),
                                                   DriveFeedforwards.zeros(swerveDrive.getModules().length)));
            AtomicReference<Double> previousTime = new AtomicReference<>();

            return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                    () -> {
                      double newTime = Timer.getFPGATimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                        newSetpoint.moduleStates(),
                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);

                    });
        }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
            });
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
    return Commands.none();
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

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void resetOdometry(Pose2d intialHolonomicPose) {
        swerveDrive.resetOdometry(intialHolonomicPose);
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), getHeading().getRadians(), SwerveConstants.maxSpeed);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
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

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(
            () -> {
                swerveDrive.driveFieldOriented(velocity.get());
            }
        );
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
      //Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
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

