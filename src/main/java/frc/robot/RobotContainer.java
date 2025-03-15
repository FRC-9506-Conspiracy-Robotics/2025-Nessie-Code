// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.fasterxml.jackson.core.json.WriterBasedJsonGenerator;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ReceiveCoralConfiguration;
//import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);
    private double speedModifier = 1.0;

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final ElbowSubsystem elbow = new ElbowSubsystem();
    public final ClawSubsystem claw = new ClawSubsystem();
    private final ReceiveCoralConfiguration receiveCoralCmd = new ReceiveCoralConfiguration(
        elevator,
        claw,
        elbow
    );
//    private final RGBSubsystem rgb = new RGBSubsystem();

    //converts controller inputs to swerveinputstream type for field oriented
    SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> mDriverController.getLeftY() * -speedModifier,
        () -> mDriverController.getLeftX() * -speedModifier
    )
    .withControllerRotationAxis(() -> mDriverController.getRightX() * speedModifier)
    .deadband(DriverConstants.kDeadband)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

    //copies previous stream and converts to robot oriented
    SwerveInputStream driveRobotOriented = 
    driveAngularVelocity.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);

    //controls for keyboard
    SwerveInputStream driveAngularVelocityKeyboard = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> -mDriverController.getLeftY(),
    () -> -mDriverController.getLeftX())
    .withControllerRotationAxis(
        () -> mDriverController.getRawAxis(2)
    )
    .deadband(DriverConstants.kDeadband)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

    public RobotContainer() {
        configureBindings();
        elbow.resetZero();
        claw.resetZero();
    }

    private Command toggleSpeedModifier() {
        return Commands.runOnce(
            () -> {
                if (speedModifier == 1.0) {
                    speedModifier = 0.5;
                } else {
                    speedModifier = 1.0;
                }
            }, 
        drivebase);
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocity);
        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        mDriverController.y().toggleOnTrue(elevator.goToCurrentFloor());
        mDriverController.y().toggleOnFalse(elevator.goToBottom());
        mDriverController.start().onTrue(elbow.resetZero().andThen(claw.resetZero())); 
        mDriverController.a().whileTrue(claw.runIntake());
        mDriverController.x().whileTrue(claw.reverseIntake());

        mDriverController.leftBumper().onTrue(claw.setWristAngle(EndEffectorConstants.wristHorizontalAngle));
        mDriverController.rightBumper().onTrue(claw.setWristAngle(EndEffectorConstants.wristVerticalAngle));
        mDriverController.povRight().onTrue(elbow.setElbowAngle(EndEffectorConstants.intakeAngle));
        mDriverController.povLeft().onTrue(elbow.setElbowAngle(EndEffectorConstants.restingAngle));
        mDriverController.povUp().onTrue(elevator.incrementFloor());
        mDriverController.povDown().onTrue(elevator.decrementFloor());

        mDriverController.leftStick().onTrue(drivebase.zero());
        mDriverController.rightStick().onTrue(elbow.fixSetpoint());

        mDriverController.leftTrigger(0.5).onTrue(toggleSpeedModifier());
        mDriverController.rightTrigger(0.5).onTrue(receiveCoralCmd.getIntoCoralReceiveConfig());
    }

    public Command getAutonomousCommand() {
        return  drivebase.getAutonomousCommand("New Auto");
    }
}