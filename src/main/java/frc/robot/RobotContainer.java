// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.tempdpadSubsystem;

import java.io.File;

import org.ejml.equation.Variable;

import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final tempdpadSubsystem dpad = new tempdpadSubsystem();

    //converts controller inputs to swerveinputstream type for field oriented
    SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> mDriverController.getLeftY() * -1,
    () -> mDriverController.getLeftX() * -1)
    .withControllerRotationAxis(mDriverController::getRightX)
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
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocity);
        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        mDriverController.povCenter().onTrue(dpad.printdpad(-1));
        mDriverController.povUp().onTrue(dpad.printdpad(0));
        mDriverController.povRight().onTrue(dpad.printdpad(90));
        mDriverController.povDown().onTrue(dpad.printdpad(180));
        mDriverController.povLeft().onTrue(dpad.printdpad(270));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}