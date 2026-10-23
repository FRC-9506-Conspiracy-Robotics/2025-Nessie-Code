// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.Macros;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

// make macros ig

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);

    private final Timer driveTimer = new Timer();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final Macros macros = new Macros(
        pivot,
        shooter
    );

    //converts controller inputs to swerveinputstream type for field oriented
    SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> mDriverController.getLeftY() * -getSpeedModifier(),
        () -> mDriverController.getLeftX() * -getSpeedModifier()
    )
    .withControllerRotationAxis(() -> -mDriverController.getRightX() * getSpeedModifier())
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

        mDriverController.leftTrigger().whileTrue(macros.intakeMacro());
        mDriverController.rightTrigger().whileTrue(macros.shootMacro());

        mDriverController.leftStick().onTrue(drivebase.zero());
        
        NamedCommands.registerCommand("shoot", macros.shootMacro().withTimeout(1.5));
        NamedCommands.registerCommand("intake", macros.intakeMacro().withTimeout(2.0));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private double getSpeedModifier() {
        return 1.0;
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }
    }

    public Command resetDriveTimer() {
        return Commands.runOnce(() -> {
            driveTimer.reset();
            driveTimer.start();
        }, drivebase);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}