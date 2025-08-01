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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.Macros;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);

    private final Timer driveTimer = new Timer();
    private final Climber climber = new Climber();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final ElbowSubsystem elbow = new ElbowSubsystem();
    public final ClawSubsystem claw = new ClawSubsystem();
    private final Macros macros = new Macros(
        elevator,
        claw,
        elbow
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
        //NamedCommands.registerCommand("eject coral", this.claw.reverseIntake());
        //NamedCommands.registerCommand("set elbow to intake", this.elbow.setElbowAngle(EndEffectorConstants.intakeAngle));

        NamedCommands.registerCommand("l1 eject", macros.l1eject());

        configureBindings();
        claw.resetZero();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private double getSpeedModifier() {
        if (elevator.getHeightInches() > (ElevatorConstants.intakeSetpoint)) {
            double slope = (0.25 - 1.0) / (ElevatorConstants.l4Setpoint - ElevatorConstants.intakeSetpoint);
            double intercept = 1.0 - slope * ElevatorConstants.intakeSetpoint;
            return slope * elevator.getHeightInches() + intercept;
        }
        return 1.0;
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocity);
        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        mDriverController.y().onTrue(macros.configureForL4());
        mDriverController.start().onTrue(macros.getIntoCoralReceiveConfig()); 
        mDriverController.rightBumper().whileTrue(claw.runIntake());
        mDriverController.x().whileTrue(claw.reverseIntake());


        mDriverController.leftBumper().onTrue(claw.setWristAngle(EndEffectorConstants.wristHorizontalAngle));
        mDriverController.a().onTrue(claw.setWristAngle(EndEffectorConstants.wristVerticalAngle));
        mDriverController.povRight().onTrue(elbow.goToStage(1));
        mDriverController.povLeft().onTrue(elbow.goToStage(2));
        mDriverController.povUp().onTrue(elevator.incrementFloor());
        mDriverController.povDown().onTrue(elevator.decrementFloor());

        mDriverController.leftStick().onTrue(drivebase.zero());
        mDriverController.back().onTrue(elbow.zeroElbow());
        //mDriverController.rightStick().onTrue(elbow.fixSetpoint());

        mDriverController.rightTrigger(0.5).onTrue(macros.getIntoFloorIntakeConfig());
        mDriverController.b().whileTrue(climber.deploy());
        mDriverController.leftTrigger(0.5).whileTrue(climber.climb());
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