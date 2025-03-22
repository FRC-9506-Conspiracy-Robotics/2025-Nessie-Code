// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
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
    final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);

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
//    private final RGBSubsystem rgb = new RGBSubsystem();

    //converts controller inputs to swerveinputstream type for field oriented
    SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> mDriverController.getLeftY() * -getSpeedModifier(),
        () -> mDriverController.getLeftX() * -getSpeedModifier()
    )
    .withControllerRotationAxis(() -> mDriverController.getRightX() * getSpeedModifier())
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

        mDriverController.rightTrigger(0.5).onTrue(macros.getIntoCoralReceiveConfig());
        mDriverController.b().toggleOnTrue(climber.deploy());
        mDriverController.leftTrigger().toggleOnTrue(climber.climb());
    }

    public Command getAutonomousCommand() {
        return  drivebase.getAutonomousCommand("New Auto")
        .andThen(elbow.block())
        .withTimeout(2.0)
        .andThen(elbow.setElbowAngle(EndEffectorConstants.intakeAngle))
        .andThen(elbow.block())
        .until(()->{
            return elbow.getElbowAngleRad() < (
                EndEffectorConstants.intakeAngle + (5 * Math.PI / 180.0)
            );})
        .andThen(claw.reverseIntake());
    }
}