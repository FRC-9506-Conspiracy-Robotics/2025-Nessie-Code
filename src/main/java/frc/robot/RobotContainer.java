// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.ReceiveCoralConfiguration;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final DigitalInput limitswitch = new DigitalInput(0);
    private final Trigger limitTrigger = new Trigger(limitswitch::get);
    private final ElbowSubsystem elbow = new ElbowSubsystem();
    private final ClawSubsystem claw = new ClawSubsystem();
    private final ReceiveCoralConfiguration receiveCoralSequence = new ReceiveCoralConfiguration(
        elevator, claw, elbow);

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

        // TODO: @Alberto
        // Make sure to change the elevator commands from "onTrue" to "toggleOnTrue"
        mDriverController.povUp().onTrue(elevator.goUpOneFloor().andThen(elevator.holdPosition(false)));
        mDriverController.povDown().onTrue(elevator.goDownOneFloor().andThen(elevator.holdPosition(false)));  
        mDriverController.povLeft().onTrue(elbow.setElbowAngle(.5));
        mDriverController.povRight().onTrue(elbow.setElbowAngle(0));  

        //mDriverController.x().onTrue(elevator.holdPosition(true));
        //mDriverController.y().onTrue(elevator.stopElevator());
        limitTrigger.onTrue(elevator.stopElevator());
        //mDriverController.a().onTrue(elbow.setElbowAngle(1));
        //mDriverController.b().onTrue(elbow.setElbowAngle(0));
        //mDriverController.x().onTrue(claw.holdClaw());
        mDriverController.leftBumper().onTrue(claw.setWristAngle(1.4));
        mDriverController.rightBumper().onTrue(claw.setWristAngle(-0.017));
        
        // TODO: @Alberto
        // Map the intake commands to buttons here. You have 3 commands: 
        // runIntake, stopIntake, and reverse the intake.
        mDriverController.a().whileTrue(claw.runIntake());
        mDriverController.x().whileTrue(claw.reverseIntake());
        mDriverController.b().onTrue(claw.stopIntake());

        // TODO: @Alberto, figure out if this command sequence works the way it's supposed to.
        //mDriverController.y().toggleOnTrue(receiveCoralSequence.getIntoCoralReceiveConfig());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}