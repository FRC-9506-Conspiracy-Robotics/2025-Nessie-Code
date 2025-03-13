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
//import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandXboxController mDriverController = new CommandXboxController(DriverConstants.kDriverControllerPort);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    //private final ElbowSubsystem elbow = new ElbowSubsystem();
    //private final ClawSubsystem claw = new ClawSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
//    private final RGBSubsystem rgb = new RGBSubsystem();

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
        //elbow.setZero();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocity);
        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        mDriverController.povUp().onTrue(elevator.incrementFloor());
        mDriverController.povDown().onTrue(elevator.decrementFloor());
        mDriverController.y().toggleOnTrue(elevator.goToCurrentFloor());
        mDriverController.y().toggleOnFalse(elevator.goToBottom());
        //mDriverController.a().onTrue(
            //claw.setWristAngle(EndEffectorConstants.wristVerticalAngle));
             //.alongWith(Commands.waitSeconds(2.0).andThen(elbow.setElbowAngle(EndEffectorConstants.intakeAngle))));
        //mDriverController.b().onTrue(claw.setWristAngle(EndEffectorConstants.wristHorizontalAngle));
            //.alongWith(claw.setWristAngle((EndEffectorConstants.wristHorizontalAngle))));
            
        //mDriverController.a().onTrue(elbow.resetZero().andThen(claw.resetZero())); 

        mDriverController.leftBumper().onTrue(wrist.setTargetRad(EndEffectorConstants.wristHorizontalAngle));
        mDriverController.rightBumper().onTrue(wrist.setTargetRad(EndEffectorConstants.wristVerticalAngle));
        //mDriverController.povLeft().onTrue(elbow.setElbowAngle(EndEffectorConstants.intakeAngle));
        //mDriverController.povRight().onTrue(elbow.setElbowAngle(EndEffectorConstants.restingAngle));
        //mDriverController.b().toggleOnTrue(claw.runIntake());
        //mDriverController.x().toggleOnTrue(claw.reverseIntake());



        /**mDriverController.a()
            .toggleOnTrue(elbow.setElbowAngle(EndEffectorConstants.intakeAngle)
            .andThen(elevator.goToFloor(4))
            .alongWith(claw.setWristAngle(EndEffectorConstants.wristHorizontalAngle))
            // .alongWith(claw.runIntake())
        );
        mDriverController.a()
            .toggleOnFalse(claw.stopIntake()
            .alongWith(elbow.setElbowAngle(EndEffectorConstants.clearanceAngle))
            // .alongWith(claw.setWristAngle(EndEffectorConstants.wristVerticalAngle))
            .andThen(elevator.goToBottom())
        );*/
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}