package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class Macros {
    // This sequence is responsible for ensuring the following goals:
    // 1. Get the elevator into coral station height
    // 2. Set elbow down
    // 3. Turn wrist to horizontal
    // 4. Run intake motors.

    private final ElevatorSubsystem elevator;
    private final ClawSubsystem claw;
    private final ElbowSubsystem elbow;

    public Macros( 
        ElevatorSubsystem elevator, 
        ClawSubsystem claw, 
        ElbowSubsystem elbow
    ) {
        this.elevator = elevator;
        this.elbow = elbow;
        this.claw = claw;
    }

    public Command getIntoCoralReceiveConfig() {
        return this.elevator.goToFloor(1)
        .andThen(claw.setWristAngle(EndEffectorConstants.wristHorizontalAngle))
        .until(() -> {return claw.getWristAngleRad() < (5 * Math.PI / 180.0);})
        .andThen(elbow.setElbowAngle(EndEffectorConstants.intakeAngle));
    }

    /*public Command configureForL4() {
        return this.elbow.setElbowAngle(EndEffectorConstants.restingAngle)
        .andThen(claw.setWristAngle(EndEffectorConstants.wristVerticalAngle))
        .andThen(claw.block())
        .until(() -> {
            return elbow.getElbowAngleRad() < (
                EndEffectorConstants.restingAngle + (5 * Math.PI / 180.0)
            );})
        .andThen(elevator.goToFloor(4))
        .until(() -> {
            return elevator.getHeightInches() > (ElevatorConstants.l4Setpoint - 3.0);
        })
        .andThen(elbow.setElbowAngle(EndEffectorConstants.fullyVertical));
    }*/

    public Command configureForL4() {
        return this.elbow.setElbowAngle(Units.degreesToRadians(60))
        .andThen(claw.setWristAngle(EndEffectorConstants.wristHorizontalAngle))
        .andThen(elevator.goToFloor(4))
        .andThen(elevator.block())
        .until(() -> {
            return elevator.getHeightInches() > ElevatorConstants.l4Setpoint - 5;
        })
        .andThen(claw.setWristAngle(EndEffectorConstants.wristVerticalAngle));
    }

    public Command ejectCoral() {
        return this.elbow.setElbowAngle(EndEffectorConstants.intakeAngle)
            .until(() -> {
                return elbow.getElbowAngleRad() < (
                    EndEffectorConstants.intakeAngle + (5 * Math.PI / 180.0)
                );
            })
            .andThen(claw.reverseIntake());
    }
}
