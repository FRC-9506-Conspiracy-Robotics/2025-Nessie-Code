package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ReceiveCoralConfiguration {
    // This sequence is responsible for ensuring the following goals:
    // 1. Get the elevator into coral station height
    // 2. Set elbow down
    // 3. Turn wrist to horizontal
    // 4. Run intake motors.

    private final ElevatorSubsystem elevator;
    private final ClawSubsystem claw;
    private final ElbowSubsystem elbow;

    public ReceiveCoralConfiguration( 
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
        .andThen(
            Commands.parallel(
                this.elevator.holdPosition(false),
                this.elbow.setElbowAngle(0.0),
                this.claw.setWristAngle(0.0),
                this.claw.runIntake()
            )
        );
    }
    
}
