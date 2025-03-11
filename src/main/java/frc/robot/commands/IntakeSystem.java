package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeSystem {
    // This sequence is responsible for ensuring the following goals:
    // 1. Get the elevator into coral station height
    // 2. Set elbow down
    // 3. Turn wrist to horizontal
    // 4. Run intake motors.

    private ElevatorSubsystem m_elevator;
    private ClawSubsystem m_claw;
    private ElbowSubsystem m_elbow;
    private SwerveSubsystem m_swerve;

    public IntakeSystem( 
        ElevatorSubsystem elevator, 
        ClawSubsystem claw, 
        ElbowSubsystem elbow,
        SwerveSubsystem swerve
    ) {
        m_elevator = elevator;
        m_elbow = elbow;
        m_claw = claw;
        m_swerve = swerve;
    }

    public Command recieveCoral() {
        return (Commands.parallel(m_elbow.goToIntakePos(), m_claw.setWristAngle(EndEffectorConstants.wristHorizontalAngle))
            .andThen(m_elevator.setFloor(4)));
    }
    
}
