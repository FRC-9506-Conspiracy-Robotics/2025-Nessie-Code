package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.Constants.ShooterConstants;

public class Macros {
    private final PivotSubsystem pivot;
    private final ShooterSubsystem shooter;

    public Macros(
        PivotSubsystem pivot,
        ShooterSubsystem shooter
    ) {
        this.pivot = pivot;
        this.shooter = shooter;
    }

    public Command shootMacro() {
        return pivot.setPivotPositionCommand(true)
            .andThen(shooter.setShooterSpeedCommand(ShooterConstants.shootRpm))
            .withName("ShootMacro");
    }

    public Command intakeMacro() {
        return Commands.parallel(
                shooter.setShooterSpeedCommand(ShooterConstants.intakeRpm),
                pivot.setPivotPositionCommand(false)
            )
            .withName("IntakeMacro");
    }
}
