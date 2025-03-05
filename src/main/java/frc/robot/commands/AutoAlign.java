package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign {
    private final SwerveSubsystem swerveDrive;
    private final CameraSubsystem camera;

    public AutoAlign(SwerveSubsystem swerve, CameraSubsystem camera) {
        this.swerveDrive = swerve;
        this.camera = camera;
    }

    public Command autoAlign() {
        return Commands.run(() -> {

        }, this.swerveDrive, this.camera);
    }
    
}
