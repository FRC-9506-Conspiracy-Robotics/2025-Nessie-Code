package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
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
        return Commands.runEnd(
            () -> {
                PhotonTrackedTarget target = this.camera.getBestTarget();
                if (target == null) {
                    return;
                }

                double targetYaw = target.getYaw();
                double turn = (
                    -1.0 * targetYaw * 
                    Constants.VisionConstants.visionTurnkP * 
                    Constants.SwerveConstants.maxAngularSpeedDegPerSec * Math.PI / 180.0
                );
                this.swerveDrive.drive(
                    new Translation2d(0.0, 0.0),
                    turn,
                    false
                );
            }, 
            () -> {
                this.swerveDrive.drive(
                    new Translation2d(0.0, 0.0), 
                    0.0, 
                    false
                );
            },
            this.swerveDrive, this.camera
        );
    }
    
}
