package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import comp.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanId;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraSubsystem extends SubsystemBase {
        private final PhotonCamera camera = new PhotonCamera("HD_Camera");

        public CameraSubsystem() {
            
        }
        
        
        public Command runAtAllMatch() {

            return run(() -> {
        
                    var result = camera.getLatestResult();
                    boolean hasTargets = result.hasTargets();

                    if (!hasTargets) {
                    
                        return;
                    }

                    List<PhotonTrackedTarget> targets = result.getTargets();
                    PhotonTrackedTarget bestTarget = result.getBestTarget();

                    
                    System.out.println("This is X: " + bestTarget.getBestCameraToTarget().getX());
                    System.out.println("This is Y: " + bestTarget.getBestCameraToTarget().getY());
                    System.out.println("This is z: " + bestTarget.getBestCameraToTarget().getZ());

                    System.out.println("This is yaw: " + bestTarget.getBestCameraToTarget().getRotation().getZ() * (180 / Math.PI));

                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                    }
            }
            );
        }
}