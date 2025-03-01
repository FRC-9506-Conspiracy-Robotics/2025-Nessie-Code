package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class tempdpadSubsystem extends SubsystemBase{
    public Command printdpad(double position) {
        return runOnce(() -> System.out.println("dpad: " + position));
    }
}
