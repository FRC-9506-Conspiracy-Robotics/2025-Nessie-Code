/**/package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGBSubsystem extends SubsystemBase {
    private static final int port = 9;
    private static final int length = 60;

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public RGBSubsystem() {
        this.led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.start();
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    public Command staticColor() {
        LEDPattern color = LEDPattern.solid(Color.kRed);
        return run(() -> color.applyTo(ledBuffer));
    }

    public Command pulseColor() {
        LEDPattern basecolor = LEDPattern.solid(Color.kRed);
        LEDPattern pulsecolor = basecolor.breathe(Second.of(10));
        return run(() -> pulsecolor.applyTo(ledBuffer));
    }

    public Command staticRainbow() {
        LEDPattern rainbow = LEDPattern.rainbow(255, 255);
        System.out.println("rainbow");
        return run(() -> rainbow.applyTo(ledBuffer));
    }

    public Command customPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer));
    }
}