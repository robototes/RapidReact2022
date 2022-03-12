package frc.team2412.robot.subsystem;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.team2412.robot.Hardware.HardwareConstants.PNEUMATIC_HUB;

public class PneumaticHubSubsystem extends SubsystemBase implements Loggable {

    private final PneumaticHub pneumaticHub;
    private static final double MIN_PRESSURE = 100;
    private static final double MAX_PRESSURE = 120;

    public PneumaticHubSubsystem() {
        pneumaticHub = new PneumaticHub(PNEUMATIC_HUB);
        pneumaticHub.enableCompressorAnalog(MIN_PRESSURE, MAX_PRESSURE);
    }

    @Log(name = "Pressure")
    public double getPressure() {
        return pneumaticHub.getPressure(0);
    }
}
