package frc.team2412.robot.subsystem;

import static org.photonvision.common.hardware.VisionLEDMode.kOff;
import static org.photonvision.common.hardware.VisionLEDMode.kOn;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterVisionSubsystem extends SubsystemBase {

    private PhotonCamera limelight;

    private PhotonTrackedTarget target;

    public ShooterVisionSubsystem(PhotonCamera limelight) {
        this.limelight = limelight;
    }

    @Override
    public void periodic() {
        if (hasTarget()) {
            target = limelight.getLatestResult().getBestTarget();
        }
    }

    public boolean hasTarget() {
        return limelight.getLatestResult().hasTargets();
    }

    // x-axis
    public double getYaw() {
        return target.getYaw();
    }

    // y-axis
    public double getPitch() {
        return target.getPitch();
    }

    public void setLedOn() {
        limelight.setLED(kOn);
    }

    public void setLedOff() {
        limelight.setLED(kOff);
    }

}
