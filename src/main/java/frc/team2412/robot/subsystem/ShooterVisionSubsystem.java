package frc.team2412.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class ShooterVisionSubsystem extends SubsystemBase {

    private PhotonCamera limelight;

    public ShooterVisionSubsystem(PhotonCamera limelight) {
        this.limelight = limelight;
    }

    public double getYaw(){
        return limelight.getLatestResult().getBestTarget().getYaw();
    }



    
}
