package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class PostClimbSubsystem extends SubsystemBase implements Loggable {

    private Solenoid blockSolenoid;
    private DoubleSolenoid firingSolenoid;

    public PostClimbSubsystem() {
        blockSolenoid = new Solenoid(PNEUMATIC_HUB, PneumaticsModuleType.REVPH, POST_CLIMB_BLOCKING_SOLENOID);
        firingSolenoid = new DoubleSolenoid(PNEUMATIC_HUB, PneumaticsModuleType.REVPH, POST_CLIMB_FIRING_SOLENOID_FIRE,
                POST_CLIMB_FIRING_SOLENOID_CLOSE);
        block();
        disarmSolenoid();
    }

    public void removeBlock() {
        blockSolenoid.set(false);
    }

    public void block() {
        blockSolenoid.set(true);
    }

    public void armSolenoid() {
        firingSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void disarmSolenoid() {
        firingSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

}
