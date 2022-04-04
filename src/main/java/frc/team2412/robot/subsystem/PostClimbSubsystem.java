package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class PostClimbSubsystem extends SubsystemBase implements Loggable {

    private Solenoid upwardsSolenoid;
    private Solenoid downwardsSolenoid;

    public PostClimbSubsystem() {
        upwardsSolenoid = new Solenoid(PNEUMATIC_HUB, PneumaticsModuleType.REVPH, POST_CLIMB_SOLENOID_UPWARDS);
        downwardsSolenoid = new Solenoid(PNEUMATIC_HUB, PneumaticsModuleType.REVPH, POST_CLIMB_SOLENOID_DOWNWARDS);
    }

    public void goingUp() {
        upwardsSolenoid.set(true);
    }

    public void goingDown() {
        downwardsSolenoid.set(true);
    }

}
