package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

/**
 * Subsystems: {@link ShooterSubsystem}
 */
public class ShooterHoodRPMCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private double rpm, hood;

    public ShooterHoodRPMCommand(ShooterSubsystem shooter, double targetRPM, double targetHood) {
        this.shooter = shooter;
        rpm = targetRPM;
        hood = targetHood;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setFlywheelRPM(rpm);
        shooter.setHoodAngle(hood);
    }
}
