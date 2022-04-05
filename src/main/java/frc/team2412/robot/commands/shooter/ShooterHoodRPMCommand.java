package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterHoodRPMCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double rpm;
    private final double hoodAngle;

    public ShooterHoodRPMCommand(ShooterSubsystem shooter, double targetRPM, double targetHood) {
        this.shooter = shooter;
        this.rpm = targetRPM;
        this.hoodAngle = targetHood;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (rpm != 0) {
            shooter.setFlywheelRPM(rpm);
        } else {
            shooter.stopFlywheel();
        }

        shooter.setHoodAngle(hoodAngle);
    }

}
