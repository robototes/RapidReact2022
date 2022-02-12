package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterResetEncodersCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public ShooterResetEncodersCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetHoodEncoder();
        shooter.resetTurretEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
