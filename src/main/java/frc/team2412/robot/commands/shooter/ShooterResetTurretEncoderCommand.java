package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterResetTurretEncoderCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public ShooterResetTurretEncoderCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.resetTurretEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}