package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterIdleCommand extends CommandBase {
    public ShooterIdleCommand(ShooterSubsystem shooter) {
        addRequirements(shooter);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
