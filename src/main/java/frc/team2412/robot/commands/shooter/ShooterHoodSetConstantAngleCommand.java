package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

/**
 * Subsystems: {@link ShooterSubsystem}
 */
public class ShooterHoodSetConstantAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double angle;

    public ShooterHoodSetConstantAngleCommand(double angle) {
        this.shooter = ShooterSubsystem.instance;
        this.angle = angle;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setHoodAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return shooter.isHoodAtAngle(angle);
    }
}
