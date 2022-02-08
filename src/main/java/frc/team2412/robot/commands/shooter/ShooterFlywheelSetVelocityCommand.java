package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterFlywheelSetVelocityCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double velocity;

    public ShooterFlywheelSetVelocityCommand(ShooterSubsystem shooter, double velocity) {
        this.shooter = shooter;
        this.velocity = velocity;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setFlywheelVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
