package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterTurretSetAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double angle;

    public ShooterTurretSetAngleCommand(ShooterSubsystem shooter, double angle) {
        this.shooter = shooter;
        this.angle = angle;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setTurretAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ShooterTurretSetAngleCommand finished");
    }

    @Override
    public boolean isFinished() {
        return shooter.isTurretAtAngle(angle);
    }
}
