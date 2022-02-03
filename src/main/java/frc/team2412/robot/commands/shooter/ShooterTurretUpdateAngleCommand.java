package frc.team2412.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterTurretUpdateAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier angleSupplier;

    public ShooterTurretUpdateAngleCommand(ShooterSubsystem shooter, DoubleSupplier angleSupplier) {
        this.shooter = shooter;
        this.angleSupplier = angleSupplier;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double angle = angleSupplier.getAsDouble();
        shooter.updateTurretAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
