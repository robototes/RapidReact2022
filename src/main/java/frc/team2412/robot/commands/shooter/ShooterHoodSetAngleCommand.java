package frc.team2412.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class ShooterHoodSetAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier angleSupplier;

    public ShooterHoodSetAngleCommand(ShooterSubsystem shooter, double angle) {
        this.shooter = shooter;
        this.angleSupplier = () -> angle;
        addRequirements(shooter);
    }

    public ShooterHoodSetAngleCommand(ShooterSubsystem shooter, ShooterVisionSubsystem vision) {
        this.shooter = shooter;
        this.angleSupplier = null; // TODO: Interface with ShooterVisionSubsystem
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double angle = angleSupplier.getAsDouble();
        shooter.hoodMotorSetAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
