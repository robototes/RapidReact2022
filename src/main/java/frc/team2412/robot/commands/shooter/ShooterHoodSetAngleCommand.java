package frc.team2412.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterHoodSetAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier angleSupplier;

    /**
     * Subsystems: {@link ShooterSubsystem}
     */
    public ShooterHoodSetAngleCommand(DoubleSupplier angleSupplier) {
        this.shooter = ShooterSubsystem.instance;
        this.angleSupplier = angleSupplier;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double angle = angleSupplier.getAsDouble();
        shooter.setHoodAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
