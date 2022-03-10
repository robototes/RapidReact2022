package frc.team2412.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterUpdateHoodRPMCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier flywheelRPMsupplier;
    private final DoubleSupplier hoodAngleSupplier;

    public ShooterUpdateHoodRPMCommand(ShooterSubsystem shooter) {
        this(shooter, shooter::getFlywheelTestRPM, shooter::getHoodTestAngle);
    }

    public ShooterUpdateHoodRPMCommand(ShooterSubsystem shooter, DoubleSupplier flywheelRPMsupplier,
            DoubleSupplier hoodAngleSupplier) {
        this.shooter = shooter;
        this.flywheelRPMsupplier = flywheelRPMsupplier;
        this.hoodAngleSupplier = hoodAngleSupplier;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setFlywheelRPM(flywheelRPMsupplier.getAsDouble());
        shooter.setHoodAngle(hoodAngleSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
