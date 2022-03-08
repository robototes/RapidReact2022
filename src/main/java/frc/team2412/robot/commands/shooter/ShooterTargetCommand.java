package frc.team2412.robot.commands.shooter;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;
import frc.team2412.robot.util.ShooterDataDistancePoint;

public class ShooterTargetCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final TargetLocalizer localizer;
    private final BooleanSupplier turret;

    public ShooterTargetCommand(ShooterSubsystem shooter, TargetLocalizer vision) {
        this(shooter, vision, () -> false);
    }

    public ShooterTargetCommand(ShooterSubsystem shooter, TargetLocalizer vision, BooleanSupplier turretButton) {
        this.shooter = shooter;
        localizer = vision;
        turret = turretButton;
        addRequirements(shooter);
    }

    double turretAngle = 0;

    @Override
    public void execute() {
        if (!shooter.turretWorking)
            return;
        if (ShooterConstants.dataPoints != null) {
            ShooterDataDistancePoint shooterData = ShooterConstants.dataPoints
                    .getInterpolated(localizer.getAdjustedDistance());
            shooter.setHoodAngle(shooterData.getAngle());
            shooter.setFlywheelRPM(shooterData.getRPM());
        }
        if (turret.getAsBoolean()) {
            turretAngle = shooter.getTurretAngle() + localizer.getYaw();
            shooter.setTurretAngle(turretAngle + localizer.yawAdjustment());
        } else {
            shooter.setTurretAngle(0);
            turretAngle = 0;
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
