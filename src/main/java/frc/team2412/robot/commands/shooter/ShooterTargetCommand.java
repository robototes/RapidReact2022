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
    private final BooleanSupplier turretEnable;

    public ShooterTargetCommand(ShooterSubsystem shooter, TargetLocalizer localizer) {
        this(shooter, localizer, () -> false);
    }

    public ShooterTargetCommand(ShooterSubsystem shooter, TargetLocalizer localizer, BooleanSupplier turretButton) {
        this.shooter = shooter;
        this.localizer = localizer;
        turretEnable = turretButton;
        addRequirements(shooter);
    }

    double turretAngle = 0;

    @Override
    public void execute() {
        if (ShooterConstants.dataPoints != null) {
            ShooterDataDistancePoint shooterData = ShooterConstants.dataPoints
                    .getInterpolated(localizer.getAdjustedDistance());
            shooter.setHoodAngle(shooterData.getAngle());
            shooter.setFlywheelRPM(shooterData.getRPM());
        }
        turretAngle = turretEnable.getAsBoolean() ? turretAngle + localizer.getYaw() : 0;
        shooter.setTurretAngle(turretAngle + localizer.yawAdjustment());

    }
}
