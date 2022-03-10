package frc.team2412.robot.commands.shooter;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;
import frc.team2412.robot.util.ShooterDataDistancePoint;

public class ShooterTargetCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final ShooterVisionSubsystem vision;
    private final BooleanSupplier turretEnable;

    public ShooterTargetCommand(ShooterSubsystem shooter, ShooterVisionSubsystem vision) {
        this(shooter, vision, () -> false);
    }

    public ShooterTargetCommand(ShooterSubsystem shooter, ShooterVisionSubsystem vision, BooleanSupplier turretEnable) {
        this.shooter = shooter;
        this.vision = vision;
        this.turretEnable = turretEnable;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (!shooter.enableTurret) {
            return;
        }

        double distance = 0, yaw = 0;

        if (vision.hasTarget()) {
            distance = vision.getDistance() + shooter.getDistanceBias();
            yaw = vision.getYaw() + shooter.getTurretAngleBias();
        }

        if (ShooterConstants.dataPoints != null) {
            ShooterDataDistancePoint shooterData = ShooterConstants.dataPoints.getInterpolated(distance);
            shooter.setHoodAngle(shooterData.getAngle());
            shooter.setFlywheelRPM(shooterData.getRPM());
        }

        if (turretEnable.getAsBoolean()) {
            shooter.updateTurretAngle(yaw);
        } else {
            shooter.setTurretAngle(0);
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
