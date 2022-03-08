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
    private final BooleanSupplier turret;

    /**
     * Subsystems: {@link ShooterSubsystem}, {@link ShooterVisionSubsystem}
     */
    public ShooterTargetCommand() {
        this(() -> false);
    }

    public ShooterTargetCommand(BooleanSupplier turretButton) {
        this.shooter = ShooterSubsystem.instance;
        this.vision = ShooterVisionSubsystem.instance;
        addRequirements(shooter);
        turret = turretButton;
    }

    @Override
    public void execute() {
        if (!shooter.turretWorking)
            return;

        double distance = vision.hasTarget() ? vision.getDistance() + shooter.getDistanceBias() : 0;

        double yaw = vision.getYaw() + shooter.getTurretAngleBias();
        if (ShooterConstants.dataPoints != null) {
            ShooterDataDistancePoint shooterData = ShooterConstants.dataPoints.getInterpolated(distance);
            shooter.setHoodAngle(shooterData.getAngle());
            shooter.setFlywheelRPM(shooterData.getRPM());
        }
        if (turret.getAsBoolean()) {
            shooter.updateTurretAngle(yaw);
        } else
            shooter.setTurretAngle(0);

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
