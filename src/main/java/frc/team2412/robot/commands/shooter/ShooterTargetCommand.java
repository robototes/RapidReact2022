package frc.team2412.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.util.ShooterDataDistancePoint;

public class ShooterTargetCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier distanceSupplier;
    private final DoubleSupplier yawSupplier;

    public ShooterTargetCommand(ShooterSubsystem shooter, DoubleSupplier distanceSupplier, DoubleSupplier yawSupplier) {
        this.shooter = shooter;
        this.distanceSupplier = distanceSupplier;
        this.yawSupplier = yawSupplier;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distance = distanceSupplier.getAsDouble();
        double yaw = yawSupplier.getAsDouble();
        ShooterDataDistancePoint shooterData = ShooterSubsystem.dataPoints.getInterpolated(distance);
        shooter.hoodMotorSetAngle(shooterData.getAngle());
        shooter.setFlywheelVelocity(shooterData.getPower());
        shooter.updateTurretAngle(yaw);
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
