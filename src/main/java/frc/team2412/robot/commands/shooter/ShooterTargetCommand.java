package frc.team2412.robot.commands.shooter;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;
import frc.team2412.robot.util.ShooterDataDistancePoint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.util.function.Supplier;

public class ShooterTargetCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final ShooterVisionSubsystem vision;
    private final Supplier<Vector2> feedforward;

    public ShooterTargetCommand(ShooterSubsystem shooter, ShooterVisionSubsystem vision, Supplier<Vector2> ff) {
        this.shooter = shooter;
        this.vision = vision;
        feedforward = ff;
        addRequirements(shooter);
    }

    public ShooterTargetCommand(ShooterSubsystem shooter, ShooterVisionSubsystem vision) {
        this(shooter, vision, () -> Vector2.ZERO);
    }

    @Override
    public void execute() {
        double distance = vision.getDistance() + shooter.getDistanceBias();
        double yaw = vision.getDistance() + shooter.getTurretAngleBias();
        double ff = feedforward.get()
                .rotateBy(Rotation2.fromDegrees(shooter.getTurretAngle() + ShooterConstants.TURRET_ANGLE_OFFSET)).x
                * ShooterConstants.TURRET_FF;
        ShooterDataDistancePoint shooterData = ShooterConstants.dataPoints.getInterpolated(distance);
        shooter.setHoodAngle(shooterData.getAngle());
        shooter.setFlywheelRPM(shooterData.getRPM());

        shooter.updateTurretAngle(yaw + ff);
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
