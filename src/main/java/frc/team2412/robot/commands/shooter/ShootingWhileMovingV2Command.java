package frc.team2412.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants.*;
import frc.team2412.robot.subsystem.TargetLocalizer;
import frc.team2412.robot.util.ShooterDataDistancePoint;
import frc.team2412.robot.util.WPILIBInterpolatingTreeMap;

public class ShootingWhileMovingV2Command extends CommandBase {
    private final ShooterSubsystem shooter;
    private final TargetLocalizer localizer;

    private static WPILIBInterpolatingTreeMap<Number, Double> timeOfFlight = new WPILIBInterpolatingTreeMap<Number, Double>();

    static {
        timeOfFlight.put(80, 1.0);
        timeOfFlight.put(100, 1.25);
        timeOfFlight.put(120, 1.5);
        timeOfFlight.put(140, 1.75);
        timeOfFlight.put(160, 2.0);
    }

    public ShootingWhileMovingV2Command(ShooterSubsystem shooter, TargetLocalizer localizer) {
        this.shooter = shooter;
        this.localizer = localizer;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (!localizer.hasTarget()) {
            return;
        }

        // get target location
        // get current shot time
        // Adjust target location by that shot time
        // point at that and shoot at that new distance

        Translation2d targetPosition = localizer.getTargetPosition();

        double actualDistance = targetPosition.getNorm();

        double currentShotTime = timeOfFlight.get(actualDistance);

        Translation2d theoreticalTargetPosition = localizer.getAdjustedTargetPosition(currentShotTime);

        double theoreticalDistance = theoreticalTargetPosition.getNorm();

        ShooterDataDistancePoint rpmHoodValues = DATA_POINTS.getInterpolated(theoreticalDistance + shooter.getDistanceBias());

        shooter.setFlywheelRPM(rpmHoodValues.getRPM());
        shooter.setHoodAngle(rpmHoodValues.getAngle());

        double turretRadianChange = Math.atan2(theoreticalTargetPosition.getY(), theoreticalTargetPosition.getX());
        double turretDegreeChange = Units.radiansToDegrees(turretRadianChange);
        
        shooter.updateTurretAngle(turretDegreeChange);
        // Don't do wrap around yet since we don't have 360
    }

}
