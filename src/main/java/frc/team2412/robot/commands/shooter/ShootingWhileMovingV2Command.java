package frc.team2412.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants.*;
import frc.team2412.robot.subsystem.TargetLocalizer;
import frc.team2412.robot.util.ShooterDataDistancePoint;

public class ShootingWhileMovingV2Command extends CommandBase {
    private final ShooterSubsystem shooter;
    private final TargetLocalizer localizer;

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

        // get target distance
        // get current shot time
        // Adjust target location by that shot time
        // point at that and shoot at that new distance


        double actualDistance = localizer.getDistance();

        double currentShotTime = DATA_POINTS.getInterpolated(actualDistance).getTimeOfFlight();

        Translation2d theoreticalTargetPosition = localizer.getAdjustedTargetPosition(currentShotTime);

        double theoreticalDistance = theoreticalTargetPosition.getNorm();

        ShooterDataDistancePoint rpmHoodValues = DATA_POINTS
                .getInterpolated(theoreticalDistance + shooter.getDistanceBias());

        shooter.setFlywheelRPM(rpmHoodValues.getRPM());
        shooter.setHoodAngle(rpmHoodValues.getAngle());

        double turretRadianChange = Math.atan2(theoreticalTargetPosition.getY(), theoreticalTargetPosition.getX());
        double turretDegreeChange = Units.radiansToDegrees(turretRadianChange);

        if(turretDegreeChange == 180){
            turretDegreeChange = 0;
        }

        shooter.updateTurretAngle(turretDegreeChange);
        // Don't do wrap around yet since we don't have 360
    }

}
