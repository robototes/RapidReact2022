package frc.team2412.robot.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory squarePathAuto;
    private Trajectory starPathAuto;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        squarePathAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(-24, 0))
                        .lineTo(new Vector2(-24, 24))
                        .lineTo(new Vector2(0, 24))
                        .lineTo(new Vector2(0, 0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        starPathAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(12, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(24, 12))
                        .lineTo(new Vector2(36, 0))
                        .lineTo(new Vector2(30, 18))
                        .lineTo(new Vector2(42, 30))
                        .lineTo(new Vector2(30, 30))
                        .lineTo(new Vector2(24, 42), Rotation2.fromDegrees(90))
                        .lineTo(new Vector2(18, 30), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(6, 30))
                        .lineTo(new Vector2(18, 18))
                        .lineTo(new Vector2(12, 0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getSquarePathAuto() {
        return squarePathAuto;
    }

    public Trajectory getStarPathAuto() {
        return starPathAuto;
    }
}
