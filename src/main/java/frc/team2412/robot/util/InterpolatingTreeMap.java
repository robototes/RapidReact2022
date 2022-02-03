package frc.team2412.robot.util;

import java.util.TreeMap;

public class InterpolatingTreeMap extends TreeMap<Double, ShooterDataDistancePoint> {

    public ShooterDataDistancePoint getInterpolated(Double key) {
        ShooterDataDistancePoint value = get(key);

        // Check if we have exact value
        if (value != null) {
            return value;
        }

        // Get nearest keys
        Double floor = floorKey(key);
        Double ceiling = ceilingKey(key);

        ShooterDataDistancePoint interpolatedPoint;

        if (floor == null && ceiling == null) {
            // If the floor and ceiling keys are not present, no keys are in the map and
            // there is nothing to interpolate.
            return null;
        } else if (floor == null) {
            ShooterDataDistancePoint floorVal = get(ceiling);
            ShooterDataDistancePoint ceilingVal = get(ceilingKey(ceiling));
            interpolatedPoint = interpolate(floorVal, ceilingVal, key);
        } else {
            ShooterDataDistancePoint ceilingVal = get(floor);
            ShooterDataDistancePoint floorVal = get(floorKey(floor));
            interpolatedPoint = interpolate(floorVal, ceilingVal, key);
        }

        return interpolatedPoint;
    }

    public static ShooterDataDistancePoint interpolate(ShooterDataDistancePoint floor, ShooterDataDistancePoint ceiling,
            Double key) {
        double angleSlope = (ceiling.getAngle() - floor.getAngle()) / (ceiling.getDistance() - floor.getDistance());
        double powerSlope = (ceiling.getPower() - floor.getPower()) / (ceiling.getDistance() - floor.getDistance());
        double distanceOffset = key - floor.getDistance();
        double interpolateAngle = angleSlope * distanceOffset + floor.getAngle();
        double interpolatePower = powerSlope * distanceOffset + floor.getPower();
        ShooterDataDistancePoint interpolatePoint = new ShooterDataDistancePoint(key, interpolateAngle,
                interpolatePower);

        return interpolatePoint;
    }
}
