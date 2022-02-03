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

        ShooterDataDistancePoint floorVal;
        ShooterDataDistancePoint ceilingVal;

        if (floor == null && ceiling == null) {
            // If the floor and ceiling keys are not present, no keys are in the map and
            // there is nothing to interpolate.
            return null;
        } else if (floor == null) {
            // key is below lowest value in map
            floorVal = get(ceiling);
            ceilingVal = get(higherKey(ceiling));
        } else if (ceiling == null) {
            // key is above highest value in map
            floorVal = get(lowerKey(floor));
            ceilingVal = get(floor);
        } else {
            // key is in between values in map
            floorVal = get(floor);
            ceilingVal = get(ceiling);
        }

        return interpolate(floorVal, ceilingVal, key);
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
