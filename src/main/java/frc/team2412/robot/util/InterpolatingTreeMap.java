package frc.team2412.robot.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.TreeMap;

public class InterpolatingTreeMap extends TreeMap<Double, ShooterDataDistancePoint> {
    /**
     * Creates an empty {@link InterpolatingTreeMap}.
     */
    public InterpolatingTreeMap() {
        this(new ShooterDataDistancePoint[] {});
    }

    /**
     * Creates an {@link InterpolatingTreeMap} from an array of {@link ShooterDataDistancePoint}.
     */
    public InterpolatingTreeMap(ShooterDataDistancePoint[] dataPoints) {
        super();
        for (ShooterDataDistancePoint dataPoint : dataPoints) {
            addDataPoint(dataPoint);
        }
    }

    /**
     * Creates a {@link InterpolatingTreeMap} from a path to a CSV file.
     *
     * @param fileName
     *            The path to the CSV file.
     * @return An {@link InterpolatingTreeMap} from the data in the CSV file.
     */
    public static InterpolatingTreeMap fromCSV(String fileName) {
        System.out.println("Deserializing " + fileName + " to an InterpolatingTreeMap");
        try (BufferedReader reader = Files.newBufferedReader(Paths.get(fileName))) {
            String line;
            InterpolatingTreeMap map = new InterpolatingTreeMap();

            while ((line = reader.readLine()) != null) {
                String[] items = line.split(",");
                if (items.length < 3) {
                    System.out.println("Line " + line + " has less than 3 items, skipping line");
                    continue;
                } else if (items.length > 3) {
                    System.out.println(
                            "Line " + line + " has more than 3 items, ignoring extra items");
                }

                try {
                    ShooterDataDistancePoint point = new ShooterDataDistancePoint(Double.parseDouble(items[0]),
                            Double.parseDouble(items[1]),
                            Double.parseDouble(items[2]));
                    map.addDataPoint(point);
                } catch (NumberFormatException err) {
                    System.out.println("Line " + line + " contains a non-numerical value, skipping line");
                    continue;
                }
            }

            return map;
        } catch (IOException err) {
            err.printStackTrace();
            return null;
        }
    }

    /**
     * Replaces all data in the {@link InterpolatingTreeMap} with data from a CSV file.
     *
     * @param fileName
     *            The path to the CSV file.
     */
    public void replaceFromCSV(String fileName) {
        clear();
        putAll(fromCSV(fileName));
    }

    /**
     * Adds a {@link ShooterDataDistancePoint}.
     *
     * @param dataPoint
     *            The {@link ShooterDataDistancePoint} to add
     */
    private void addDataPoint(ShooterDataDistancePoint dataPoint) {
        put(dataPoint.getDistance(), dataPoint);
    }

    /**
     * Gets an value at a specified distance from the origin, interpolating it if there isn't an exact
     * match.
     *
     * @param key
     *            The distance to get the value from.
     * @return An value from the {@link InterpolatingTreeMap}, interpolated if there isn't an exact
     *         match.
     */
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
            System.out.println("getInterpolated was called, but InterpolatingTreeMap is empty");
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

    /**
     * Returns an interpolated value at a specified distance from the origin.
     *
     * @param floor
     *            The {@link ShooterDataDistancePoint} closer to the origin.
     * @param ceiling
     *            The {@link ShooterDataDistancePoint} father from the origin.
     * @param key
     *            The distance to interpolate to.
     * @return {@link ShooterDataDistancePoint} an interpolated value at the specified distance.
     */
    private static ShooterDataDistancePoint interpolate(ShooterDataDistancePoint floor,
            ShooterDataDistancePoint ceiling,
            Double key) {
        double slopeDistanceDifference = ceiling.getDistance() - floor.getDistance();
        if (slopeDistanceDifference == 0 || Double.isNaN(slopeDistanceDifference)
                || Double.isInfinite(slopeDistanceDifference)) {
            System.out.println("ERROR, distance between sample points is an illegal value: " + slopeDistanceDifference);
            return null;
        }
        double angleSlope = (ceiling.getAngle() - floor.getAngle()) / slopeDistanceDifference;
        double powerSlope = (ceiling.getPower() - floor.getPower()) / slopeDistanceDifference;
        double distanceOffset = key - floor.getDistance();
        double interpolateAngle = angleSlope * distanceOffset + floor.getAngle();
        double interpolatePower = powerSlope * distanceOffset + floor.getPower();
        ShooterDataDistancePoint interpolatePoint = new ShooterDataDistancePoint(key, interpolateAngle,
                interpolatePower);

        return interpolatePoint;
    }
}
