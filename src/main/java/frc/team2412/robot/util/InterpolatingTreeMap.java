package frc.team2412.robot.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.TreeMap;
import java.util.function.Consumer;

import frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants;

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
            int lineNum = 0;
            InterpolatingTreeMap map = new InterpolatingTreeMap();

            while ((line = reader.readLine()) != null) {
                lineNum++;
                String msgPrefix = "Line #" + lineNum + ": ";
                Consumer<String> debug = (msg) -> {
                    System.out.println(msgPrefix + msg);
                };

                int hashtagIndex = line.indexOf("#");
                int doubleSlashIndex = line.indexOf("//");
                if (hashtagIndex == 0) {
                    debug.accept("Starts with '#', skipping line");
                    continue;
                }
                if (doubleSlashIndex == 0) {
                    debug.accept("Starts with '//', skipping line");
                    continue;
                }
                if (hashtagIndex != -1 && (doubleSlashIndex == -1 || hashtagIndex < doubleSlashIndex)) {
                    debug.accept("'#' at char index " + hashtagIndex + ", trimming comment");
                    line = line.substring(0, hashtagIndex);
                } else if (doubleSlashIndex != -1 && (hashtagIndex == -1 || doubleSlashIndex < hashtagIndex)) {
                    debug.accept("'//' at char index " + doubleSlashIndex + ", trimming comment");
                    line = line.substring(0, doubleSlashIndex);
                }

                String[] items = line.split(",", -1);
                if (items.length < 4) {
                    debug.accept("Less than 4 items, skipping line");
                    continue;
                } else if (items.length > 4) {
                    debug.accept("More than 4 items, ignoring extra items");
                    // Extra items aren't processed, could use Arrays.copyOf(items, [newlength]) if needed
                }

                double distance, angle, RPM, timeOfFlight;

                try {
                    distance = Double.parseDouble(items[0]);
                    angle = Double.parseDouble(items[1]);
                    RPM = Double.parseDouble(items[2]);
                    timeOfFlight = Double.parseDouble(items[3]);
                } catch (NumberFormatException err) {
                    debug.accept("Non-numerical value, skipping line");
                    continue;
                }

                if (distance < 0) {
                    debug.accept("Distance " + distance + " is negative, skipping line");
                    continue;
                }
                if (angle < ShooterConstants.MIN_HOOD_ANGLE) {
                    debug.accept("Hood angle " + angle + " is less than the min value, skipping line");
                    continue;
                }
                if (angle > ShooterConstants.MAX_HOOD_ANGLE) {
                    debug.accept("Hood angle " + angle + " is greater than the max value, skipping line");
                    continue;
                }
                if (RPM < 0) {
                    debug.accept("Flywheel RPM " + RPM + " is negative, skipping line");
                    continue;
                }

                if (timeOfFlight < 0) {
                    debug.accept("Time of flight " + timeOfFlight + "is negative, skipping line");
                    continue;
                }

                map.addDataPoint(new ShooterDataDistancePoint(distance, angle, RPM, timeOfFlight));
            }

            // Debug code
            System.out.println("All points:");
            for (ShooterDataDistancePoint point : map.values()) {
                System.out.println(point.getDistance() + ": " + point.getAngle() + ", " + point.getRPM());
            }

            System.out.println("Done deserializing CSV");
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
        double rpmSlope = (ceiling.getRPM() - floor.getRPM()) / slopeDistanceDifference;
        double timeOfFlightSlope = (ceiling.getTimeOfFlight() - floor.getTimeOfFlight()) / slopeDistanceDifference;

        double distanceOffset = key - floor.getDistance();

        double interpolateAngle = angleSlope * distanceOffset + floor.getAngle();
        double interpolateRPM = rpmSlope * distanceOffset + floor.getRPM();
        double interpolateTimeOfFlight = timeOfFlightSlope * distanceOffset + floor.getTimeOfFlight();

        ShooterDataDistancePoint interpolatePoint = new ShooterDataDistancePoint(key, interpolateAngle,
                interpolateRPM, interpolateTimeOfFlight);

        return interpolatePoint;
    }
}
