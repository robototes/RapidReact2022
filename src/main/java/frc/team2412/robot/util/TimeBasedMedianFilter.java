package frc.team2412.robot.util;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Queue;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class that implements a median filter based on how recent information is.
 */
public class TimeBasedMedianFilter {

    private static class TimeData {
        public final double time;
        public final double data;

        public TimeData(double time, double data) {
            this.time = time;
            this.data = data;
        }
    }

    private final Queue<TimeData> values;
    private final List<Double> orderedValues;
    private final double filterTime;

    /**
     * Creates a new {@code TimeBasedAverageFilter}.
     *
     * @param filterTime
     *            Length of time in seconds that inputs affect average.
     */
    public TimeBasedMedianFilter(double filterTime) {
        this.filterTime = filterTime;
        this.values = new ArrayDeque<>();
        this.orderedValues = new ArrayList<>();
    }

    /**
     * Calculates the median of the most recent values for the next value of the input stream.
     *
     * @param next
     *            The next input value.
     * @return The median of the values within a certain time window, including the next value.
     */
    public double calculate(double next) {
        double time = Timer.getFPGATimestamp();
        // Add value to queue
        values.add(new TimeData(time, next));
        // Insert at appropriate position to keep list sorted
        int index = Collections.binarySearch(orderedValues, next);
        if (index < 0) {
            index = -1 - index;
        }
        orderedValues.add(index, next);
        // Remove values that are too old
        final double cutoffTime = time - filterTime;
        while (values.peek().time < cutoffTime) {
            orderedValues.remove(Collections.binarySearch(orderedValues, values.remove().data));
        }
        // Calculate median
        int size = values.size();
        if (size % 2 != 0) {
            return orderedValues.get(size / 2);
        }
        return (orderedValues.get(size / 2 - 1) + orderedValues.get(size / 2)) / 2.0;
    }

    /**
     * Resets the filter, clearing the window of all elements
     */
    public void reset() {
        values.clear();
        // <o/ dabs
    }
}
