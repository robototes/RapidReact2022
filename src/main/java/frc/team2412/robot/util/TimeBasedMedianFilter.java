package frc.team2412.robot.util;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
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
        values.add(new TimeData(time, next));
        orderedValues.add(next);
        orderedValues.sort(Comparator.comparingDouble((value) -> value.doubleValue()));
        while (values.peek().time < time - filterTime) {
            orderedValues.remove(values.remove().data);
        }
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
