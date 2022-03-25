package frc.team2412.robot.util;

import java.util.ArrayDeque;
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
    private final double filterTime;

    /**
     * Creates a new {@code TimeBasedMedianFilter}.
     *
     * @param filterTime
     *            Length of time in seconds that inputs affect average.
     */
    public TimeBasedMedianFilter(double filterTime) {
        this.filterTime = filterTime;
        this.values = new ArrayDeque<>();
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
        while (values.peek().time < time - filterTime) {
            values.remove();
        }
        int count = values.size();
        double total = 0;
        for (TimeData value : values) {
            total += value.data;
        }
        return total / count;
    }

    /**
     * Resets the filter, clearing the window of all elements
     */
    public void reset() {
        values.clear();
        // <o/ dabs
    }
}
