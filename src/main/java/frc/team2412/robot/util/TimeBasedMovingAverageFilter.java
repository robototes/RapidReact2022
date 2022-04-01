package frc.team2412.robot.util;

import java.util.ArrayDeque;
import java.util.Queue;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class that implements a moving average filter with a limit on the age of data.
 */
public class TimeBasedMovingAverageFilter {
    private static class TimeData {
        public final double time;
        public final double data;

        public TimeData(double time, double data) {
            this.time = time;
            this.data = data;
        }
    }

    private final Queue<TimeData> inputs;
    private final double filterTime;

    /**
     * Creates a new {@link TimeBasedMovingAverageFilter}.
     *
     * @param filterTime
     *            Length of time in seconds that inputs affects the moving average.
     */
    public TimeBasedMovingAverageFilter(double filterTime) {
        this.inputs = new ArrayDeque<>();
        this.filterTime = filterTime;
    }

    /**
     * Calculates the next value of the filter.
     *
     * @param currentInput
     *            Current input value.
     * @return The filtered value at this step.
     */
    public double calculate(double currentInput) {
        double time = Timer.getFPGATimestamp();
        // Add value to queue
        inputs.add(new TimeData(time, currentInput));
        // Remove values that are too old
        final double cutoffTime = time - filterTime;
        while (inputs.peek().time < cutoffTime) {
            inputs.remove();
        }
        // Calculate mean average
        double total = 0;
        for (TimeData prevInput : inputs) {
            total += prevInput.data;
        }
        return total / inputs.size();
    }

    /**
     * Reset the filter state.
     */
    public void reset() {
        inputs.clear();
    }
}
