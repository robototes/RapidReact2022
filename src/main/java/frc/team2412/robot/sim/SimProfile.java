package frc.team2412.robot.sim;

import edu.wpi.first.wpilibj.Timer;

public abstract class SimProfile {
    private Timer timer = null;

    /**
     * Runs the simulation profile. Overrided by device specific profiles.
     */
    public abstract void run();

    /**
     * Returns the time since last call.
     *
     * @return Time since last call, in milliseconds.
     */
    protected double getPeriod() {
        if (timer == null) {
            timer = new Timer();
        }

        double period = timer.get() * 1000;
        timer.reset();

        return period;
    }
}
