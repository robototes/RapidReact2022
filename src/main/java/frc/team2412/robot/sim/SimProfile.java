package frc.team2412.robot.sim;

public abstract class SimProfile {
    private long lastTime;
    private boolean running = false;

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
        if (!running) {
            lastTime = System.nanoTime();
            running = true;
        }

        long now = System.nanoTime();
        double period = (now - lastTime) / 1_000_000.;
        lastTime = now;

        return period;
    }
}
