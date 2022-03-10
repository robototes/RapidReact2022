// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class VectorSlewLimiter {
    private final double m_rateLimit;
    private Vector2 m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new SlewRateLimiter with the given rate limit and initial value.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     */
    public VectorSlewLimiter(double rateLimit, Vector2 initialValue) {
        m_rateLimit = rateLimit;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }


    public VectorSlewLimiter(double rateLimit) {
        this(rateLimit, Vector2.ZERO);
    }


    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public Vector2 calculate(Vector2 input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        double x = MathUtil.clamp(input.x-m_prevVal.x, -m_rateLimit*elapsedTime, m_rateLimit*elapsedTime);
        double y = MathUtil.clamp(input.y-m_prevVal.y, -m_rateLimit*elapsedTime, m_rateLimit*elapsedTime);
        m_prevVal = m_prevVal.add(x, y);
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(Vector2 value) {
        m_prevVal = value;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }
}
