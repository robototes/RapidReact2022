// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;
import org.frcteam2910.common.math.Vector2;

public class VectorSlewLimiter {
    private final double rateLimit;
    private Vector2 prevVal;
    private double prevTime;

    public VectorSlewLimiter(double rateLimit, Vector2 initialValue) {
        this.rateLimit = rateLimit;
        prevVal = initialValue;
        prevTime = WPIUtilJNI.now() * 1e-6;
    }

    public VectorSlewLimiter(double rateLimit) {
        this(rateLimit, Vector2.ZERO);
    }

    public Vector2 calculate(Vector2 input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - prevTime;
        prevTime = currentTime;
        double x =
                MathUtil.clamp(
                        input.x - prevVal.x, -rateLimit * elapsedTime, rateLimit * elapsedTime);
        double y =
                MathUtil.clamp(
                        input.y - prevVal.y, -rateLimit * elapsedTime, rateLimit * elapsedTime);
        prevVal = prevVal.add(x, y);
        return prevVal;
    }

    public void reset(Vector2 value) {
        prevVal = value;
        prevTime = WPIUtilJNI.now() * 1e-6;
    }
}
