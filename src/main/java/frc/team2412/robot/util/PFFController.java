package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.Vector3;

/**
 * Class for PFF controllers that provide P controllers but with the advantage of FeedForward
 *
 * @author Alex Stedman
 * @param <T>
 *            type of input/output
 */
@SuppressWarnings("unused")
public class PFFController<T> {
    private final double p, f;
    private final Timer timer;
    private final PFFOperator<T> op;

    /**
     * functional interface for core operator used by the controller
     *
     * @author Alex Stedman
     * @param <T>
     *            type of input/output
     */
    @FunctionalInterface
    public interface PFFOperator<T> {
        /**
         * operates as the following
         *
         * @param first
         *            value a
         * @param second
         *            value b
         * @param scale
         *            value c
         *
         * @return (a-b)*c
         */
        T apply(T first, T second, double scale);
    }

    protected PFFController(double pValue, double fValue, PFFOperator<T> operator) {
        p = pValue;
        f = fValue;
        op = operator;
        timer = new Timer();
    }

    private T targetPosition;

    /**
     * Set the target position of the controller
     *
     * @param target
     *            new target position
     * @return this
     */
    public PFFController<T> setTargetPosition(T target) {
        targetPosition = target;
        timer.reset();
        return this;
    }

    private T pastValue;

    /**
     * output the adjustment offered by the controller
     *
     * @param measuredValue
     *            measured value at current time
     * @return new value to set
     */
    public T update(T measuredValue) {
        if (pastValue == null)
            pastValue = measuredValue;
        if (targetPosition == null)
            return null;
        T velocity = op.apply(measuredValue, pastValue, -f);
        pastValue = measuredValue;
        T pControl = op.apply(targetPosition, measuredValue, p);
        timer.reset();
        return op.apply(pControl, velocity, 1);
    }

    /**
     * create PFF controller of doubles
     *
     * @param pValue
     *            P value for proportional control
     * @param fValue
     *            F value for feedforward control
     * @return new PFFController
     */
    public static PFFController<Double> ofDouble(double pValue, double fValue) {
        return new PFFController<>(pValue, fValue, (a, b, s) -> (a - b) * s);
    }

    /**
     * create PFF controller of {@link Vector2}
     *
     * @param pValue
     *            P value for proportional control
     * @param fValue
     *            F value for feedforward control
     * @return new PFFController
     */
    public static PFFController<Vector2> ofVector2(double pValue, double fValue) {
        return new PFFController<>(pValue, fValue, (a, b, s) -> a.subtract(b).scale(s));
    }

    /**
     * create PFF controller of {@link Vector3}
     *
     * @param pValue
     *            P value for proportional control
     * @param fValue
     *            F value for feedforward control
     * @return new PFFController
     */
    public static PFFController<Vector3> ofVector3(double pValue, double fValue) {
        return new PFFController<>(pValue, fValue, (a, b, s) -> a.subtract(b).scale(s));
    }
}
