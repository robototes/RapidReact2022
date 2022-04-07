package frc.team2412.robot.util.controller;

import java.util.*;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.XboxController;

/**
 * Compound controller class
 *
 * @author Alex Stedman
 * @param <T> preset map type
 * @param <U> controller type
 */
@SuppressWarnings("unused")
public class CompoundController<T, U extends Controller> extends OptionalController<U>
        implements MultiController<T, U> {
    private final SwitchableController<T, U> switchableController;

    /**
     * create compound controller
     *
     * @param one if for the presets onlyOne mode is enabled
     * @param c controller
     * @param presets preset keys
     */
    @SafeVarargs
    public CompoundController(boolean one, U c, T... presets) {
        super(c, true);
        switchableController = new SwitchableController<>(one, c, presets);
    }

    /**
     * create compound controller
     *
     * @param c controller
     * @param presets preset keys
     */
    @SafeVarargs
    public CompoundController(U c, T... presets) {
        this(true, c, presets);
    }

    @Override
    public boolean activate(T... preset) {
        return switchableController.activate(preset);
    }

    @Override
    public boolean deactivate(Object... preset) {
        return switchableController.deactivate(preset);
    }

    @Override
    public U getRawController(Object key) {
        return switchableController.getRawController(key);
    }

    @Override
    public Set<T> getChoice() {
        return switchableController.getChoice();
    }

    @Override
    public OptionalController<U> getPreset(Object key) {
        return switchableController.getPreset(key);
    }

    @Override
    public boolean onlyOnePreset() {
        return switchableController.onlyOnePreset();
    }

    @Override
    public OptionalController<?>[] getActiveController() {
        return switchableController.getActiveController();
    }

    /**
     * Staticly create compound controller on port
     *
     * @param port port for xbox controller
     * @param args array for preset map
     * @param <T> type of preset key
     * @return the new compound controller
     */
    @SafeVarargs
    public static <T> CompoundController<T, XboxController> of(int port, T... args) {
        return of(true, port, args);
    }

    /**
     * Staticly create compound controller on port
     *
     * @param one if only one preset mode is enabled
     * @param port port for xbox controller
     * @param args array for preset map
     * @param <T> type of preset key
     * @return the new compound controller
     */
    @SafeVarargs
    public static <T> CompoundController<T, XboxController> of(boolean one, int port, T... args) {
        return new CompoundController<>(one, new XboxController(port), args);
    }
}
