package frc.team2412.robot.util.controller;

import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.XboxController;

import java.util.*;

/**
 * Controller class that maps multiple controllers to a single physical controller and allows the user to
 * switch between them
 *
 * @author Alex Stedman
 * @param <T>
 *            key for switching
 * @param <U>
 *            type of wrapped optional controllers
 */
@SuppressWarnings("unused")

public class SwitchableController<T, U extends Controller> implements MultiController<T, U> {
    private final Map<T, OptionalController<U>> controllers;
    private final Set<T> choice;
    private final boolean oneChoice;

    /**
     * create switchable controller
     *
     * @param one
     *            if for the presets onlyOne mode is enabled
     * @param c
     *            controller
     * @param presets
     *            preset keys
     */
    @SafeVarargs
    public SwitchableController(boolean one, U c, T... presets) {
        oneChoice = one;
        controllers = new LinkedHashMap<>();
        for (T preset : presets)
            controllers.put(preset, new OptionalController<>(c));
        choice = new LinkedHashSet<>();

    }

    /**
     * create switchable controller
     *
     * @param c
     *            controller
     * @param presets
     *            preset keys
     */
    @SafeVarargs
    public SwitchableController(U c, T... presets) {
        this(true, c, presets);
    }

    @Override
    public Set<T> getChoice() {
        return choice;
    }

    @SuppressWarnings("SuspiciousMethodCalls")
    @Override
    public OptionalController<U> getPreset(Object key) {
        // SUS
        return controllers.get(key);
    }

    @Override
    public boolean onlyOnePreset() {
        return oneChoice;
    }

    @Override
    public OptionalController<?>[] getActiveController() {
        return choice.stream().map(this::getPreset).toArray(OptionalController<?>[]::new);
    }

    /**
     * Staticly create switchable controller on port
     *
     * @param port
     *            port for xbox controller
     * @param args
     *            array for preset map
     * @param <T>
     *            type of preset key
     * @return the new compound controller
     */
    @SafeVarargs
    public static <T> SwitchableController<T, XboxController> of(int port, T... args) {
        return of(true, port, args);
    }

    /**
     * Staticly create switchable controller on port
     *
     * @param one
     *            if only one preset mode is enabled
     * @param port
     *            port for xbox controller
     * @param args
     *            array for preset map
     * @param <T>
     *            type of preset key
     * @return the new compound controller
     */
    @SafeVarargs
    public static <T> SwitchableController<T, XboxController> of(boolean one, int port, T... args) {
        return new SwitchableController<>(one, new XboxController(port), args);
    }
}
