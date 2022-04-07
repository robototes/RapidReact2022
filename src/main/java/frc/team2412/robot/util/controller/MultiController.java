package frc.team2412.robot.util.controller;

import java.util.Arrays;
import java.util.Set;
import org.frcteam2910.common.robot.input.Controller;

/**
 * Overarching class for multi controllers with presets
 *
 * @author Alex Stedman
 * @param <T> index for map
 * @param <U> controller type
 */
@SuppressWarnings("unused")
public interface MultiController<T, U extends Controller> {
    /**
     * Get current presets
     *
     * @return set of presets, or 1 element set if single preset mode is enabled
     */
    Set<T> getChoice();

    /**
     * Get controller associated with key
     *
     * @param key the key
     * @return the controller
     */
    OptionalController<U> getPreset(Object key);

    /**
     * get raw controller associated with key
     *
     * @param key the key
     * @return the raw controller
     */
    default U getRawController(Object key) {
        return getPreset(key).getController();
    }

    /**
     * Activate a list of presets. if single preset mode is active it will deactivate all presets
     * first
     *
     * @param preset presets to activate
     * @return if change was made to presets
     */
    @SuppressWarnings("unchecked")
    default boolean activate(T... preset) {
        if (onlyOnePreset()) deactivate();
        for (T t : preset) {
            if (getPreset(t) != null) getPreset(t).activate();
        }
        return getChoice().addAll(Arrays.asList(preset.clone()));
    }

    /**
     * Deactivate a list of presets
     *
     * @param preset presets to deactivate
     * @return if a change was made
     */
    @SuppressWarnings("SuspiciousMethodCalls")
    default boolean deactivate(Object... preset) {
        // SUSUSUS
        if (!getChoice().removeAll(Arrays.asList(preset.clone()))) return false;
        for (Object t : preset) {
            if (getPreset(t) != null) getPreset(t).deactivate();
        }
        return true;
    }

    /**
     * deactivate all presets
     *
     * @return if a change was made
     */
    default boolean deactivate() {
        return deactivate(getChoice().toArray());
    }

    /** example controller types you can use for making IDs */
    enum Controllers {
        PRIMARY,
        SECONDARY,
        TERTIARY,
        BACKUP,
        DRIVER,
        CODRIVER,
        DEBUG,
        TEST;
    }

    /**
     * if only one preset mode is enabled
     *
     * @return if only one mode is enabled
     */
    boolean onlyOnePreset();

    /**
     * get array of active controllers
     *
     * @return array of active controllers
     */
    OptionalController<?>[] getActiveController();
}
