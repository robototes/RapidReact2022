package frc.team2412.robot.util;

import edu.wpi.first.wpilibj2.command.button.Button;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.DPadButton;

public class OptionalController<T extends Controller> extends Controller {

    private final T controller;

    private boolean active;

    public OptionalController(T c) {
        this(c, false);
    }

    public OptionalController(T c, boolean enabled) {
        controller = c;
        active = enabled;
    }

    public boolean isActive() {
        return active;
    }

    public boolean deactivate() {
        return set(false);
    }

    public boolean activate() {
        return set(true);
    }

    public boolean set(boolean a) {
        if (active == a)
            return false;
        active = a;
        return true;
    }

    protected Axis get(Axis a) {
        return new Axis() {
            @Override
            public double getRaw() {
                return active ? a.getRaw() : 0;
            }
        };
    }

    protected Button get(Button a) {
        return new Button(() -> active && a.getAsBoolean());
    }

    @Override
    public Axis getLeftTriggerAxis() {
        return get(controller.getLeftTriggerAxis());
    }

    @Override
    public Axis getLeftXAxis() {
        return get(controller.getLeftXAxis());
    }

    @Override
    public Axis getLeftYAxis() {
        return get(controller.getLeftYAxis());
    }

    @Override
    public Axis getRightTriggerAxis() {
        return get(controller.getRightTriggerAxis());
    }

    @Override
    public Axis getRightXAxis() {
        return get(controller.getRightXAxis());
    }

    @Override
    public Axis getRightYAxis() {
        return get(controller.getRightYAxis());
    }

    @Override
    public Button getAButton() {
        return get(controller.getAButton());
    }

    @Override
    public Button getBButton() {
        return get(controller.getBButton());
    }

    @Override
    public Button getXButton() {
        return get(controller.getXButton());
    }

    @Override
    public Button getYButton() {
        return get(controller.getYButton());
    }

    @Override
    public Button getLeftBumperButton() {
        return get(controller.getLeftBumperButton());
    }

    @Override
    public Button getRightBumperButton() {
        return get(controller.getRightBumperButton());
    }

    @Override
    public Button getBackButton() {
        return get(controller.getBackButton());
    }

    @Override
    public Button getStartButton() {
        return get(controller.getStartButton());
    }

    @Override
    public Button getLeftJoystickButton() {
        return get(controller.getLeftJoystickButton());
    }

    @Override
    public Button getRightJoystickButton() {
        return get(controller.getRightJoystickButton());
    }

    @Override
    public Button getDPadButton(DPadButton.Direction direction) {
        return get(controller.getDPadButton(direction));
    }

}
