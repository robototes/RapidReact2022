package frc.team2412.robot;

import org.frcteam2910.common.robot.input.XboxController;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
    }

    public XboxController controller;

    public Subsystems subsystems;

    public Controls(Subsystems s) {
        subsystems = s;
        controller = new XboxController(ControlConstants.CONTROLLER_PORT);
        if (CLIMB_ENABLED)
            bindClimbControls();
        if (DRIVE_ENABLED)
            bindDriveControls();
        if (INDEX_ENABLED)
            bindIndexControls();
        if (INTAKE_ENABLED)
            bindIntakeControls();
        if (SHOOTER_ENABLED)
            bindShooterControls();
    }

    // TODO these yay

    public void bindClimbControls() {

    }

    public void bindDriveControls() {

    }

    public void bindIndexControls() {

    }

    public void bindIntakeControls() {

    }

    public void bindShooterControls() {

    }
}
