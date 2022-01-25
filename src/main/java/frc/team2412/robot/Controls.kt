package frc.team2412.robot

import org.frcteam2910.common.robot.input.XboxController

public class Controls(sub: Subsystems) {
    public companion object ControlConstants {
        public val CONTROLLER_PORT = 0
    }
    public val subsystems = sub
    public val controller = XboxController(ControlConstants.CONTROLLER_PORT)

    init {
        if (Subsystems.CLIMB_ENABLED) bindClimbControls()
        if (Subsystems.DRIVE_ENABLED) bindDriveControls()
        if (Subsystems.INDEX_ENABLED) bindIndexControls()
        if (Subsystems.INTAKE_ENABLED) bindIntakeControls()
        if (Subsystems.SHOOTER_ENABLED) bindShoooterControls()
    }

    // TODO
    public fun bindClimbControls() {}

    public fun bindDriveControls() {}

    public fun bindIndexControls() {}

    public fun bindIntakeControls() {}

    public fun bindShoooterControls() {}
}
