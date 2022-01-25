package frc.team2412.robot

import org.frcteam2910.common.robot.input.XboxController

public class Controls(sub: SubsystemContainer) {
    public companion object ControlConstants {
        public val CONTROLLER_PORT = 0
    }
    public val subsystems = sub
    public val controller = XboxController(ControlConstants.CONTROLLER_PORT)

    init {
        if (SubsystemContainer.CLIMB_ENABLED) bindClimbControls()
        if (SubsystemContainer.DRIVE_ENABLED) bindDriveControls()
        if (SubsystemContainer.INDEX_ENABLED) bindIndexControls()
        if (SubsystemContainer.INTAKE_ENABLED) bindIntakeControls()
        if (SubsystemContainer.SHOOTER_ENABLED) bindShoooterControls()
    }

    // TODO
    public fun bindClimbControls() {}

    public fun bindDriveControls() {}

    public fun bindIndexControls() {}

    public fun bindIntakeControls() {}

    public fun bindShoooterControls() {}
}
