package frc.team2412.robot

import frc.team2412.robot.SubsystemContainer.SubsystemConstants.*
import org.frcteam2910.common.robot.input.XboxController

public class Controls(sub: SubsystemContainer) {
    public companion object ControlConstants {
        public val CONTROLLER_PORT = 0
    }
    public val subsystems = sub
    public val controller = XboxController(ControlConstants.CONTROLLER_PORT)

    init {
        if (CLIMB_ENABLED) bindClimbControls()
        if (DRIVE_ENABLED) bindDriveControls()
        if (INDEX_ENABLED) bindIndexControls()
        if (INTAKE_ENABLED) bindIntakeControls()
        if (SHOOTER_ENABLED) bindShoooterControls()
    }

    // TODO
    public fun bindClimbControls() {}

    public fun bindDriveControls() {}

    public fun bindIndexControls() {}

    public fun bindIntakeControls() {}

    public fun bindShoooterControls() {}
}
