// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot

import edu.wpi.first.wpilibj.TimedRobot

public class Robot : TimedRobot() {
    companion object {
        /** Singleton Stuff */
        private val instance = Robot()

        @JvmStatic
        public fun getInstance(): Robot {
            return instance
        }
    }

    public val hardware = Hardware()
    public val subsystems = Subsystems(hardware)
    public val controls = Controls(subsystems)

    // TODO start overriding methods

}
