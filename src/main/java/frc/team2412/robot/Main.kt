// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot

import edu.wpi.first.wpilibj.RobotBase

public class Main {
    companion object {
        @JvmStatic
        public fun main(args: Array<String>) {
            RobotBase.startRobot(Robot::getInstance)
        }
    }
}
