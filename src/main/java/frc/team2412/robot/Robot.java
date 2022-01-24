// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    /**
     * Singleton Stuff
     */
    private static Robot instance = null;

    public static Robot getInstance() {
        if (instance == null)
            instance = new Robot();
        return instance;
    }

    public final Controls controls;
    public final SubsystemContainer subsystems;
    public final Hardware hardware;

    private Robot() {
        instance = this;
        hardware = new Hardware();
        subsystems = new SubsystemContainer(hardware);
        controls = new Controls(subsystems);
    }

    // TODO start overriding methods

}
