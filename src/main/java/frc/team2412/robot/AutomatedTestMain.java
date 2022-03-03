package frc.team2412.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;

public final class AutomatedTestMain {
    private AutomatedTestMain() {
    }

    public static void main(String... args) {
        RobotBase.startRobot((Supplier<RobotBase>) () -> new Robot(Robot.RobotType.AUTOMATED_TEST));
    }
}
