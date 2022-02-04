package frc.team2412.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;

public final class AutomatedTestMain {
    private AutomatedTestMain() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(new Supplier<RobotBase>() {
            @Override
            public RobotBase get() {
                return Robot.getInstance(Robot.RobotType.AUTOMATED_TEST);
            }
        });
    }
}
