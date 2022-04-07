package frc.team2412.robot.util;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public interface UtilityCommand {
    default InstantCommand instant(Runnable r) {
        return new InstantCommand(r);
    }

    default WaitCommand await(double t) {
        return new WaitCommand(t);
    }

    default ScheduleCommand schedule(Command c) {
        return new ScheduleCommand(c);
    }
}
