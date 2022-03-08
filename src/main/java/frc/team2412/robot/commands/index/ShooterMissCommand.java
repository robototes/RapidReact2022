package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.shooter.ShooterMisfireCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class ShooterMissCommand extends ParallelCommandGroup {

    public ShooterMissCommand(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            ShooterVisionSubsystem shooterVisionSubsystem) {

        addCommands(new IndexShootCommand(indexSubsystem),
                new ShooterMisfireCommand(shooterSubsystem, shooterVisionSubsystem));
    }
}
