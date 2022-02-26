package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            ShooterVisionSubsystem shooterVisionSubsystem) {

        addCommands(new IndexShootCommand(indexSubsystem),
                new ShooterTargetCommand(shooterSubsystem, shooterVisionSubsystem));
    }
}
