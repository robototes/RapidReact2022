package frc.team2412.robot.commands.index;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;

public class IndexShootCommand extends CommandBase {
    private final IndexSubsystem subsystem;
    private final TargetLocalizer localizer;
    private final NetworkTableEntry shootSpeedToggled;

    public IndexShootCommand(IndexSubsystem subsystem, TargetLocalizer targetLocalizer) {
        this.subsystem = subsystem;
        localizer = targetLocalizer;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        shootSpeedToggled = inst.getTable("Shuffleboard/Drivebase").getEntry("ShootSpeedToggled");
        addRequirements(subsystem);
    }

    public IndexShootCommand(IndexSubsystem subsystem) {
        this(subsystem, null);
    }

    @Override
    public void execute() {
        // turn on both motors
        if (localizer == null || localizer.upToSpeed()) {
            subsystem.ingestMotorShoot();
            subsystem.feederMotorShoot();
            shootSpeedToggled.setBoolean(true);
        } else {
            subsystem.ingestMotorStop();
            subsystem.feederMotorStop();
            shootSpeedToggled.setBoolean(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.ingestMotorStop();
        subsystem.feederMotorStop();
        shootSpeedToggled.setBoolean(false);
    }
}
