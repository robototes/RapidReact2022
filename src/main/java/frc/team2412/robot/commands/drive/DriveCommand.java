package frc.team2412.robot.commands.drive;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class DriveCommand extends CommandBase {
    private final DrivebaseSubsystem drivebaseSubsystem;
    private final Axis forward;
    private final Axis strafe;
    private final Axis rotation;
    private final boolean fieldOriented;

    public DriveCommand(DrivebaseSubsystem drivebaseSubsystem, Axis forward, Axis strafe, Axis rotation,
            boolean fieldOriented) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;

        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void execute() {
        drivebaseSubsystem.drive(new Vector2(-forward.get(false), strafe.get(false)), rotation.get(false),
                fieldOriented);
        // drivebaseSubsystem.drive(new Vector2(-forward.get(false), 0.0), 0.0,
        // fieldOriented);
    }

    @Override
    public void end(boolean interrupted) {
        //drivebaseSubsystem.drive(Vector2.ZERO, 0.0, fieldOriented);
    }
}
