package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;

import java.util.function.BooleanSupplier;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.robot.input.DPadButton.Direction;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.climb.FullExtendFixedHookCommand;
import frc.team2412.robot.commands.climb.RetractFixedHookCommand;
import frc.team2412.robot.commands.index.IndexCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;
import frc.team2412.robot.commands.intake.SpitBallCommand;
import frc.team2412.robot.commands.shooter.ShooterHoodRPMCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

@SuppressWarnings("unused")
public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    // public CompoundController<MultiController.Controllers, XboxController> driveController;
    public XboxController driveController;

    // climb
    public final Button climbFixedArmUp;
    // public final Button climbFixedArmFullUp;
    public final Button climbFixedArmDown;
    public final Button climbFixedArmFullDown;

    public Controller shootPreset, climbPreset;
    public XboxController codriverController;

    // index
    // public final Button indexShootButton;

    // shooter
    public final Button shootButton;
    // public final Button hoodUpButton;
    // public final Button hoodDownButton;
    // public final Button turretLeftButton;
    // public final Button turretRightButton;

    // intake
    public final Button[] intakeInButton;
    // public final Button intakeExtendButton;
    public final Button[] intakeSpitButton;
    public final Button intakeRetractButton;

    // drive
    public final Button resetDriveGyroButton;

    public Subsystems subsystems;

    private Controls(Subsystems s) {
        subsystems = s;

        // driveController = CompoundController.of(CONTROLLER_PORT, PRIMARY, SECONDARY);
        driveController = new XboxController(CONTROLLER_PORT);

        // driveController.activate(PRIMARY);

        // shootPreset = driveController.getPreset(PRIMARY);

        // climbPreset = driveController.getPreset(SECONDARY);

        // fixedArmUpManualButton = climbPreset.getDPadButton(Direction.UP);
        // fixedArmDownManualButton = climbPreset.getDPadButton(Direction.DOWN);
        // dynamicArmUpManualButton = climbPreset.getDPadButton(Direction.LEFT);
        // dynamicArmDownManualButton = climbPreset.getDPadButton(Direction.RIGHT);

        // fixedArmUpButton = climbPreset.getXButton();
        // fixedArmDownButton = climbPreset.getYButton();
        // dynamicArmUpButton = climbPreset.getAButton();
        // dynamicArmDownButton = climbPreset.getBButton();

        // rungClimbButton = climbPreset.getRightBumperButton();

        resetDriveGyroButton = driveController.getRightJoystickButton();
        shootButton = driveController.getLeftBumperButton();
        intakeInButton = new Button[] { driveController.getAButton(),
                driveController.getLeftTriggerAxis().getButton(0.1),
                driveController.getRightTriggerAxis().getButton(0.1),

        };
        intakeSpitButton = new Button[] { driveController.getBButton() };
        intakeRetractButton = driveController.getYButton();
        climbFixedArmUp = driveController.getStartButton();
        climbFixedArmDown = driveController.getBackButton();
        climbFixedArmFullDown = driveController.getDPadButton(DPadButton.Direction.DOWN);

        // intakeInButton = shootPreset.getRightBumperButton();
        // intakeExtendButton = shootPreset.getXButton();
        // intakeSpitButton = shootPreset.getAButton();
        // intakeRetractButton = shootPreset.getBButton();

        // indexShootButton = shootPreset.getLeftBumperButton();
        // shootButton = shootPreset.getLeftTriggerAxis().getButton(0.5);
        // hoodUpButton = shootPreset.getDPadButton(Direction.UP);
        // hoodDownButton = shootPreset.getDPadButton(Direction.DOWN);
        // turretLeftButton = shootPreset.getDPadButton(Direction.LEFT);
        // turretRightButton = shootPreset.getDPadButton(Direction.RIGHT);

        // driveController.getStartButton().whenPressed(() -> driveController.activate(PRIMARY));

        // driveController.getBackButton().whenPressed(() -> driveController.activate(SECONDARY));

        boolean comp = Robot.instance.isCompetition();

        if (subsystems.drivebaseSubsystem != null) {
            bindDriveControls();
        }

        if (subsystems.climbSubsystem != null) {
            bindClimbControls();
        }

        if (subsystems.indexSubsystem != null) {
            bindIndexControls();
        }

        if (subsystems.intakeSubsystem != null) {
            bindIntakeControls();
        }

        if (subsystems.shooterSubsystem != null) {
            bindShooterControls();
        }
    }

    public void bindClimbControls() {
        climbFixedArmDown.whenPressed(new RetractFixedHookCommand());
        climbFixedArmUp.whenPressed(new FullExtendFixedHookCommand());
        // climbFixedArmFullUp.whenPressed(new FullExtendFixedHookCommand(subsystems.climbSubsystem));
        // climbFixedArmFullDown.whenPressed(new FullRetractFixedHookCommand(subsystems.climbSubsystem));
    }

    public void bindDriveControls() {
        resetDriveGyroButton.whenPressed(() -> subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO));
    }

    public void bindIndexControls() {
        driveController.getRightBumperButton().whileHeld(new IndexCommand());
        // subsystems.indexSubsystem.setDefaultCommand(new IntakeBitmapCommand(subsystems.intakeSubsystem,
        // subsystems.indexSubsystem));
        // indexShootButton.whileHeld(new IndexShootCommand(subsystems.indexSubsystem));
        // shootButton.whileHeld(new IndexShootCommand(subsystems.indexSubsystem));
    }

    public void bindIntakeControls() {
        for (Button b : intakeInButton)
            b.whenPressed(new IntakeInCommand());// .whenReleased(new
                                                 // IntakeBitmapCommand(subsystems.intakeSubsystem,
                                                 // subsystems.indexSubsystem));
        // intakeExtendButton.whenPressed(new IntakeExtendCommand(subsystems.intakeSubsystem));
        for (Button b : intakeSpitButton)
            b.whileHeld(new SpitBallCommand());
        intakeRetractButton.whenPressed(new IntakeRetractCommand());
    }

    public void bindShooterControls() {
        if (!Subsystems.SubsystemConstants.SHOOTER_TESTING) {

            BooleanSupplier interrupt = driveController.getDPadButton(Direction.UP)::get;
            driveController.getDPadButton(Direction.DOWN).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 500, 35).withInterrupt(interrupt));
            driveController.getDPadButton(Direction.LEFT).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 2000, 15).withInterrupt(interrupt));

            // shootButton.whileHeld(
            // new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem));
            subsystems.shooterSubsystem.setDefaultCommand(
                    new ShooterTargetCommand(driveController.getLeftBumperButton()::get));
            // hoodUpButton.whileHeld(new ShooterHoodSetConstantAngleCommand(subsystems.shooterSubsystem,
            // subsystems.shooterSubsystem.getHoodAngle() + 1));
            // hoodDownButton.whileHeld(new ShooterHoodSetConstantAngleCommand(subsystems.shooterSubsystem,
            // subsystems.shooterSubsystem.getHoodAngle() - 1));
            // turretLeftButton.whileHeld(new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem,
            // subsystems.shooterSubsystem.getTurretAngle() - 5));
            // turretRightButton.whileHeld(new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem,
            // subsystems.shooterSubsystem.getTurretAngle() + 5));
        }
    }

    // Singleton
    public static final Controls instance = new Controls(Subsystems.instance);
}
