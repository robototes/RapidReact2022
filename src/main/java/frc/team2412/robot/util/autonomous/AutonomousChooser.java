package frc.team2412.robot.util.autonomous;

import java.util.Map;

import com.google.errorprone.annotations.Immutable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.*;
import frc.team2412.robot.commands.autonomous.debug.LinePath;
import frc.team2412.robot.commands.autonomous.debug.SquarePath;
import frc.team2412.robot.commands.autonomous.debug.StarPath;
import frc.team2412.robot.commands.climb.ClimbExtendSlowlyCommand;
import frc.team2412.robot.commands.climb.ClimbRetractSlowlyCommand;
import frc.team2412.robot.commands.climb.ClimbTestCommand;
import frc.team2412.robot.commands.diagnostic.DiagnosticIntakeCommandGroup;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.index.IndexTestCommand;
import frc.team2412.robot.commands.intake.*;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;

public class AutonomousChooser {

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    public final Subsystems subsystems;
    public final NetworkTableEntry delayedStartEntry;
    public final NetworkTableEntry setupImage;

    public AutonomousChooser(Subsystems subsystems) {
        this.subsystems = subsystems;

        boolean setDefault = false;
        for (var mode : AutonomousMode.values()) {
            if (mode.enabled) {
                if (!setDefault) {
                    autonomousModeChooser.setDefaultOption(mode.uiName, mode);
                    setDefault = true;
                } else
                    autonomousModeChooser.addOption(mode.uiName, mode);
            }
        }

        ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

        autonomousTab.add("Choose Auto Mode", autonomousModeChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        setupImage = autonomousTab.add("Auto Setup", "").withWidget("Image")
                .withProperties(Map.of("Keep Aspect Ratio", true)).withSize(5, 3).withPosition(4, 0).getEntry();

        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
                .getSubTable("Choose Auto Mode").addEntryListener((table, key, entry, value, flags) -> {
                    System.out.println(value.getString());
                    setupImage.setString(autonomousModeChooser.getSelected().setupImage);
                }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

        delayedStartEntry = autonomousTab.add("Delay start", 0.0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    }

    public void scheduleCommand() {
        AutonomousMode autoMode = autonomousModeChooser.getSelected();
        if (autoMode != null) {
            autoMode.commandSupplier.getCommand(subsystems).schedule();
        }
    }

    public Pose2d getStartPose() {
        return autonomousModeChooser.getSelected().startPose;
    }

    @Immutable
    @FunctionalInterface
    private interface CommandSupplier {
        CommandBase getCommand(Subsystems subsystems);
    }

    private static boolean allEnabled = Subsystems.SubsystemConstants.INDEX_ENABLED &&
            Subsystems.SubsystemConstants.INTAKE_ENABLED &&
            Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
            Subsystems.SubsystemConstants.SHOOTER_VISION_ENABLED &&
            Subsystems.SubsystemConstants.DRIVE_ENABLED &&
            Subsystems.SubsystemConstants.INTAKE_ENABLED;
    private static String imagesPath = "C:/Users/Public/Pictures/RRSetupReference/";

    public enum AutonomousMode {
        // Replace with individual testing commands
        ONE_BALL(
                (subsystems) -> new OneBallAutoCommand(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem, subsystems.targetLocalizer, subsystems.drivebaseSubsystem,
                        subsystems.intakeSubsystem),
                "Y One ball auto Y",
                Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
                        Subsystems.SubsystemConstants.DRIVE_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED),
        TWO_BALL_LEFT(
                (subsystems) -> new TwoBallAutoCommandLeft(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem,
                        subsystems.targetLocalizer, subsystems.drivebaseSubsystem,
                        subsystems.intakeSubsystem),
                "Y Two ball auto left Y",
                allEnabled,
                new Pose2d(new Translation2d(397.308, 122.461), Rotation2d.fromDegrees(316.877)),
                imagesPath + "jackTwoBallLeft.png"),
        TWO_BALL_MIDDLE(
                (subsystems) -> new TwoBallAutoCommandMiddle(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem,
                        subsystems.targetLocalizer, subsystems.drivebaseSubsystem,
                        subsystems.intakeSubsystem),
                "Y Two ball auto middle Y",
                allEnabled,
                new Pose2d(new Translation2d(381.791, 211.487), Rotation2d.fromDegrees(25)),
                imagesPath + "jackTwoBallMiddle.png"),
        TWO_BALL_RIGHT(
                (subsystems) -> new TwoBallAutoCommandRight(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem,
                        subsystems.targetLocalizer, subsystems.drivebaseSubsystem,
                        subsystems.intakeSubsystem),
                "Y Two ball auto right Y",
                allEnabled,
                new Pose2d(new Translation2d(341, 250.434), Rotation2d.fromDegrees(90)),
                imagesPath + "jackTwoBallRight.png"),
        TWO_BALL_FENDER(
                (subsystems) -> new TwoBallFenderAutoCommand(subsystems.drivebaseSubsystem,
                        subsystems.shooterSubsystem),
                "X Two ball fender path X",
                allEnabled,
                new Pose2d(new Translation2d(231.8, 200.8), Rotation2d.fromDegrees(46)),
                imagesPath + "twoBallFender.png"),
        JACK_FIVE_BALL(
                (subsystems) -> new JackFiveBallAutoCommand(subsystems.drivebaseSubsystem, subsystems.intakeSubsystem,
                        subsystems.indexSubsystem,
                        subsystems.shooterSubsystem, subsystems.targetLocalizer),
                "Y 2910 Five ball path Y", Subsystems.SubsystemConstants.DRIVE_ENABLED
                        && Subsystems.SubsystemConstants.INTAKE_ENABLED
                        && Subsystems.SubsystemConstants.INDEX_ENABLED
                        && Subsystems.SubsystemConstants.SHOOTER_ENABLED,
                new Pose2d(new Translation2d(328, 75.551), Rotation2d.fromDegrees(-90)),
                imagesPath + "jackFiveBall.png"),
        WPI_PATH(
                (subsystems) -> new WPILibFiveBallAutoCommand(subsystems.drivebaseSubsystem, subsystems.intakeSubsystem,
                        subsystems.indexSubsystem, subsystems.shooterSubsystem, subsystems.targetLocalizer),
                "X WPILib Five ball path X", Subsystems.SubsystemConstants.DRIVE_ENABLED,
                new Pose2d(new Translation2d(331, 71), Rotation2d.fromDegrees(0)),
                imagesPath + "fiveBall.png"),
        SQUARE_PATH((subsystems) -> new SquarePath(subsystems.drivebaseSubsystem),
                "Square Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        LINE_PATH((subsystems) -> new LinePath(subsystems.drivebaseSubsystem),
                "Line Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        STAR_PATH((subsystems) -> new StarPath(subsystems.drivebaseSubsystem),
                "Star Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        CLIMB((subsystems) -> new ClimbTestCommand(subsystems.climbSubsystem),
                "Climb test", Subsystems.SubsystemConstants.CLIMB_ENABLED),
        INDEX((subsystems) -> new IndexTestCommand(subsystems.indexSubsystem),
                "Index test", Subsystems.SubsystemConstants.INDEX_ENABLED),
        INTAKE((subsystems) -> new DiagnosticIntakeCommandGroup(subsystems.intakeSubsystem),
                "Intake test", Subsystems.SubsystemConstants.INTAKE_ENABLED),
        SHOOTER((subsystems) -> new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem,
                subsystems.shooterSubsystem.getTurretTestAngle()), "Shooter test",
                Subsystems.SubsystemConstants.SHOOTER_ENABLED),
        CLIMB_DOWN_IN_QUEUE(
                (subsystems) -> new ClimbRetractSlowlyCommand(subsystems.climbSubsystem,
                        subsystems.intakeSubsystem, subsystems.indexSubsystem, subsystems.shooterSubsystem,
                        subsystems.drivebaseSubsystem),
                "Climb down in queue",
                Subsystems.SubsystemConstants.CLIMB_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED),
        INTAKE_SHOOTER(
                (subsystems) -> new IndexShootCommand(subsystems.indexSubsystem).alongWith(
                        new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.targetLocalizer)
                                .alongWith(new IntakeSetInCommand(subsystems.intakeSubsystem))),
                "Intake and shoot",
                Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED),
        CLIMB_UP_IN_QUEUE(
                (subsystems) -> new ClimbExtendSlowlyCommand(subsystems.climbSubsystem,
                        subsystems.intakeSubsystem, subsystems.indexSubsystem, subsystems.shooterSubsystem),
                "Climb up in queue",
                Subsystems.SubsystemConstants.CLIMB_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED);

        public final CommandSupplier commandSupplier;
        public final String uiName;
        public final boolean enabled;
        public final Pose2d startPose;
        public final String setupImage;

        private AutonomousMode(CommandSupplier commandSupplier, String uiName, boolean enabled, Pose2d startPose,
                String setupImage) {
            this.commandSupplier = commandSupplier;
            this.uiName = uiName;
            this.enabled = enabled;
            this.startPose = startPose;
            this.setupImage = setupImage;
        }

        private AutonomousMode(CommandSupplier commandSupplier, String uiName, boolean enabled, Pose2d startPose) {
            this.commandSupplier = commandSupplier;
            this.uiName = uiName;
            this.enabled = enabled;
            this.startPose = startPose;
            this.setupImage = imagesPath + "imgnotfound.png";
        }

        private AutonomousMode(CommandSupplier commandSupplier, String uiName, boolean enabled) {
            this.commandSupplier = commandSupplier;
            this.uiName = uiName;
            this.enabled = enabled;
            this.startPose = new Pose2d();
            this.setupImage = imagesPath + "imgnotfound.png";
        }
    }
}
