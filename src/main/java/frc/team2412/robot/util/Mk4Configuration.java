package frc.team2412.robot.util;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class Mk4Configuration {

    private final Mk4SwerveModuleHelper.GearRatio ratio;
    private final int drive, angle, encoder;
    private final double offset;
    private final String canBus;

    public Mk4Configuration(
            Mk4SwerveModuleHelper.GearRatio rat,
            int dr,
            int ang,
            int enc,
            double off,
            String canBusName) {
        ratio = rat;
        drive = dr;
        angle = ang;
        encoder = enc;
        offset = off;
        canBus = canBusName;
    }

    public Mk4Configuration(
            Mk4SwerveModuleHelper.GearRatio rat, int dr, int ang, int enc, double off) {
        this(rat, dr, ang, enc, off, "rio");
    }

    public SwerveModule create() {
        return create(true);
    }

    public SwerveModule create(boolean supportAbsoluteEncoder) {
        return Mk4SwerveModuleHelper.createFalcon500(
                ratio,
                drive,
                angle,
                supportAbsoluteEncoder ? encoder : -1,
                supportAbsoluteEncoder ? offset : 0,
                canBus);
    }

    public Mk4SwerveModuleHelper.GearRatio getRatio() {
        return ratio;
    }

    public int getDrive() {
        return drive;
    }

    public int getAngle() {
        return angle;
    }

    public int getEncoder() {
        return encoder;
    }

    public double getOffset() {
        return offset;
    }
}
