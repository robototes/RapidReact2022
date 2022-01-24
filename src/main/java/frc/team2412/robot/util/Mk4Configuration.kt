package frc.team2412.robot.util

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper
import com.swervedrivespecialties.swervelib.SwerveModule

//kotlin is very fun
data class Mk4Configuration(val ratio: Mk4SwerveModuleHelper.GearRatio, val drive: Int, val angle: Int, val encoder: Int, val offset: Double) {
    fun falcons(): SwerveModule {
        return Mk4SwerveModuleHelper.createFalcon500(ratio, drive, angle, encoder, offset);
    }
}
