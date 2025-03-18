package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.MotorSafety
import org.team9432.annotation.Logged

//get translated idiot

interface ModuleIO {

    @Logged
    open class ModuleIOInputs{
        var driveConnected = false
        var drivePositionRad = 0.0
        var driveVelocityRadPerSec = 0.0
        var driveAppliedVolts = 0.0
        var driveCurrentAmps = 0.0

        var turnConnected = false
        var turnPositionMotor = Rotation2d()
        var turnPositionAbsolute = Rotation2d()
        var turnVelocityRadPerSec = 0.0
        var turnAppliedVolts = 0.0
        var turnCurrentAmps = 0.0

        var odometryTimestamps = doubleArrayOf()
        var odometryDrivePositionsRad = doubleArrayOf()
        var odometryTurnPositions:Array<Rotation2d> = arrayOf()
    }

    fun updateInputs(inputs:ModuleIOInputs)

    fun setDriveOpenLoop(output:Double)

    fun setTurnOpenLoop(output:Double)

    fun setDriveVelocity(velocityRadPerSec:Double, withPID:Boolean = true)

    fun setTurnPosition(rotation:Rotation2d)

    fun updateTurnEncoder(rotation:Rotation2d)

    fun setCurrentLimit(limit:Int)
}