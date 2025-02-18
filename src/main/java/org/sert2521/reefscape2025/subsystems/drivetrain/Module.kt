package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import org.littletonrobotics.junction.Logger

class Module(val index:Int) {
    val io = ModuleIOSpark(index)
    val inputs = LoggedModuleIOInputs()
    private val indexToCorner = SwerveConstants.indexToCorner

    val driveDisconnectedAlert = Alert(
        "Disconnected drive motor on "+indexToCorner[index]+" module.",
        Alert.AlertType.kError
    )
    val turnDisconnectedAlert = Alert(
        "Disconnected angle motor on "+indexToCorner[index]+" module.",
        Alert.AlertType.kError
    )
    var turnInited = false

    private var odometryPositions = arrayOf<SwerveModulePosition>()

    init{

    }

    fun periodic(){
        io.updateInputs(inputs)
        Logger.processInputs("Drive/"+indexToCorner[index]+" Module", inputs)

        val sampleCount = inputs.odometryTimestamps.size
        odometryPositions = Array<SwerveModulePosition>(sampleCount) {
            SwerveModulePosition(
                inputs.odometryDrivePositionsRad[it] * SwerveConstants.WHEEL_RADIUS_METERS,
                inputs.odometryTurnPositions[it]
            )
        }

        driveDisconnectedAlert.set(!inputs.driveConnected)
        turnDisconnectedAlert.set(!inputs.turnConnected)

        if ((inputs.turnConnected &&
            !MathUtil.isNear(inputs.turnPositionAbsolute.degrees,
                inputs.turnPositionMotor.degrees, 0.5))
            || !turnInited){

            io.updateTurnEncoder(inputs.turnPositionAbsolute)
        }
    }

    fun runSetpoint(state:SwerveModuleState):SwerveModuleState{
        /**
         * Returns the optimized swerve module state for logging purposes
         **/

        state.optimize(getAngle())
        state.cosineScale(getAngle())

        io.setDriveVelocity(state.speedMetersPerSecond/SwerveConstants.WHEEL_RADIUS_METERS)
        io.setTurnPosition(state.angle)

        return state
    }

    fun runCharacterization(output:Double){
        io.setDriveOpenLoop(output)
        io.setTurnPosition(Rotation2d())
    }

    fun stop(){
        io.setDriveOpenLoop(0.0)
        io.setTurnOpenLoop(0.0)
    }

    fun getAngle():Rotation2d{
        return inputs.turnPositionMotor
    }

    fun getPositionMeters():Double{
        return inputs.drivePositionRad * SwerveConstants.WHEEL_RADIUS_METERS
    }

    fun getVelocityMetersPerSec():Double{
        return inputs.driveVelocityRadPerSec * SwerveConstants.WHEEL_RADIUS_METERS
    }

    fun getPosition():SwerveModulePosition{
        return SwerveModulePosition(getPositionMeters(), getAngle())
    }

    fun getState():SwerveModuleState{
        return SwerveModuleState(getVelocityMetersPerSec(), getAngle())
    }

    fun getOdometryPositions():Array<SwerveModulePosition>{
        return odometryPositions
    }

    fun getOdometryTimestamps():DoubleArray{
        return inputs.odometryTimestamps
    }

    fun getWheelRadiusCharacterizationPosition():Double{
        return inputs.drivePositionRad
    }

    fun getFFCharacterizationVelocity():Double{
        return inputs.driveVelocityRadPerSec
    }
}