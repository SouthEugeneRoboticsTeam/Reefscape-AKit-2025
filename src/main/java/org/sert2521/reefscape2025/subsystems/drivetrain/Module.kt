package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import org.littletonrobotics.junction.Logger

class Module(private val index:Int) {
    private val io = ModuleIOSpark(index)
    private val inputs = LoggedModuleIOInputs()
    private val indexToCorner = SwerveConstants.indexToCorner

    private val driveDisconnectedAlert = Alert(
        "Disconnected drive motor on "+indexToCorner[index]+" module.",
        Alert.AlertType.kError
    )
    private val turnDisconnectedAlert = Alert(
        "Disconnected angle motor on "+indexToCorner[index]+" module.",
        Alert.AlertType.kError
    )
    private var turnInited = false

    init{

    }

    fun periodic(){
        io.updateInputs(inputs)
        Logger.processInputs("Drive/"+indexToCorner[index]+" Module", inputs)

        driveDisconnectedAlert.set(!inputs.driveConnected)
        turnDisconnectedAlert.set(!inputs.turnConnected)

        if ((inputs.turnConnected &&
            !MathUtil.isNear(inputs.turnPositionAbsolute.degrees,
                inputs.turnPositionMotor.degrees, 0.5))
            || !turnInited){

            io.updateTurnEncoder(inputs.turnPositionAbsolute)
            turnInited = true
        }
    }

    fun runSetpoint(state:SwerveModuleState, withPID:Boolean = true):SwerveModuleState{
        /**
         * Returns the optimized swerve module state for logging purposes
         **/

        state.optimize(getAngle())
        state.cosineScale(getAngle())

        io.setDriveVelocity(state.speedMetersPerSecond/SwerveConstants.WHEEL_RADIUS_METERS, withPID)
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

    fun setCurrentLimit(limit:Int){
        io.setCurrentLimit(limit)
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

    fun getWheelRadiusCharacterizationPosition():Double{
        return inputs.drivePositionRad
    }

    fun getFFCharacterizationVelocity():Double{
        return inputs.driveVelocityRadPerSec
    }
}