package org.sert2521.reefscape2025.subsystems.wrist

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.SetpointConstants.WRIST_INIT
import org.sert2521.reefscape2025.SetpointConstants.WRIST_STOW

object Wrist : SubsystemBase() {
    private val io = WristIOSpark()
    private val ioInputs = LoggedWristIOInputs()

    init{
        defaultCommand = holdWristCommand()
    }

    override fun periodic(){
        io.updateInputs(ioInputs)
        Logger.processInputs("Wrist/Pivot", ioInputs)
    }

    fun setVoltage(voltage:Double){
        io.setVoltage(voltage)
    }

    fun getRotations():Double{
        return ioInputs.wristAbsPosition
    }

    fun setReference(goal:Double){
        io.setReference(goal)
    }

    fun stop(){
        io.setVoltage(0.0)
    }

    fun setWristCommand(goal:Double): Command {
        return runOnce{
            io.setReference(goal)
        }.andThen(Commands.waitUntil{
            MathUtil.isNear(goal, getRotations(), 0.1)
        })
    }

    private fun holdWristCommand():Command{
        return Commands.none()
    }

    fun initWristCommand():Command{
        return runOnce{
            io.resetMotorEncoder()
            io.setReference(WRIST_INIT)
        }
            .andThen(Commands.waitUntil{
                MathUtil.isNear(WRIST_INIT, getRotations(), 0.1)
            })
            .andThen(Commands.waitSeconds(1.0))
            .andThen(runOnce{ io.resetMotorEncoder() })
            .andThen(setWristCommand(WRIST_STOW))
    }
}