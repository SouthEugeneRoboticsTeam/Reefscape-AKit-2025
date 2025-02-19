package org.sert2521.reefscape2025.subsystems.wrist

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants

object Wrist : SubsystemBase() {
    private val io = WristIOSpark()
    private val ioInputs = LoggedWristIOInputs()

    var goal = SetpointConstants.WRIST_STOW

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

    fun getRadians():Double{
        return ioInputs.wristPosition
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
        }
    }

    private fun holdWristCommand():Command{
        return run{}
    }
}