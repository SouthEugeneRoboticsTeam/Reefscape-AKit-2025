package org.sert2521.reefscape2025.subsystems.wrist

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.commands.wrist.HoldWrist

object Wrist : SubsystemBase() {
    private val io = WristIOSpark()
    private val ioInputs = LoggedWristIOInputs()

    var goal = SetpointConstants.WRIST_STOW

    init{
        defaultCommand = HoldWrist()
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

    fun stop(){
        io.setVoltage(0.0)
    }
}