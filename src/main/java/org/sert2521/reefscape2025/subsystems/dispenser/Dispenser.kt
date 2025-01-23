package org.sert2521.reefscape2025.subsystems.dispenser

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.reefscape2025.BeamState

object Dispenser : SubsystemBase() {
    val io = DispenserIOSpark()
    val ioInputs = LoggedDispenserIOInputs()

    override fun periodic() {
        io.updateInputs(ioInputs)
    }

    fun setVoltage(voltage:Double){
        io.setVoltage(voltage)
    }

    fun setMotor(speed:Double){
        io.setMotor(speed)
    }

    fun stop(){
        io.setMotor(0.0)
    }

    fun getBeambreakBlocked():Boolean{
        return !ioInputs.beambreakCleared
    }

    fun getVelocity():Double{
        return ioInputs.velocityRPM
    }

    //DOES NOT have beambreak stuff just practice
    fun voltageCommand(voltage:Double): Command {
        return runEnd({
                setVoltage(voltage)
            },
            {
                stop()
            })
    }
}