package org.sert2521.reefscape2025.subsystems.dispenser

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput

object Dispenser : SubsystemBase() {
    private val io = DispenserIOSpark()
    private val ioInputs = LoggedDispenserIOInputs()

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

    @AutoLogOutput(key="Dispenser/Dispenser Beambreak")
    fun getDispenserBeambreakBlocked():Boolean{
        return !ioInputs.beambreakDispenserClear
    }

    @AutoLogOutput(key="Dispenser/Ramp Beambreak")
    fun getRampBeambreakBlocked():Boolean{
        return !ioInputs.beambreakRampClear
    }

    fun getVelocity():Double{
        return ioInputs.velocityRPM
    }

    fun runDispenserCommand():Command{
        return run{
            if (getDispenserBeambreakBlocked() || getRampBeambreakBlocked()){
                setMotor(0.4)
            }else{
                stop()
            }
        }
    }
}