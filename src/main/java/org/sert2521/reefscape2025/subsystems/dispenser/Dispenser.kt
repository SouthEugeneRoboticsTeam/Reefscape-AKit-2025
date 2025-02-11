package org.sert2521.reefscape2025.subsystems.dispenser

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_INTAKE_SPEED
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_OUTTAKE_SPEED

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

    fun getBlocked():Boolean{
        return !ioInputs.beambreakDispenserClear || !ioInputs.beambreakRampClear
    }

    fun getVelocity():Double{
        return ioInputs.velocityRPM
    }





    fun idleDispenserCommand():Command{
        return run{
            if (getBlocked()) {
                setMotor(DISPENSER_INTAKE_SPEED)
            } else {
                stop()
            }
        }
    }

    fun runDispenserCommand():Command{
        return run{
            setMotor(DISPENSER_INTAKE_SPEED)
        }
    }

    fun idleDispenserBeambreakNonfunctional():Command{
        return run{
            if (getRampBeambreakBlocked()){
                runDispenserCommand()
                    .until{!getRampBeambreakBlocked()}
                    .andThen(runDispenserCommand()
                        .withTimeout(0.4)
                    ).schedule()
            } else {
                stop()
            }
        }
    }

    fun idleRampBeambreakNonfunctional():Command{
        return run{
            if (getDispenserBeambreakBlocked()){
                setMotor(DISPENSER_INTAKE_SPEED)
            }
        }
    }

    fun outtakeCommand():Command{
        return run{
            setMotor(DISPENSER_OUTTAKE_SPEED)
        }
    }
}