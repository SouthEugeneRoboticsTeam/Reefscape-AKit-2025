package org.sert2521.reefscape2025.subsystems.grintake

import edu.wpi.first.wpilibj2.command.SubsystemBase;

object GroundIntake : SubsystemBase() {
    private val io = GroundIntakeIOSpark()
    private val ioInputs = LoggedGroundIntakeIOInputs()

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


    fun getVelocity():Double{
        return ioInputs.velocityRPM
    }
}