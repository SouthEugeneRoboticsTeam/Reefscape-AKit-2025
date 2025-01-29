package org.sert2521.reefscape2025.subsystems.grintake

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command

object GroundIntake : SubsystemBase() {
    private val io = GroundIntakeIOSpark()
    private val ioInputs = LoggedGroundIntakeIOInputs()

    override fun periodic() {
        GroundIntake.io.updateInputs(GroundIntake.ioInputs)
    }
}