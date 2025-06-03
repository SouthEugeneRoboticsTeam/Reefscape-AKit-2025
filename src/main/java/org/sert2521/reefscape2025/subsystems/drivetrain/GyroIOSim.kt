package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import org.ironmaple.simulation.drivesims.GyroSimulation

class GyroIOSim(private val gyroSimulation: GyroSimulation) : GyroIO {
    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = true
        inputs.yawPosition = gyroSimulation.gyroReading
        inputs.yawVelocityRadPerSec = gyroSimulation.measuredAngularVelocity.`in`(RadiansPerSecond)
    }
}