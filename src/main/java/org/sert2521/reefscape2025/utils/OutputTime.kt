package org.sert2521.reefscape2025.utils

import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger

class OutputTime(private val key:String) : Command() {
    private var startTime = Logger.getTimestamp()

    override fun initialize() {
        startTime = Logger.getTimestamp()
    }

    override fun end(interrupted: Boolean) {
        Logger.recordOutput(key, Logger.getTimestamp()-startTime)
    }
}
