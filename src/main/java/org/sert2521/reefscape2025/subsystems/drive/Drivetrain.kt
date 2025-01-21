package org.sert2521.reefscape2025.subsystems.drive

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.ReentrantLock

object Drivetrain : SubsystemBase() {
    @JvmField
    val odometryLock:ReentrantLock = ReentrantLock()


}