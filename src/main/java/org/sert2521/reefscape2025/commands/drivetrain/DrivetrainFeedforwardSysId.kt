package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import java.text.DecimalFormat
import java.text.NumberFormat
import java.util.*


object DrivetrainFeedforwardSysId {
    val velocitySamples: MutableList<Double> = LinkedList()
    val voltageSamples: MutableList<Double> = LinkedList()
    val timer = Timer()

    fun get():Command{
        return Commands.sequence( // Reset data
            Commands.runOnce(
                {

                    velocitySamples.clear()
                    voltageSamples.clear()
                }),  // Allow modules to orient

            Commands.run(
                {
                    Drivetrain.runCharacterization(0.0)
                },
                Drivetrain
            )
                .withTimeout(1.0),

            // Start timer
            Commands.runOnce(timer::restart),  // Accelerate and gather data

            Commands.run(
                {
                    val voltage: Double = timer.get() * SwerveConstants.FF_RAMP_RATE
                    Drivetrain.runCharacterization(voltage)
                    velocitySamples.add(Drivetrain.getFFCharacterizationVelocity())
                    voltageSamples.add(voltage)
                },
                Drivetrain
            ) // When cancelled, calculate and print results

                .finallyDo(
                    Runnable {
                        val n = velocitySamples.size
                        var sumX = 0.0
                        var sumY = 0.0
                        var sumXY = 0.0
                        var sumX2 = 0.0
                        for (i in 0..<n) {
                            sumX += velocitySamples[i]
                            sumY += voltageSamples[i]
                            sumXY += velocitySamples[i] * voltageSamples[i]
                            sumX2 += velocitySamples[i] * velocitySamples[i]
                        }
                        val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
                        val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

                        val formatter: NumberFormat = DecimalFormat("#0.00000")
                        println("********** Drive FF Characterization Results **********")
                        println("\tkS: " + formatter.format(kS))
                        println("\tkV: " + formatter.format(kV))
                    })
        )
    }
}
