package org.sert2521.reefscape2025.subsystems.dispenser

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_INTAKE_SPEED
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_OUTTAKE_L4
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_OUTTAKE_SLOW_SPEED
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_OUTTAKE_SPEED
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_RECENTER_SPEED_BACKWARD
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_RECENTER_SPEED_FORWARD
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_STOP_VOLTAGE
import org.sert2521.reefscape2025.subsystems.elevator.Elevator

object Dispenser : SubsystemBase() {
    private val io = when (MetaConstants.currentMode) {
        MetaConstants.Mode.REAL -> DispenserIOSpark()
        MetaConstants.Mode.SIM -> DispenserIOSim()
        MetaConstants.Mode.REPLAY -> object : DispenserIO {}
    }
    private val ioInputs = LoggedDispenserIOInputs()

    private val rampBeambreakFunctionalNT = LoggedNetworkBoolean("Dispenser/Ramp Beambreak Working", true)
    private val dispenserBeambreakFunctionNT = LoggedNetworkBoolean("Dispenser/Dispenser Beambreak Working", true)

    private val rampFunctionalTrigger = Trigger { rampBeambreakFunctionalNT.get() }
    private val dispenserFunctionalTrigger = Trigger { dispenserBeambreakFunctionNT.get() }

    private var currentBeambreakFunctionality = Pair(true, true)
    private val beambreakFunctionalToCommand = mapOf<Pair<Boolean, Boolean>, Command>(
        Pair(true, true) to idleCommand(),
        Pair(true, false) to idleDispenserBeambreakNonfunctional(),
        Pair(false, true) to idleRampBeambreakNonfunctional(),
        Pair(false, false) to run { setVoltage(DISPENSER_STOP_VOLTAGE) }
    )

    init {
        defaultCommand = idleCommand()

        rampFunctionalTrigger.onChange(runOnce {
            currentBeambreakFunctionality = Pair(rampBeambreakFunctionalNT.get(), currentBeambreakFunctionality.second)
            defaultCommand = beambreakFunctionalToCommand[currentBeambreakFunctionality]
        })
        dispenserFunctionalTrigger.onChange(runOnce {
            currentBeambreakFunctionality =
                Pair(currentBeambreakFunctionality.first, dispenserBeambreakFunctionNT.get())
            defaultCommand = beambreakFunctionalToCommand[currentBeambreakFunctionality]
        })

        Logger.registerDashboardInput(rampBeambreakFunctionalNT)
        Logger.registerDashboardInput(dispenserBeambreakFunctionNT)
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Dispenser", ioInputs)
    }

    fun setVoltage(voltage: Double) {
        io.setVoltage(voltage)
    }

    fun setMotor(speed: Double) {
        io.setMotor(speed)
    }

    fun stop() {
        io.setMotor(0.0)
    }

    @AutoLogOutput(key = "Dispenser/Dispenser Beambreak")
    fun getDispenserBeambreakBlocked(): Boolean {
        return !ioInputs.beambreakDispenserClear
    }

    @AutoLogOutput(key = "Dispenser/Ramp Beambreak")
    fun getRampBeambreakBlocked(): Boolean {
        return !ioInputs.beambreakRampClear
    }

    fun getBlocked(): Boolean {
        return (!ioInputs.beambreakRampClear && currentBeambreakFunctionality.first)
                || (!ioInputs.beambreakDispenserClear && currentBeambreakFunctionality.second)
    }

    fun getVelocity(): Double {
        return ioInputs.velocityRPM
    }

    /* === Commands === */

    fun idleCommand(): Command {
        return run {
            if (getBlocked()) {
                if (getRampBeambreakBlocked()) {
                    setMotor(DISPENSER_INTAKE_SPEED)
                } else {
                    setMotor(DISPENSER_RECENTER_SPEED_FORWARD)
                }
            } else {
                setVoltage(DISPENSER_STOP_VOLTAGE)
            }
        }
    }

    fun idleDispenserBeambreakNonfunctional(): Command {
        return run {
            if (getRampBeambreakBlocked()) {
                intakeCommand()
                    .until { !getRampBeambreakBlocked() }
                    .andThen(
                        intakeCommand()
                            .withTimeout(0.4)
                    ).schedule()
            } else {
                stop()
            }
        }
    }

    fun idleRampBeambreakNonfunctional(): Command {
        return run {
            if (getBlocked()) {
                setMotor(DISPENSER_RECENTER_SPEED_FORWARD)
            } else {
                stop()
            }
        }
    }


    fun intakeCommand(): Command {
        return run {
            setMotor(DISPENSER_INTAKE_SPEED)
        }
    }

    fun outtakeCommand(): Command {
        return run {
            if (Elevator.goal.position == SetpointConstants.ELEVATOR_L4) {
                setMotor(DISPENSER_OUTTAKE_L4)
            } else {
                setMotor(DISPENSER_OUTTAKE_SPEED)
            }
        }.withTimeout(0.2).andThen(stopCommand().withTimeout(0.4))
    }

    fun outtakeCommandNoWait(): Command {
        return run {
            if (Elevator.goal.position == SetpointConstants.ELEVATOR_L4) {
                setMotor(DISPENSER_OUTTAKE_L4)
            } else {
                setMotor(DISPENSER_OUTTAKE_SPEED)
            }
        }.withTimeout(0.2)
    }

    fun outtakeSlowCommand(): Command {
        return run {
            setMotor(DISPENSER_OUTTAKE_SLOW_SPEED)
        }
    }

    fun stopCommand(): Command {
        return run {
            setMotor(0.0)
        }
    }

    fun recenterCommand(): Command {
        return run {
            setMotor(DISPENSER_RECENTER_SPEED_BACKWARD)
        }.withTimeout(2.0).until { getBlocked() }
    }
}