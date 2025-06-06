package org.sert2521.reefscape2025.subsystems.wrist

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.SetpointConstants.WRIST_INIT
import org.sert2521.reefscape2025.SetpointConstants.WRIST_STOW
import kotlin.math.PI

object Wrist : SubsystemBase() {
    private val io = when (MetaConstants.currentMode) {
        MetaConstants.Mode.REAL -> WristIOSpark()
        MetaConstants.Mode.SIM -> WristIOSim()
        MetaConstants.Mode.REPLAY -> object : WristIO {}
    }
    private val ioInputs = LoggedWristIOInputs()

    private val mechanism = LoggedMechanism2d(0.0, 0.0, Color8Bit("#FF0000"))
    private val root = mechanism.getRoot("Wrist Root", -0.285750, 0.247650)
    private val wristMain = LoggedMechanismLigament2d("Wrist Main", 0.0, 100.0, 0.0, Color8Bit())
    private val wristTop = LoggedMechanismLigament2d("Wrist Top", 0.42, 100.0 - 111.612575184 - 11.4592, 4.0, Color8Bit("#00D000"))
    private val wristBottom = LoggedMechanismLigament2d("Wrist Bottom", 0.348, 100.0 - 86.8856310843 - 11.4592, 4.0,
        Color8Bit("#00D000")
    )

    var goal = WRIST_STOW

    init {
        defaultCommand = holdWristCommand()

        wristMain.append(wristTop)
        wristMain.append(wristBottom)
        root.append(wristMain)
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Wrist/Pivot", ioInputs)

        wristMain.angle = -getRotations() * 360 + 180

        val wristPose = Pose3d(-0.285750, 0.0, 0.247650, Rotation3d(0.0, getRotations() * 2 * PI + 0.42 + 0.2, 0.0))
        Logger.recordOutput("Mechanism2d/Wrist", mechanism)
        Logger.recordOutput("Mechanism3d/Wrist", wristPose)
    }

    fun setVoltage(voltage: Double) {
        io.setVoltage(voltage)
    }

    fun getRotations(): Double {
        return ioInputs.wristAbsPosition
    }

    fun setReference(goal: Double) {
        io.setReferenceFast(goal)
    }

    fun stop() {
        io.setVoltage(0.0)
    }

    fun setWristCommandFast(goal: Double): Command {

        return runOnce {
            this.goal = goal
            io.setReferenceFast(goal)
        }.andThen(Commands.waitUntil {
            MathUtil.isNear(goal, getRotations(), 0.1)
        })
    }

    fun setWristCommandSlow(goal: Double): Command {
        return runOnce {
            this.goal = goal
            io.setReferenceSlow(goal)
        }.andThen(Commands.waitUntil {
            MathUtil.isNear(goal, getRotations(), 0.1)
        })
    }

    private fun holdWristCommand(): Command {
        return runOnce {}
    }

    fun initWristCommand(): Command {
        return Commands.sequence(
            runOnce {
                io.resetMotorEncoder()
                io.setReferenceFast(WRIST_INIT)
            },
            Commands.waitUntil {
                MathUtil.isNear(WRIST_INIT, getRotations(), 0.1)
            },
            Commands.waitSeconds(0.2),
            runOnce { io.resetMotorEncoder() },
            setWristCommandFast(WRIST_STOW)
        )
    }
}