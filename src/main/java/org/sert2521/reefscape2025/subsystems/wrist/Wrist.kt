package org.sert2521.reefscape2025.subsystems.wrist

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.SetpointConstants.WRIST_INIT
import org.sert2521.reefscape2025.SetpointConstants.WRIST_STOW

object Wrist : SubsystemBase() {
    private val io = when (MetaConstants.currentMode){
        MetaConstants.Mode.REAL -> WristIOSpark()
        MetaConstants.Mode.SIM -> WristIOSim()
        MetaConstants.Mode.REPLAY -> object:WristIO{}
    }
    private val ioInputs = LoggedWristIOInputs()

    private val mechanism = Mechanism2d(0.9017, 0.9017)
    private val root = mechanism.getRoot("Wrist Root", 0.285750, 0.0)
    private val wristMain = MechanismLigament2d("Wrist Main", 0.0, 100.0, 0.0, Color8Bit())
    private val wristTop = MechanismLigament2d("Wrist Top", 0.42, 100.0 - 111.612575184)
    private val wristBottom = MechanismLigament2d("Wrist Bottom", 0.348, 100.0 - 86.8856310843)

    var goal = WRIST_STOW

    init{
        defaultCommand = holdWristCommand()

        wristMain.append(wristTop)
        wristMain.append(wristBottom)
        root.append(wristMain)
    }

    override fun periodic(){
        io.updateInputs(ioInputs)
        Logger.processInputs("Wrist/Pivot", ioInputs)

        wristMain.angle = -getRotations()*360 + 180
        SmartDashboard.putData("Wrist Mechanism", mechanism)
    }

    fun setVoltage(voltage:Double){
        io.setVoltage(voltage)
    }

    fun getRotations():Double{
        return ioInputs.wristAbsPosition
    }

    fun setReference(goal:Double){
        io.setReferenceFast(goal)
    }

    fun stop(){
        io.setVoltage(0.0)
    }

    fun setWristCommandFast(goal:Double): Command {

        return runOnce{
            this.goal = goal
            io.setReferenceFast(goal)
        }.andThen(Commands.waitUntil{
            MathUtil.isNear(goal, getRotations(), 0.1)
        })
    }

    fun setWristCommandSlow(goal:Double):Command{
        return runOnce{
            this.goal = goal
            io.setReferenceSlow(goal)
        }.andThen(Commands.waitUntil{
            MathUtil.isNear(goal, getRotations(), 0.1)
        })
    }

    private fun holdWristCommand():Command{
        return runOnce {}
    }

    fun initWristCommand():Command{
        return Commands.sequence(
            runOnce{
                io.resetMotorEncoder()
                io.setReferenceFast(WRIST_INIT)
            },
            Commands.waitUntil{
                MathUtil.isNear(WRIST_INIT, getRotations(), 0.1)
            },
            Commands.waitSeconds(0.2),
            runOnce{ io.resetMotorEncoder() },
            setWristCommandFast(WRIST_STOW)
        )
    }
}