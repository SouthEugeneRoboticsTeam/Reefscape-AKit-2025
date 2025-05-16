package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_PROFILE
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

object Elevator : SubsystemBase() {
    private val io = when (MetaConstants.currentMode){
        MetaConstants.Mode.REAL -> ElevatorIOSpark()
        MetaConstants.Mode.SIM -> ElevatorIOSim()
        MetaConstants.Mode.REPLAY -> object:ElevatorIO{}
    }
    private val ioInputs = LoggedElevatorIOInputs()

    private val profile = TrapezoidProfile(ELEVATOR_PROFILE)

    var goal  = TrapezoidProfile.State(0.0, 0.0)
    private var currentState = TrapezoidProfile.State(0.0, 0.0)

    private val mechanism = Mechanism2d(0.901700, 1.0)
    private val mechanismRoot = mechanism.getRoot("Elevator",0.488950, 0.0)
    private val firstStage = MechanismLigament2d("First Stage", 1.060526, 90.0, 6.0, Color8Bit(0, 0, 150))
    private val connector = MechanismLigament2d("Connector", 0.02, 0.0, 0.0, Color8Bit())
    private val secondStage = MechanismLigament2d("Second Stage", 0.374865, 90.0, 5.0, Color8Bit(150, 0, 0))

    init{
        defaultCommand = holdElevatorCommand()

        mechanismRoot.append(firstStage)
        mechanismRoot.append(connector)
        connector.append(secondStage)

        SmartDashboard.putData("Elevator Mechanism", mechanism)
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Elevator", ioInputs)

        firstStage.length = 1.060526 + ioInputs.positionMeters
        secondStage.length = 0.374865 + 2 * ioInputs.positionMeters
    }

    fun setVoltage(voltage:Double){
        io.setVoltage(voltage)
    }

    fun getVelocity():Double {
        return ioInputs.velocityMetersPerSec
    }

    fun getPosition():Double{
        return ioInputs.positionMeters
    }

    fun stop() {
        io.setVoltage(0.0)
    }

    fun setElevatorCommand(goalMeters:Double): Command {
        /* Sets the goal of the motion profile, and then executes the calculations required
            then sets the motor setpoint to the result */
        return startRun({
                goal.position = goalMeters
                goal.velocity = 0.0
            },
            {
                currentState = profile.calculate(0.02, currentState, goal)

                io.setReference(currentState.position, currentState.velocity)

                Logger.recordOutput("Elevator/At Setpoint",
                    MathUtil.isNear(currentState.position, getPosition(), 0.05))
            }
        ).until{
            profile.isFinished(0.0)
        }
    }

    fun setElevatorSafeCommand(goalMeters: Double):Command{
        return Commands.waitUntil{!Dispenser.getBlocked()}
            .andThen(Elevator.setElevatorCommand(goalMeters))
    }

    private fun holdElevatorCommand():Command{
        // If it's at stow, then set the voltage to 0
        // Otherwise just run the profile without a new goal until another goal is set
        return run{
            if (goal.position == SetpointConstants.ELEVATOR_STOW){
                io.setVoltage(0.0)
            } else {
                io.setReference(currentState.position, currentState.velocity)
            }

            Logger.recordOutput("Elevator/At Setpoint",
                MathUtil.isNear(currentState.position, getPosition(), 0.05))
        }
    }
}