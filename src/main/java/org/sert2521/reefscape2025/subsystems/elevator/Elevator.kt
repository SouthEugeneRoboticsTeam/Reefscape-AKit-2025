package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_PROFILE
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

object Elevator : SubsystemBase() {
    private val io = ElevatorIOSpark()
    private val ioInputs = LoggedElevatorIOInputs()

    private val profile = TrapezoidProfile(ELEVATOR_PROFILE)

    var goal  = TrapezoidProfile.State(0.0, 0.0)
    private var currentState = TrapezoidProfile.State(0.0, 0.0)

    private val mechanism = Mechanism2d(3.0, 3.0)
    private val mechanismRoot = mechanism.getRoot("Elevator",0.1, 0.1)
    private val elevatorBase = MechanismLigament2d("Elevator Base", 0.1, 0.0)
    private val firstStage = MechanismLigament2d("First Stage", 0.2, 0.0)
    private val secondStage = MechanismLigament2d("Second Stage", 0.3, 0.0)

    init{
        defaultCommand = holdElevatorCommand()

        mechanismRoot.append(elevatorBase)
        mechanismRoot.append(firstStage)
        mechanismRoot.append(secondStage)
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Elevator", ioInputs)
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