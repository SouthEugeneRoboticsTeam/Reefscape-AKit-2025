package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_PROFILE
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants

object Elevator : SubsystemBase() {
    private val io = ElevatorIOSpark()
    private val ioInputs = LoggedElevatorIOInputs()

    private val profile = TrapezoidProfile(ELEVATOR_PROFILE)

    var goal  = TrapezoidProfile.State(0.0, 0.0)
    private var currentState = TrapezoidProfile.State(0.0, 0.0)

    init{
        defaultCommand = holdElevatorCommand()
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Elevator", ioInputs)

        Logger.recordOutput("Elevator/Processed Laser", getPosition())

//        if (downFilter.calculate(ioInputs.laserPosition<0.03)){
//            io.setEncoder(0.0)
//        }
    }

    fun setVoltage(voltage:Double){
        io.setVoltage(voltage)
    }

    fun getVelocity():Double {
        return ioInputs.motorsVelocity
    }

    fun getPosition():Double{
        return ioInputs.motorsPosition
    }

    fun getMotorPosition():Double{
        return ioInputs.motorsPosition
    }

    fun getAccelLimit():Double{
        return MathUtil.interpolate(
            SwerveConstants.DRIVE_ACCEL_FAST, SwerveConstants.DRIVE_ACCEL_SLOW,
            getPosition()/SetpointConstants.ELEVATOR_L4)
    }

    fun getDeccelLimit():Double{
        return MathUtil.interpolate(
            SwerveConstants.DRIVE_DECCEL_FAST, SwerveConstants.DRIVE_DECCEL_SLOW,
            getPosition()/SetpointConstants.ELEVATOR_L4)
    }

    fun getSpeedLimit():Double{
        return MathUtil.interpolate(
            SwerveConstants.DRIVE_SPEED_FAST, SwerveConstants.DRIVE_SPEED_SLOW,
            getPosition()/SetpointConstants.ELEVATOR_L4)
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
                Logger.recordOutput("Elevator/Reference Position", currentState.position)
                Logger.recordOutput("Elevator/Reference Velocity", currentState.velocity)

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
            .until{Dispenser.getBlocked()}
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
            Logger.recordOutput("Elevator/Reference Position", currentState.position)
            Logger.recordOutput("Elevator/Reference Velocity", currentState.velocity)
        }
    }
}