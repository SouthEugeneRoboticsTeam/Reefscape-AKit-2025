package org.sert2521.reefscape2025.commands.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.TuningConstants
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_PROFILE
import org.sert2521.reefscape2025.subsystems.elevator.Elevator


//THIS IS BORKED DO NOT USE IT
//TODO: THIS HAS BEEN REPLACED BY setElevatorCommand IN THE ELEVATOR SUBSYSTEM
class SetElevator(private val goal:Double) : Command() {
    private val motorPID = ProfiledPIDController(TuningConstants.ELEVATOR_P, TuningConstants.ELEVATOR_I, TuningConstants.ELEVATOR_D, TuningConstants.ELEVATOR_PROFILE)
    private val feedforward = ElevatorFeedforward(
        TuningConstants.ELEVATOR_S, TuningConstants.ELEVATOR_G,
        TuningConstants.ELEVATOR_V, TuningConstants.ELEVATOR_A)
    //private val unprofiled = PIDController(TuningConstants.ELEVATOR_P, TuningConstants.ELEVATOR_I, TuningConstants.ELEVATOR_D)
    private var elevatorPosition = Elevator.getPosition()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Elevator)
    }

    override fun initialize() {
        Elevator.goal = TrapezoidProfile.State(goal, 0.0)
    }

    override fun execute() {
        elevatorPosition = Elevator.getPosition()
        Elevator.setVoltage(motorPID.calculate(elevatorPosition, goal) + feedforward.calculate(motorPID.setpoint.velocity))
        //Elevator.setVoltage(unprofiled.calculate(elevatorPosition, goal) + feedforward.calculate(motorPID.setpoint.velocity))
        Logger.recordOutput("Elevator/At Setpoint", motorPID.atSetpoint())
    }

    override fun isFinished(): Boolean {
        return motorPID.atGoal()
    }
}
