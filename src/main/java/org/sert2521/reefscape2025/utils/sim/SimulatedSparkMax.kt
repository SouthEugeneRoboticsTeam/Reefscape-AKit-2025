package org.sert2521.reefscape2025.utils.sim

import com.revrobotics.spark.ClosedLoopSlot
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import kotlin.math.abs
import kotlin.math.withSign

class SimulatedSparkMax(private val model: DCMotor) : SimulatedMotorController {
    private var currentLimit: Current = Units.Amps.of(150.0)
    private var forwardSoftwareLimit: Angle = Units.Radians.of(Double.POSITIVE_INFINITY)
    private var reverseSoftwareLimit: Angle = Units.Radians.of(-Double.POSITIVE_INFINITY)

    private var requestedVoltage: Voltage = Units.Volts.zero()
    var appliedVoltage: Voltage = Units.Volts.zero()
        private set

    private var positionConversionFactor = 1.0
    private var velocityConversionFactor = 1.0

    enum class ControlMode{
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    private val pidController = PIDController(0.0, 0.0, 0.0)
    private var ffGain = 0.0

    private var closedLoopSlot = ClosedLoopSlot.kSlot0
    private var controlMode = ControlMode.VOLTAGE

    private var pidffGainsSlot0 = arrayOf(0.0, 0.0, 0.0, 0.0)
    private var pidffGainsSlot1 = arrayOf(0.0, 0.0, 0.0, 0.0)
    private var pidffGainsSlot2 = arrayOf(0.0, 0.0, 0.0, 0.0)
    private var pidffGainsSlot3 = arrayOf(0.0, 0.0, 0.0, 0.0)

    private var requestedAngle = Rotations.zero()
    private var requestedVelocity = RotationsPerSecond.zero()

    fun withCurrentLimit(currentLimit: Current): SimulatedSparkMax {
        this.currentLimit = currentLimit
        return this
    }

    fun withSoftwareLimits(forwardSoftwareLimit: Angle, reverseSoftwareLimit: Angle): SimulatedSparkMax {
        this.forwardSoftwareLimit = forwardSoftwareLimit
        this.reverseSoftwareLimit = reverseSoftwareLimit
        return this
    }

    fun withPIDController(slot: ClosedLoopSlot, p:Double, i:Double, d:Double, ff:Double): SimulatedSparkMax {
        when (slot){
            ClosedLoopSlot.kSlot0 -> pidffGainsSlot0 = arrayOf(p, i, d, ff)
            ClosedLoopSlot.kSlot1 -> pidffGainsSlot1 = arrayOf(p, i, d, ff)
            ClosedLoopSlot.kSlot2 -> pidffGainsSlot2 = arrayOf(p, i, d, ff)
            ClosedLoopSlot.kSlot3 -> pidffGainsSlot3 = arrayOf(p, i, d, ff)
        }

        return this
    }

    fun requestVoltage(voltage: Voltage) {
        requestedVoltage = voltage
        controlMode = ControlMode.VOLTAGE
    }

    fun setReference(value:Double, ctrl: ControlMode, slot:ClosedLoopSlot=closedLoopSlot, arbFF:Voltage=Volts.of(0.0)){
        this.controlMode = ctrl
        this.closedLoopSlot = slot

        when (controlMode){
            ControlMode.VOLTAGE -> {
                requestedVoltage = Volts.of(value).plus(arbFF)
            }
            ControlMode.POSITION -> {
                requestedAngle = Rotations.of(value/positionConversionFactor)
                requestedVoltage = arbFF
            }
            ControlMode.VELOCITY -> {
                requestedVelocity = RPM.of(value/velocityConversionFactor)
                requestedVoltage = arbFF
            }
        }

    }

    /**
     *
     *
     * <h2>(Utility Function) Constrains the Output Voltage of a Motor.</h2>
     *
     *
     * Constrains the output voltage of a motor such that the **stator** current does not exceed the
     * current limit
     *
     *
     * Prevents motor from exceeding software limits
     *
     * @param encoderAngle the angle of the encoder
     * @param encoderVelocity the velocity of the encoder
     * @param requestedVoltage the requested voltage
     * @return the constrained voltage that satisfied the limits
     */
    fun constrainOutputVoltage(
        encoderAngle: Angle, encoderVelocity: AngularVelocity, requestedVoltage: Voltage
    ): Voltage {
        val kCurrentThreshold = 1.2

        // don't use WpiLib Units for calculations
        val motorCurrentVelocityRadPerSec = encoderVelocity.`in`(Units.RadiansPerSecond)
        val currentLimitAmps = currentLimit.`in`(Units.Amps)
        val requestedOutputVoltageVolts = requestedVoltage.`in`(Units.Volts)
        val currentAtRequestedVoltageAmps =
            model.getCurrent(motorCurrentVelocityRadPerSec, requestedOutputVoltageVolts)

        // Resource for current limiting:
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
        var limitedVoltage = requestedOutputVoltageVolts
        val currentTooHigh =
            abs(currentAtRequestedVoltageAmps) > (kCurrentThreshold * currentLimitAmps)
        if (currentTooHigh) {
            val limitedCurrent: Double = currentLimitAmps.withSign(currentAtRequestedVoltageAmps)
            limitedVoltage = model.getVoltage(model.getTorque(limitedCurrent), motorCurrentVelocityRadPerSec)
        }

        // ensure the current limit doesn't cause an increase to output voltage
        if (abs(limitedVoltage) > abs(requestedOutputVoltageVolts)) limitedVoltage = requestedOutputVoltageVolts

        // apply software limits
        if (encoderAngle.gte(forwardSoftwareLimit) && limitedVoltage > 0) limitedVoltage = 0.0
        if (encoderAngle.lte(reverseSoftwareLimit) && limitedVoltage < 0) limitedVoltage = 0.0

        // constrain the output voltage to the battery voltage
        return Units.Volts.of(limitedVoltage)
    }

    override fun updateControlSignal(
        mechanismAngle: Angle?,
        mechanismVelocity: AngularVelocity?,
        encoderAngle: Angle,
        encoderVelocity: AngularVelocity
    ): Voltage {
        when (closedLoopSlot){
            ClosedLoopSlot.kSlot0 -> {
                pidController.setPID(pidffGainsSlot0[0], pidffGainsSlot0[1], pidffGainsSlot0[2])
                ffGain = pidffGainsSlot0[3]
            }
            ClosedLoopSlot.kSlot1 -> {
                pidController.setPID(pidffGainsSlot1[0], pidffGainsSlot1[1], pidffGainsSlot1[2])
                ffGain = pidffGainsSlot1[3]
            }
            ClosedLoopSlot.kSlot2 -> {
                pidController.setPID(pidffGainsSlot2[0], pidffGainsSlot2[1], pidffGainsSlot2[2])
                ffGain = pidffGainsSlot2[3]
            }
            ClosedLoopSlot.kSlot3 -> {
                pidController.setPID(pidffGainsSlot3[0], pidffGainsSlot3[1], pidffGainsSlot3[2])
                ffGain = pidffGainsSlot3[3]
            }
        }

        val finalRequestedVoltage = when (controlMode) {
            ControlMode.VOLTAGE -> requestedVoltage
            ControlMode.POSITION -> {
                requestedVoltage + Volts.of(
                    pidController.calculate(encoderAngle.`in`(Rotations), requestedAngle.`in`(Rotations))
                        * SimulatedBattery.getBatteryVoltage().`in`(Volts) // Multiply by battery voltage because PID on spark is in duty cycle
                )

            }
            ControlMode.VELOCITY -> {
                requestedVoltage + Volts.of(
                    pidController.calculate(encoderVelocity.`in`(RPM), requestedVelocity.`in`(RPM))
                        * SimulatedBattery.getBatteryVoltage().`in`(Volts)
                )
            }
        }

        appliedVoltage = constrainOutputVoltage(encoderAngle, encoderVelocity, finalRequestedVoltage)
        return appliedVoltage
    }
}