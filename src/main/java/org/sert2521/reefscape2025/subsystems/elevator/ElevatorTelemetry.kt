package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.util.Color8Bit
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d

class ElevatorTelemetry(private val mechanism2dEnabled: Boolean, private val mechanism3dEnabled: Boolean) {
    private val mechanism: LoggedMechanism2d?
    private val mechanismRoot: LoggedMechanismRoot2d?
    private val firstStage: LoggedMechanismLigament2d?
    private val connector: LoggedMechanismLigament2d?
    private val secondStage: LoggedMechanismLigament2d?

    private var firstStagePose: Pose3d?
    private var secondStagePose: Pose3d?

    private val firstStage2dStartLength = 1.060526
    private val secondStage2dStartLength = 0.374865

    private val stages3dStartX = 0.0
    private val fisrtStage3dStartY = 0.0
    private val secondStage3dStartY = 0.0
    private val stages3dStartZ = 0.0

    init {
        if (mechanism2dEnabled) {
            mechanism = LoggedMechanism2d(0.0, 0.0)
            mechanismRoot = mechanism.getRoot("Elevator", 0.488950, 0.0) // Measure from the origin
            firstStage = LoggedMechanismLigament2d("First Stage", firstStage2dStartLength, 90.0, 6.0, Color8Bit(0, 0, 150))
            connector = LoggedMechanismLigament2d("Connector", 0.02, 0.0, 0.0, Color8Bit())
            secondStage = LoggedMechanismLigament2d("Second Stage", secondStage2dStartLength, 90.0, 5.0, Color8Bit(150, 0, 0))

            Logger.recordOutput("Mechanism2d/Elevator", mechanism)
        } else {
            mechanism = null
            mechanismRoot = null
            firstStage = null
            connector = null
            secondStage = null
        }

        if (mechanism3dEnabled){
            firstStagePose = Pose3d()
            secondStagePose = Pose3d()

            Logger.recordOutput("Mechanism3d/Elevator", firstStagePose, secondStagePose)
        } else {
            firstStagePose = null
            secondStagePose = null
        }
    }

    fun update(){
        if (mechanism2dEnabled){
            firstStage!!.length = firstStage2dStartLength + Elevator.getPosition()
            secondStage!!.length = secondStage2dStartLength + 2 * Elevator.getPosition()

            Logger.recordOutput("Mechanism2d/Elevator", mechanism)
        }

        if (mechanism3dEnabled){
            firstStagePose = Pose3d(stages3dStartX, fisrtStage3dStartY + Elevator.getPosition(), stages3dStartZ, Rotation3d.kZero)
            secondStagePose = Pose3d(stages3dStartX, secondStage3dStartY + 2 * Elevator.getPosition(), stages3dStartZ, Rotation3d.kZero)
        }
    }

}