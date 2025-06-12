package org.sert2521.reefscape2025.subsystems.wrist

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.util.Color8Bit
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d
import kotlin.math.PI

class WristTelemetry(private val mechanism2dEnabled:Boolean, private val mechanism3dEnabled:Boolean) {
    private val mechanism:LoggedMechanism2d?
    private val root:LoggedMechanismRoot2d?

    private val wristMain:LoggedMechanismLigament2d?
    private val wristTop:LoggedMechanismLigament2d?
    private val wristBottom:LoggedMechanismLigament2d?

    private var wristPose:Pose3d?

    init {
        if (mechanism2dEnabled){
            mechanism = LoggedMechanism2d(0.0, 0.0)
            root = mechanism.getRoot("Wrist Root", -0.285750, 0.247650)

            wristMain = LoggedMechanismLigament2d(
                "Wrist Main", 0.0, 100.0, 0.0, Color8Bit()
            )
            wristTop = LoggedMechanismLigament2d(
                "Wrist Top", 0.42, 100.0 - 111.612575184 - 11.4592, 4.0,
                Color8Bit("#00D000")
            )
            wristBottom = LoggedMechanismLigament2d(
                "Wrist Bottom", 0.348, 100.0 - 86.8856310843 - 11.4592, 4.0,
                Color8Bit("#00D000")
            )

            wristMain.append(wristTop)
            wristMain.append(wristBottom)
            root.append(wristMain)

            Logger.recordOutput("Mechanism2d/Wrist", mechanism)
        } else {
            mechanism = null
            root = null
            wristMain = null
            wristTop = null
            wristBottom = null
        }

        if (mechanism3dEnabled){
            wristPose = Pose3d()
            Logger.recordOutput("Mechanism3d/Wrist", wristPose)
        } else {
            wristPose = null
        }
    }

    fun update(){
        if (mechanism2dEnabled){
            wristMain!!.angle = -Wrist.getRotations() * 360 + 180

            Logger.recordOutput("Mechanism2d/Wrist", mechanism)
        }

        if (mechanism3dEnabled){
            wristPose = Pose3d(-0.285750, 0.0, 0.247650, Rotation3d(0.0, Wrist.getRotations() * 2 * PI + 0.42 + 0.2, 0.0))

            Logger.recordOutput("Mechanism3d/Wrist", wristPose)
        }
    }
}