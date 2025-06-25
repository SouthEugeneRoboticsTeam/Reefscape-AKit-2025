package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.VisionTargetPositions
import org.sert2521.reefscape2025.commands.drivetrain.JoystickDrive
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.PI
import kotlin.math.abs


object Drivetrain : SubsystemBase() {
    @JvmField
    val odometryLock: ReentrantLock = ReentrantLock()

    val swerveDriveSimulation = //Doesn't create it in REAL or REPLAY to save objects
        if (MetaConstants.currentMode == MetaConstants.Mode.SIM) {
            SwerveDriveSimulation(
                SwerveConstants.mapleSimConfig,
                Pose2d(3.0, 3.0, Rotation2d())
            )
        } else {
            null
        }

    private val gyroInputs = LoggedGyroIOInputs()
    private val gyroIO = when (MetaConstants.currentMode) {
        MetaConstants.Mode.REAL -> GyroIONavX()
        MetaConstants.Mode.SIM -> GyroIOSim(swerveDriveSimulation!!.gyroSimulation)
        MetaConstants.Mode.REPLAY -> object : GyroIO {}
    }

    private val visionInputsLeft = LoggedVisionIOInputs()
    private val visionInputsRight = LoggedVisionIOInputs()

    private val visionIOLeft = when (MetaConstants.currentMode) {
        MetaConstants.Mode.REAL -> VisionIOLimelight("limelight-left")
        else -> object : VisionIO {}
    }
    private val visionIORight = when (MetaConstants.currentMode) {
        MetaConstants.Mode.REAL -> VisionIOLimelight("limelight-right")
        else -> object : VisionIO {}
    }

    private val modules =
        when (MetaConstants.currentMode) {
            MetaConstants.Mode.REAL -> Array(4) { Module(ModuleIOSpark(it), it) }
            MetaConstants.Mode.SIM -> Array(4) { Module(ModuleIOSim(swerveDriveSimulation!!.modules[it]), it) }
            MetaConstants.Mode.REPLAY -> Array(4) { Module(object : ModuleIO {}, it) }
        }


    private var fed = true


    private val sysId =
        SysIdRoutine(
            SysIdRoutine.Config(null, null, null) {
                Logger.recordOutput("Drive/SysIdState", it.toString())
            },
            SysIdRoutine.Mechanism(
                { runCharacterization(it.`in`(Volts)) }, null, this
            )
        )

    private val gyroDisconnectedAlert =
        Alert("Disconnected Gyro, resorting to kinematics", Alert.AlertType.kError)

    private val kinematics = SwerveDriveKinematics(*SwerveConstants.moduleTranslations)

    private val lastModulePositions = Array(4) { SwerveModulePosition() }

    private var rawGyroRotation = Rotation2d(PI / 2)

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        rawGyroRotation,
        lastModulePositions,
        Pose2d()
    )

    val field = Field2d()

    init {
        //while it's TECHNICALLY not just copy pasted, I'll still report this as swerve template whatevers
        //because I would NOT know how to program this on my own
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_RobotDrive,
            FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit
        )

        this.defaultCommand = JoystickDrive()

        SmartDashboard.putData("Robot Pose", field)

        //I'm putting the auto builder somewhere else because this is ridiculous

        if (MetaConstants.currentMode == MetaConstants.Mode.SIM) {
            SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation)
        }
    }

    override fun periodic() {
        odometryLock.lock()

        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)

        for (module in modules) {
            module.periodic()
        }

        odometryLock.unlock()

        if (DriverStation.isDisabled()) {
            for (module in modules) {
                module.stop()
            }
        }


        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveModuleStates/Setpoints", *Array(4) { SwerveModuleState() })
            Logger.recordOutput("SwerveModuleStates/Optimized Setpoints", *Array(4) { SwerveModuleState() })
        }

        val modulePositions = Array(4) { modules[it].getPosition() }


        if (gyroInputs.connected) {
            rawGyroRotation = gyroInputs.yawPosition
        } else {
            val moduleDeltas = Array(4) {
                SwerveModulePosition(
                    modulePositions[it].distanceMeters - lastModulePositions[it].distanceMeters,
                    modulePositions[it].angle - lastModulePositions[it].angle
                )
            }
            val twist = kinematics.toTwist2d(*moduleDeltas)
            rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
        }

        poseEstimator.update(rawGyroRotation, modulePositions)

        for (moduleIndex in 0..<4) {
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
        }


        visionIOLeft.updateInputs(visionInputsLeft)
        visionIORight.updateInputs(visionInputsRight)

        Logger.processInputs("Drive/Limelight Left", visionInputsLeft)
        Logger.processInputs("Drive/Limelight Right", visionInputsRight)

        gyroDisconnectedAlert.set(!gyroInputs.connected && MetaConstants.currentMode != MetaConstants.Mode.SIM)

        if (!visionInputsLeft.rejectEstimation) {
            if (visionInputsLeft.megatagTwo) {
                addVisionMeasurement(
                    visionInputsLeft.estimatedPosition,
                    visionInputsLeft.timestamp,
                    SwerveConstants.LIMELIGHT_STDV
                )
            } else {
                addVisionMeasurement(
                    visionInputsLeft.estimatedPosition,
                    visionInputsLeft.timestamp,
                    SwerveConstants.LIMELIGHT_STDV_YAW_RESET
                )
            }
        }

        if (!visionInputsRight.rejectEstimation) {
            if (visionInputsRight.megatagTwo) {
                addVisionMeasurement(
                    visionInputsRight.estimatedPosition,
                    visionInputsRight.timestamp,
                    SwerveConstants.LIMELIGHT_STDV
                )
            } else {
                addVisionMeasurement(
                    visionInputsRight.estimatedPosition,
                    visionInputsRight.timestamp,
                    SwerveConstants.LIMELIGHT_STDV_YAW_RESET
                )
            }
        }

        field.robotPose = getPose()

        if (!fed) {
            driveRobotOriented(ChassisSpeeds())
        }
        fed = false

        Logger.recordOutput("SwerveModuleStates/Measured", *getModuleStates())

        Logger.recordOutput("Odometry/Robot Pose", getPose())
        Logger.recordOutput("Odometry/Robot Rotations", getPose().rotation.rotations)

        if (MetaConstants.currentMode == MetaConstants.Mode.SIM) {
            Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation!!.simulatedDriveTrainPose)
        }
    }

    /* === Setters === */

    fun driveRobotOriented(speeds: ChassisSpeeds, withPID: Boolean = true) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)

        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_SPEED_MPS)

        Logger.recordOutput("SwerveModuleStates/Setpoints", *setpointStates)
        val optimizedStates = Array(4) { SwerveModuleState() }

        for (i in 0..<4) {
            optimizedStates[i] = modules[i].runSetpoint(setpointStates[i], withPID)
        }

        Logger.recordOutput("SwerveModuleStates/Optimized Setpoints", *optimizedStates)

        fed = true
    }

    fun setGyroYaw(gyroRotation2d: Rotation2d) {
        setPose(Pose2d(poseEstimator.estimatedPosition.x, poseEstimator.estimatedPosition.y, gyroRotation2d))
    }

    fun setPoseAuto(pose: Pose2d) {
        setGyroYaw(pose.rotation)
    }

    fun runCharacterization(output: Double) {
        for (i in 0..<4) {
            modules[i].runCharacterization(output)
        }
    }

    fun stop() {
        driveRobotOriented(ChassisSpeeds())
    }

    fun setCurrentLimit(limit: Int) {
        for (module in modules) {
            module.setCurrentLimit(limit)
        }
    }

    fun stopWithX() {
        val headings = Array(4) {
            SwerveConstants.moduleTranslations[it].angle
        }
        kinematics.resetHeadings(*headings)
        stop()
    }

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return run {
            runCharacterization(0.0)
        }.withTimeout(1.0)
            .andThen(sysId.quasistatic(direction))
    }

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return run {
            runCharacterization(0.0)
        }.withTimeout(1.0)
            .andThen(sysId.dynamic(direction))
    }

    fun addVisionMeasurement(
        visionEstimationMeters: Pose2d, timestampSeconds: Double,
        visionMeasurementsStDev: Matrix<N3, N1>
    ) {
        poseEstimator.addVisionMeasurement(
            visionEstimationMeters,
            timestampSeconds,
            visionMeasurementsStDev
        )
    }

    fun printChassisSpeeds(speeds: ChassisSpeeds) {
        println(speeds.vxMetersPerSecond)
        println(speeds.vyMetersPerSecond)
    }

    /* == Getters == */

    private fun getModuleStates(): Array<SwerveModuleState> {
        return Array(4) { modules[it].getState() }
    }

    private fun getModulePositions(): Array<SwerveModulePosition> {
        return Array(4) { modules[it].getPosition() }
    }

    fun getChassisSpeeds(): ChassisSpeeds {
        //THE IDE IS LYING I SWEAR THIS WORKS
        //I'VE ALREADY SPENT HOURS OF MY LIFE TRYING TO STOP THIS FROM HAPPENING
        //THIS IS THE ONLY WAY TO GET IT TO WORK DISREGARD EVERYTHING THE IDE TELLS YOU
        return kinematics.toChassisSpeeds(*getModuleStates())
    }

    fun wheelRadiusCharacterizationPositions(): DoubleArray {
        return DoubleArray(4) { modules[it].getWheelRadiusCharacterizationPosition() }
    }

    fun getFFCharacterizationVelocity(): Double {
        var output = 0.0
        for (i in 0..<4) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0
        }
        return output
    }

    fun getGyroRateDegrees(): Double {
        return abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec))
    }

    fun getPose(): Pose2d {
        return if (MetaConstants.currentMode == MetaConstants.Mode.SIM){
            swerveDriveSimulation!!.simulatedDriveTrainPose
        } else {
            poseEstimator.estimatedPosition
        }
    }

    fun getRotation(): Rotation2d {
        return getPose().rotation
    }

    fun setPose(pose: Pose2d) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose)
    }

    fun getMaxSpeedMPS(): Double {
        return SwerveConstants.MAX_SPEED_MPS
    }

    fun getGyroConnected(): Boolean {
        return gyroInputs.connected
    }

    fun getNearestTargetReef(left: Boolean): Pose2d {
        if (left) {
            return getPose().nearest(VisionTargetPositions.reefPositionsLeft)
        } else {
            return getPose().nearest(VisionTargetPositions.reefPositionsRight)
        }
    }

    fun driveBackCommand(): Command {
        val speeds = ChassisSpeeds(-0.7, 0.0, 0.0)
        return run {
            driveRobotOriented(speeds)
        }
    }

    fun stopCommand(): Command {
        return runOnce {
            driveRobotOriented(ChassisSpeeds())
        }
    }
}