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
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.VisionTargetPositions
import org.sert2521.reefscape2025.commands.drivetrain.JoystickDrive
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.PI
import kotlin.math.abs


object Drivetrain : SubsystemBase() {
    @JvmField
    val odometryLock:ReentrantLock = ReentrantLock()


    private val gyroInputs = LoggedGyroIOInputs()
    private val gyroIO = GyroIONavX()

    private val visionInputs = LoggedVisionIOInputs()
    private val visionIOLeft = VisionIOLimelight("limelight-left")

    private val modules = arrayOf(Module(0), Module(1), Module(2), Module(3))

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

    private val lastModulePositions = Array(4){SwerveModulePosition()}

    private var rawGyroRotation = Rotation2d(PI/2)

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        rawGyroRotation,
        lastModulePositions,
        Pose2d()
    )

    val field = Field2d()

    init{
        //while it's TECHNICALLY not just copy pasted, I'll still report this as swerve template whatevers
        //because I would NOT know how to program this on my own
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit)

        SparkOdometryThread.getInstance().start()

        this.defaultCommand = JoystickDrive()

        SmartDashboard.putData("Robot Pose", field)

        //I'm putting the auto builder somewhere else because this is ridiculous
    }

    override fun periodic() {
        odometryLock.lock()

        gyroIO.updateInputs(gyroInputs)
        visionIOLeft.updateInputs(visionInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)
        Logger.processInputs("Drive/Vision", visionInputs)

        for (module in modules){
            module.periodic()
        }

        odometryLock.unlock()

        if (DriverStation.isDisabled()){
            for (module in modules){
                module.stop()
            }
        }


        if (DriverStation.isDisabled()){
            Logger.recordOutput("SwerveModuleStates/Setpoints", *Array(4){SwerveModuleState()})
            Logger.recordOutput("SwerveModuleStates/Optimized Setpoints", *Array(4){SwerveModuleState()})
        }

        val sampleTimestamps = modules[0].getOdometryTimestamps()
        val sampleCount = sampleTimestamps.size

        for (i in 0..<sampleCount){
            val modulePositions = Array(4){SwerveModulePosition()}
            val moduleDeltas = Array(4){SwerveModulePosition()}
            for (moduleIndex in 0..<4){
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i]
                moduleDeltas[moduleIndex] =
                    SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters- lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle
                    )
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
            }

            if(gyroInputs.connected){
                rawGyroRotation = gyroInputs.odometryYawPositions[i]
            } else {
                val twist = kinematics.toTwist2d(*moduleDeltas)
                rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
            }


            poseEstimator.updateWithTime(
                sampleTimestamps[i],
                rawGyroRotation,
                modulePositions
            )
        }

        gyroDisconnectedAlert.set(!gyroInputs.connected && MetaConstants.currentMode != MetaConstants.Mode.SIM)

        if (!visionInputs.rejectEstimation){
            if (visionInputs.megatagTwo){
                addVisionMeasurement(visionInputs.estimatedPosition, visionInputs.timestamp, SwerveConstants.LIMELIGHT_STDV)
            } else {
                addVisionMeasurement(visionInputs.estimatedPosition, visionInputs.timestamp, SwerveConstants.LIMELIGHT_STDV_YAW_RESET)
            }
        }

        field.robotPose = getPose()

        Logger.recordOutput("SwerveChassisSpeeds/Measured", getChassisSpeeds())
        Logger.recordOutput("SwerveModuleStates/Measured", *getModuleStates())
        Logger.recordOutput("Odometry/Robot Pose", getPose())
        Logger.recordOutput("Odometry/Robot Rotations", getPose().rotation.rotations)
    }

    /* === Setters === */

    fun driveRobotOriented(speeds: ChassisSpeeds){
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)

        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_SPEED_MPS)

        Logger.recordOutput("SwerveModuleStates/Setpoints",*setpointStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints",discreteSpeeds)

        val optimizedStates = Array(4) { SwerveModuleState() }

        for (i in 0..<4){
            optimizedStates[i] = modules[i].runSetpoint(setpointStates[i])
        }

        Logger.recordOutput("SwerveModuleStates/Optimized Setpoints", *optimizedStates)


    }

    fun setGyroYaw(gyroRotation2d: Rotation2d){
        setPose(Pose2d(poseEstimator.estimatedPosition.x, poseEstimator.estimatedPosition.y, gyroRotation2d))
    }

    fun setPoseAuto(pose:Pose2d){
        setGyroYaw(pose.rotation)
    }

    fun runCharacterization(output:Double){
        for (i in 0..<4){
            modules[i].runCharacterization(output)
        }
    }

    fun stop(){
        driveRobotOriented(ChassisSpeeds())
    }

    fun setCurrentLimit(limit:Int){
        for (module in modules){
            module.setCurrentLimit(limit)
        }
    }

    fun stopWithX(){
        val headings = Array(4){
            SwerveConstants.moduleTranslations[it].angle
        }
        kinematics.resetHeadings(*headings)
        stop()
    }

    fun sysIdQuasistatic(direction:SysIdRoutine.Direction): Command {
        return run{
            runCharacterization(0.0)
        }.withTimeout(1.0)
            .andThen(sysId.quasistatic(direction))
    }

    fun sysIdDynamic(direction:SysIdRoutine.Direction): Command {
        return run{
            runCharacterization(0.0)
        }.withTimeout(1.0)
            .andThen(sysId.dynamic(direction))
    }

    fun addVisionMeasurement(visionEstimationMeters:Pose2d, timestampSeconds:Double,
                             visionMeasurementsStDev: Matrix<N3, N1>){
        poseEstimator.addVisionMeasurement(
            visionEstimationMeters,
            timestampSeconds,
            visionMeasurementsStDev
        )
    }

    fun printChassisSpeeds(speeds:ChassisSpeeds){
        println(speeds.vxMetersPerSecond)
        println(speeds.vyMetersPerSecond)
    }

    /* == Getters == */

    private fun getModuleStates():Array<SwerveModuleState>{
        return Array(4){modules[it].getState()}
    }

    private fun getModulePositions():Array<SwerveModulePosition>{
        return Array(4){modules[it].getPosition()}
    }

    fun getChassisSpeeds():ChassisSpeeds{
        //THE IDE IS LYING I SWEAR THIS WORKS
        //I'VE ALREADY SPENT HOURS OF MY LIFE TRYING TO STOP THIS FROM HAPPENING
        //THIS IS THE ONLY WAY TO GET IT TO WORK DISREGARD EVERYTHING THE IDE TELLS YOU
        return kinematics.toChassisSpeeds(*getModuleStates())
    }

    fun wheelRadiusCharacterizationPositions():DoubleArray{
        return DoubleArray(4){modules[it].getWheelRadiusCharacterizationPosition()}
    }

    fun getFFCharacterizationVelocity():Double{
        var output = 0.0
        for (i in 0..<4){
            output += modules[i].getFFCharacterizationVelocity() / 4.0
        }
        return output
    }

    fun getGyroRateDegrees():Double{
        return abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec))
    }

    fun getPose():Pose2d{
        return poseEstimator.estimatedPosition
    }

    fun getRotation():Rotation2d{
        return getPose().rotation
    }

    fun setPose(pose:Pose2d) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose)
    }

    fun getMaxSpeedMPS():Double{
        return SwerveConstants.MAX_SPEED_MPS
    }

    fun getGyroConnected():Boolean{
        return gyroInputs.connected
    }

    fun getNearestTarget(): Pose2d {
        return getPose().nearest(VisionTargetPositions.reefPositions)
    }
}