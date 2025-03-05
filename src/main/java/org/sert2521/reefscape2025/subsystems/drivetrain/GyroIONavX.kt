package org.sert2521.reefscape2025.subsystems.drivetrain

import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units

class GyroIONavX:GyroIO {
    /* If you're here and have a problem with the gyro, just call software lead */
    private val imu = AHRS(AHRS.NavXComType.kMXP_SPI, SwerveConstants.ODOMETRY_FREQUENCY)
    private val yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue()
    private val yawPositionQueue = SparkOdometryThread.getInstance().registerSignal{imu.angle}

    override fun updateInputs(inputs:GyroIO.GyroIOInputs){
        inputs.connected = imu.isConnected
        inputs.yawPosition = Rotation2d.fromDegrees(-imu.angle)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-imu.rate)

        inputs.odometryYawTimestamps=
            yawTimestampQueue.stream().mapToDouble{it}.toArray()
        inputs.odometryYawPositions=
            yawPositionQueue.stream().map{Rotation2d.fromDegrees(-it)}
                .toArray().filterIsInstance<Rotation2d>().toTypedArray()
        yawTimestampQueue.clear()
        yawPositionQueue.clear()
    }
}