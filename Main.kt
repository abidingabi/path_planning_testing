// https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf is an amazing paper

data class DriveTrainStatistics(val wheelRadius: Double, val wheelDistance: Double)

data class DriveTrainState(val linearVelocity: Double, val angularVelocity: Double) {
    fun toMotorVelocity(statistics: DriveTrainStatistics): MotorVelocity {        
        val rightVelocity = (2*linearVelocity + angularVelocity*statistics.wheelDistance) / (statistics.wheelRadius * 2)
        val leftVelocity = ((2*linearVelocity)/statistics.wheelRadius) - rightVelocity
        return MotorVelocity(rightVelocity, leftVelocity)
    }
}

data class MotorVelocity(val rightVelocity: Double, val leftVelocity: Double) {
    fun toDriveTrainState(statistics: DriveTrainStatistics) = DriveTrainState(
        (statistics.wheelRadius * (rightVelocity+leftVelocity)) / 2,
        (statistics.wheelRadius * (rightVelocity-leftVelocity)) / statistics.wheelDistance
    )
}

data class Waypoint(x: Double, y: Double)

interface ConstantCurvaturePath {
    val curvature: Double
    
    // time should range from 0 to 1, 1 being the end of the path, 0 being the beginning
    fun generate(time: Double): Waypoint
}

data class MotionProfilingConstraints(val maximumVelocity, val maximumAcceleration)

class ArcPath(): ConstantCurvaturePath {
    override val curvature = 

    override fun generate(time: Double): Waypoint {

    }
}

class ConstantCurvaturePathFollower(path: ConstantCurvaturePath) {

}

fun main(args: Array<String>) {
    val driveTrainStatistics = DriveTrainStatistics(1.0, 1.0)
    println(DriveTrainState(2.0, 0.1).toMotorVelocity(driveTrainStatistics))
}