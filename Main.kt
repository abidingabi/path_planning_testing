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

data class Waypoint(val x: Double, val y: Double)

interface Path {
    // t should range from 0 to 1, 1 being the end of the path, 0 being the beginning
    fun generate(t: Double): Waypoint

    val length: Double
}

interface ConstantCurvaturePath : Path{
    val curvature: Double
}

class ArcPath(val startPoint: Waypoint, val centerPoint: Waypoint, val endAngle: Double): ConstantCurvaturePath {
    private val radius = Math.sqrt(
        Math.pow(startPoint.x-centerPoint.x, 2.0) +
        Math.pow(startPoint.y-centerPoint.y, 2.0)
    )
    private val startAngle = Math.atan2(startPoint.y-centerPoint.y, startPoint.x-startPoint.x)
    private val correctedEndAngle = endAngle % Math.PI*2

    override val length = Math.abs((correctedEndAngle-startAngle)*radius)
    override val curvature = 1/radius
    
    override fun generate(t: Double): Waypoint {
        val tScaled = Math.abs((t * (correctedEndAngle - startAngle)) + startAngle)
        return Waypoint(
            centerPoint.x + radius * Math.cos(tScaled),
            centerPoint.y + radius * Math.sin(tScaled)
        )
    }
}


data class MotionProfilingConstraints(val maximumVelocity: Double, val maximumAcceleration: Double)

interface PathFollower {
    val timeToFollow: Double
    // t should range from 0 to the amount of time for the path in seconds
    fun generate(t: Double): DriveTrainState
}

class ConstantCurvaturePathFollower
    (val path: ConstantCurvaturePath, val constraints: MotionProfilingConstraints, val statistics: DriveTrainStatistics) : PathFollower {

    private val timeToAccelerate = (constraints.maximumVelocity/constraints.maximumAcceleration)

    override val timeToFollow = timeToAccelerate + (path.length/constraints.maximumVelocity)

    private val timeToCruise = timeToFollow - timeToAccelerate

    override fun generate(t: Double): DriveTrainState {
        val rightVelocity = when {
            t < timeToAccelerate -> constraints.maximumAcceleration * t
            t <= timeToCruise -> constraints.maximumVelocity
            t <= timeToFollow -> ((constraints.maximumVelocity*constraints.maximumVelocity)/
                (2.0*constraints.maximumAcceleration)) - 
                (constraints.maximumAcceleration*t)
            else -> 0.0
        }

        val rightToLeftRatio = 1-path.curvature

        println(rightToLeftRatio)

        val rightVelocityScaled = if (rightToLeftRatio > -1) rightVelocity else Math.abs(rightVelocity/rightToLeftRatio) 

        return MotorVelocity(rightVelocityScaled, rightVelocityScaled * rightToLeftRatio).toDriveTrainState(statistics)
    }
}

fun main(args: Array<String>) {
    val constraints = MotionProfilingConstraints(10.0, 1.0)
    val statistics = DriveTrainStatistics(1.0, 1.0)

    val path = ArcPath(Waypoint(0.0, 100.0), Waypoint(0.0, 0.0), Math.PI)
    val follower = ConstantCurvaturePathFollower(path, constraints, statistics)

    println(path.curvature)
    println(follower.generate(follower.timeToFollow/2).toMotorVelocity(statistics))
}