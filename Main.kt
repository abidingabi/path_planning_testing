package main

import main.*

// https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf is an amazing paper

fun main(args: Array<String>) {
    val constraints = MotionProfilingConstraints(2.3, 1.0)
    val statistics = DriveTrainStatistics(1.0, 1.0)

    val path = LinearPath(Waypoint(0.0, 0.0), Waypoint(0.0, 100.0))
    val follower = ConstantCurvaturePathFollower(path, constraints, statistics)

    var t = 0.0

    while (t <= follower.timeToFollow) {
        println(follower.generate(t).state.toMotorVelocity(statistics))
        t += 0.01
    }    
}