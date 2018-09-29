package main

import main.*

// https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf is an amazing paper

fun main(args: Array<String>) {
    val constraints = MotionProfilingConstraints(10.0, 1.0)
    val statistics = DriveTrainStatistics(1.0, 1.0)

    val path = LinearPath(Waypoint(-1.0, 0.0), Waypoint(-2.0, 2.0))
    // val follower = ConstantCurvaturePathFollower(path, constraints, statistics)

    println(path.generate(0.25))
}