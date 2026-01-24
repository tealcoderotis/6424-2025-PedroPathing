package org.firstinspires.ftc.teamcode.odometryTest;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain Up;
    public PathChain Right;
    public PathChain Down;
    public PathChain Left;

    public Paths(Follower follower) {
        Up = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(0.000, 0.000),

                                new Pose(0.000, 24.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Right = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(0.000, 24.000),

                                new Pose(24.000, 24.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Down = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.000, 24.000),

                                new Pose(24.000, 0.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Left = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.000, 0.000),

                                new Pose(0.000, 0.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}
