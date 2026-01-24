package org.firstinspires.ftc.teamcode.odometryFarSide;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain RedStart;
    public PathChain RedObservationIntakeBegin;
    public PathChain RedObservationIntakeEnd;
    public PathChain RedObservationToShooter;
    public PathChain BlueStart;
    public PathChain BlueObservationIntakeBegin;
    public PathChain BlueObservationIntakeEnd;
    public PathChain BlueObservationToShooter;

    public Paths(Follower follower) {
        RedStart = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 9.700),
                                new Pose(84.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67.5))
                .build();

        RedObservationIntakeBegin = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 12.000),
                                new Pose(84.000, 26.000),
                                new Pose(130.000, 24.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(330))
                .build();

        RedObservationIntakeEnd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130.000, 24.000),
                                new Pose(132.000, 12.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(330))
                .build();

        RedObservationToShooter = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 12.000),
                                new Pose(114.000, 24.000),
                                new Pose(84.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(67.5))
                .build();

        BlueStart = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 9.700),
                                new Pose(60.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112.5))
                .build();

        BlueObservationIntakeBegin = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 12.000),
                                new Pose(60.000, 26.000),
                                new Pose(14.000, 24.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(210))
                .build();

        BlueObservationIntakeEnd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.000, 24.000),
                                new Pose(12.000, 12.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(210))
                .build();

        BlueObservationToShooter = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.000, 12.000),
                                new Pose(30.000, 24.000),
                                new Pose(60.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(112.5))
                .build();

    }
}
