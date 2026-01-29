package org.firstinspires.ftc.teamcode.odometryFarSide;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain RedStart;
    public PathChain RedIntakeBegin;
    public PathChain RedIntakeEnd;
    public PathChain RedIntakeBack;
    public PathChain RedIntakeEnd2;
    public PathChain RedIntakeToShooter;
    public PathChain BlueStart;
    public PathChain BlueIntakeBegin;
    public PathChain BlueIntakeEnd;
    public PathChain BlueIntakeBack;
    public PathChain BlueIntakeEnd2;
    public PathChain BlueIntakeToShooter;

    public Paths(Follower follower) {
        RedStart = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 9.700),
                                new Pose(84.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(62))
                .build();

        RedIntakeBegin = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 18.000),
                                new Pose(108.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(62), Math.toRadians(0))
                .build();

        RedIntakeEnd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(108.000, 9.000),
                                new Pose(135.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedIntakeBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.000, 9.000),
                                new Pose(108.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedIntakeEnd2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(108.000, 9.000),
                                new Pose(135.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedIntakeToShooter = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.000, 9.000),
                                new Pose(84.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(62))
                .build();

        BlueStart = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 9.700),
                                new Pose(60.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(118))
                .build();

        BlueIntakeBegin = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 18.000),
                                new Pose(36.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))
                .build();

        BlueIntakeEnd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36.000, 9.000),
                                new Pose(9.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueIntakeBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 9.000),
                                new Pose(36.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        BlueIntakeEnd2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36.000, 9.000),
                                new Pose(9.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        BlueIntakeToShooter = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.000, 9.000),
                                new Pose(60.000, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(118))
                .build();
    }
}
