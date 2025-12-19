package org.firstinspires.ftc.teamcode.odometry6Ball;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

//PedroPathing Paths
public class Paths {
    //declare all of our paths
    public PathChain RedStart;
    public PathChain RedRow1IntakeBegin;
    public PathChain RedRow1IntakeEnd;
    public PathChain RedRow1ToShooter;
    public PathChain RedRow2IntakeBegin;
    public PathChain RedRow2IntakeEnd;
    public PathChain RedRow2ToShooter;
    public PathChain RedLeave;
    public PathChain BlueStart;
    public PathChain BlueRow1IntakeBegin;
    public PathChain BlueRow1IntakeEnd;
    public PathChain BlueRow1ToShooter;
    public PathChain BlueRow2IntakeBegin;
    public PathChain BlueRow2IntakeEnd;
    public PathChain BlueRow2ToShooter;
    public PathChain BlueLeave;

    //initalize all of our paths
    public Paths(Follower follower) {
        RedStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(110.36335877862595, 134.10687022900763), new Pose(83.392, 132.938))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(83.392, 132.938), new Pose(97.108, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 83.762), new Pose(125.200, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 83.762), new Pose(83.392, 132.938))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(83.392, 132.938), new Pose(97.108, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 59.579), new Pose(125.200, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 59.579), new Pose(83.392, 132.938))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(83.392, 132.938), new Pose(97.108, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        BlueStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.85648854961832, 134.10687022900763), new Pose(61.051, 132.133))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(61.051, 132.133), new Pose(46.892, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 83.762), new Pose(19.100, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.100, 83.762), new Pose(61.051, 132.133))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(61.051, 132.133), new Pose(46.892, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 59.798), new Pose(19.100, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.100, 59.798), new Pose(61.051, 132.133))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(61.051, 132.133), new Pose(46.892, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
}