package org.firstinspires.ftc.teamcode.odometry12Ball;

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
    public PathChain RedRow1ToGate1;
    public PathChain RedRow1ToGate2;
    public PathChain RedGateToShooter;
    public PathChain RedRow1ToGate;
    public PathChain RedRow1ToShooter;
    public PathChain RedRow2IntakeBegin;
    public PathChain RedRow2IntakeEnd;
    public PathChain RedRow2ToShooter;
    public PathChain RedRow3IntakeBegin;
    public PathChain RedRow3IntakeEnd;
    public PathChain RedRow3ToShooter;
    public PathChain RedLeave;
    public PathChain RedLeave12Ball;
    public PathChain RedFarStart;
    public PathChain RedFarRow1IntakeBegin;
    public PathChain RedFarRow1IntakeEnd;
    public PathChain RedFarRow1ToShooter;
    public PathChain RedFarRow2IntakeBegin;
    public PathChain RedFarRow2IntakeEnd;
    public PathChain RedFarRow2ToShooter;
    public PathChain RedFarLeave;
    public PathChain BlueStart;
    public PathChain BlueRow1IntakeBegin;
    public PathChain BlueRow1IntakeEnd;
    public PathChain BlueRow1ToGate1;
    public PathChain BlueRow1ToGate2;
    public PathChain BlueGateToShooter;
    public PathChain BlueRow1ToGate;
    public PathChain BlueRow1ToShooter;
    public PathChain BlueRow2IntakeBegin;
    public PathChain BlueRow2IntakeEnd;
    public PathChain BlueRow2ToShooter;
    public PathChain BlueRow3IntakeBegin;
    public PathChain BlueRow3IntakeEnd;
    public PathChain BlueRow3ToShooter;
    public PathChain BlueLeave;
    public PathChain BlueFarStart;
    public PathChain BlueFarRow1IntakeBegin;
    public PathChain BlueFarRow1IntakeEnd;
    public PathChain BlueFarRow1ToShooter;
    public PathChain BlueFarRow2IntakeBegin;
    public PathChain BlueFarRow2IntakeEnd;
    public PathChain BlueFarRow2ToShooter;
    public PathChain BlueFarLeave;
    public PathChain BlueLeave12Ball;

    //initalize all of our paths
    public Paths(Follower follower) {
        RedStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(110.36335877862595, 134.10687022900763), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        RedRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 83.762), new Pose(125.200, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToGate1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 83.762), new Pose(119.200, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToGate2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.200, 72.930), new Pose(125.200, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 83.762)/*, new Pose(123.100, 70.300)*/, new Pose(129.353, 70.930))
                        //Find out how to add a control point, Coming at the gate diagonally like this could cause a problem with rubber band intakes.
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();


        RedGateToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 72.930), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 83.762), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
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
                        new BezierLine(new Pose(125.200, 59.579), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedRow3IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 35.005))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        RedRow3IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 35.005), new Pose(125.200, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow3ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 35.383), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        RedLeave12Ball = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(125.200, 70.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        RedFarStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 9.7), new Pose(84.000, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67.5))
                .build();

        RedFarRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 12.000), new Pose(97.108, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(0))
                .build();

        RedFarRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 35.383), new Pose(125.200, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedFarRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 35.383), new Pose(84.000, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67.5))
                .build();

        RedFarRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 12.000), new Pose(97.108, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(0))
                .build();

        RedFarRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 59.579), new Pose(125.200, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedFarRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.200, 59.579), new Pose(84.000, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67.5))
                .build();

        RedFarLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 12.000), new Pose(97.108, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(0))
                .build();

        BlueStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.6366412214, 134.10687022900763), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        BlueRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        BlueRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 83.762), new Pose(18.800, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow1ToGate1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.800, 83.762), new Pose(18.800, 70.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();


        BlueRow1ToGate2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.800, 70.930), new Pose(14.647, 70.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        BlueRow1ToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.8, 83.762)/*, new Pose(20.9, 70.300)*/, new Pose(14.647, 70.930))
                        //Find out how to add a control point, Coming at the gate diagonally like this could cause a problem with rubber band intakes.
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        BlueGateToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.353, 70.930), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        BlueRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.8, 83.762), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        BlueRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        BlueRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 59.798), new Pose(18.800, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.800, 59.798), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        BlueRow3IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        BlueRow3IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 35.383), new Pose(18.800, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow3ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.800, 35.383), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        BlueLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        BlueLeave12Ball = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(18.8, 70.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                .build();

        BlueFarStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 9.7), new Pose(60.000, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112.5))
                .build();

        BlueFarRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 12.000), new Pose(46.892, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(180))
                .build();

        BlueFarRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 35.383), new Pose(18.800, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueFarRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.800, 35.383), new Pose(60.000, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112.5))
                .build();

        BlueFarRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 12.000), new Pose(46.892, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(180))
                .build();

        BlueFarRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 59.798), new Pose(18.800, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueFarRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.800, 59.798), new Pose(60.000, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112.5))
                .build();

        BlueFarLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 12.000), new Pose(46.892, 59.798))
                )
                .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(180))
                .build();
    }
}