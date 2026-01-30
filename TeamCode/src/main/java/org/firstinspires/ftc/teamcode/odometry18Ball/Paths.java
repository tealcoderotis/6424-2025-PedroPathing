package org.firstinspires.ftc.teamcode.odometry18Ball;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

//PedroPathing Paths
public class Paths {
    //declare all of our paths
    public PathChain RedStart;
    public PathChain RedRow1IntakeBegin;
    public PathChain RedRow1IntakeEnd;
    public PathChain RedShooterHeading;
    public PathChain RedGateToShooter;
    public PathChain RedRow1ToShooter;
    public PathChain RedRow2IntakeBegin;
    public PathChain RedRow2IntakeEnd;
    public PathChain RedRow2ToShooter;
    public PathChain RedShooterToGate;
    public PathChain RedShooterToGate2;
    public PathChain RedGateIntake;
    public PathChain RedGateIntakeEnd;
    public PathChain RedGateIntakeToShooter;
    public PathChain RedRow3IntakeBegin;
    public PathChain RedRow3IntakeEnd;
    public PathChain RedRow3ToShooter;
    public PathChain RedLeave;
    public PathChain BlueStart;
    public PathChain BlueShooterHeading;
    public PathChain BlueRow1IntakeBegin;
    public PathChain BlueRow1IntakeEnd;
    public PathChain BlueGateToShooter;
    public PathChain BlueRow1ToShooter;
    public PathChain BlueRow2IntakeBegin;
    public PathChain BlueRow2IntakeEnd;
    public PathChain BlueRow2ToShooter;
    public PathChain BlueShooterToGate;
    public PathChain BlueShooterToGate2;
    public PathChain BlueGateIntake;
    public PathChain BlueGateIntakeEnd;
    public PathChain BlueGateIntakeToShooter;
    public PathChain BlueRow3IntakeBegin;
    public PathChain BlueRow3IntakeEnd;
    public PathChain BlueRow3ToShooter;
    public PathChain BlueLeave;

    //initalize all of our paths
    public Paths(Follower follower) {
        RedStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(110.3633588, 134.10687022900763), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 83.762))
                )
                .setTangentHeadingInterpolation()
                .build();

        RedRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 83.762), new Pose(126, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126, 83.762), new Pose(96, 95.8))
                )
                .setTangentHeadingInterpolation()
                .build();

        RedShooterHeading = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(96, 95.8))
                )
                .setConstantHeadingInterpolation(42)
                .build();

        RedRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 59.579))
                )
                .setTangentHeadingInterpolation()
                .build();

        RedRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 59.579), new Pose(126, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(126, 59.579), new Pose(96, 59.579), new Pose(96, 95.8))
                )
                .setTangentHeadingInterpolation()
                .build();

        RedShooterToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(96, 95.8), new Pose(96, 66), new Pose(110.000, 66))
                )
                .setTangentHeadingInterpolation()
                .build();

        RedShooterToGate2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(110.000, 66), new Pose(126, 66))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        RedGateIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(126, 66), new Pose(126, 48), new Pose(132, 48))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        RedGateIntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132, 48), new Pose(132, 60))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        RedGateIntakeToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(132, 60), new Pose(96, 48), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(42))
                .build();

        RedRow3IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 35.005))
                )
                .setTangentHeadingInterpolation()
                .build();

        RedRow3IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.108, 35.005), new Pose(126, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow3ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(126, 35.383), new Pose(96, 35.383), new Pose(96, 95.8))
                )
                .setTangentHeadingInterpolation()
                .build();

        RedLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(97.108, 59.579))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.63664122, 134.10687022900763), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                .build();

        BlueShooterHeading = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(48, 95.8))
                )
                .setConstantHeadingInterpolation(138)
                .build();

        BlueRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 83.762))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 83.762), new Pose(18, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18, 83.762), new Pose(48, 95.8))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 59.579))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 59.579), new Pose(18, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(18, 59.579), new Pose(48, 59.579), new Pose(48, 95.8))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueShooterToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(48, 95.8), new Pose(48, 66), new Pose(34, 66))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueShooterToGate2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(34, 66), new Pose(18, 66))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        BlueGateIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(18, 66), new Pose(18, 48), new Pose(12, 48))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        BlueGateIntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(12, 48), new Pose(12, 60))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        BlueGateIntakeToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(12, 60), new Pose(48, 48), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(138))
                .build();

        BlueRow3IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 35.005))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueRow3IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 35.005), new Pose(18, 35.383))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow3ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(18, 35.383), new Pose(48, 35.383), new Pose(48, 95.8))
                )
                .setTangentHeadingInterpolation()
                .build();

        BlueLeave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 59.579))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}