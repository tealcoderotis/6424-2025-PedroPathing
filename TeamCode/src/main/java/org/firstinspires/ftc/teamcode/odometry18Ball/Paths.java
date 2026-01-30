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
    public PathChain RedShooterHeading;
    public PathChain RedRow1IntakeBegin;
    public PathChain RedRow1IntakeEnd;
    public PathChain RedRow1ToGate1;
    public PathChain RedRow1ToGate2;
    public PathChain RedGateToShooter;
    public PathChain RedRow1ToGate;
    public PathChain RedRow1ToShooter;
    public PathChain RedRow2IntakeBegin;
    public PathChain RedRow2IntakeEnd;
    public PathChain RedRow2Gate1;
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
    public PathChain RedFarIntakeBeginFromShooter;
    public PathChain RedIntakeBegin;
    public PathChain RedIntakeEnd;
    public PathChain RedIntakeBack;
    public PathChain RedIntakeEnd2;
    public PathChain RedIntakeToShooter;
    public PathChain BlueStart;
    public PathChain BlueShooterHeading;
    public PathChain BlueRow1IntakeBegin;
    public PathChain BlueRow1IntakeEnd;
    public PathChain BlueRow1ToGate1;
    public PathChain BlueRow1ToGate2;
    public PathChain BlueGateToShooter;
    public PathChain BlueRow1ToGate;
    public PathChain BlueRow1ToShooter;
    public PathChain BlueRow2IntakeBegin;
    public PathChain BlueRow2IntakeEnd;
    public PathChain BlueRow2ToGate;
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
    public PathChain BlueFarIntakeBeginFromShooter;
    public PathChain BlueIntakeBegin;
    public PathChain BlueIntakeEnd;
    public PathChain BlueIntakeBack;
    public PathChain BlueIntakeEnd2;
    public PathChain BlueIntakeToShooter;

    //initalize all of our paths
    public Paths(Follower follower) {
        RedStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.6, 121.6), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(42))
                .build();

        RedShooterHeading = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96, 95.8), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(42))
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
                        new BezierLine(new Pose(97.108, 83.762), new Pose(126, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToGate1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126, 83.762), new Pose(119.200, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToGate2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.200, 72.930), new Pose(124, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow1ToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(128, 83.762)/*, new Pose(123.100, 70.300)*/, new Pose(129.353, 70.930))
                        //Find out how to add a control point, Coming at the gate diagonally like this could cause a problem with rubber band intakes.
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedGateToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124, 72.930), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126, 83.762), new Pose(96, 95.8))
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
                        new BezierLine(new Pose(97.108, 59.579), new Pose(126, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow2Gate1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(126.000, 59.579), new Pose(110.000, 59.579), new Pose(119.200, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        RedRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(126, 59.579), new Pose(96, 59.579), new Pose(96, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))
                .build();

        RedShooterToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(96, 95.8), new Pose(96, 66), new Pose(110.000, 66))
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(90))
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
                        new BezierLine(new Pose(96, 95.8), new Pose(91.108, 35.005))
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
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                .build();

        RedFarIntakeBeginFromShooter = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96, 95.8),
                                new Pose(108.000, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
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

        BlueStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.4, 121.6), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))
                .build();

        BlueShooterHeading = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))
                .build();

        BlueRow1IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                .build();

        BlueRow1IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 83.762), new Pose(18, 83.762))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow1ToGate1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18, 83.762), new Pose(24.8, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow1ToGate2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(24.8, 72.930), new Pose(20, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueGateToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20, 72.930), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                .build();

        BlueRow1ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18, 83.762), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                .build();

        BlueRow2IntakeBegin = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(48, 95.8), new Pose(46.892, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                .build();

        BlueRow2IntakeEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.892, 59.579), new Pose(18, 59.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow2ToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(18.000, 59.579), new Pose(34.000, 59.579), new Pose(24.8, 72.930))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueRow2ToShooter = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(18, 59.579), new Pose(48, 59.579), new Pose(48, 95.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                .build();

        BlueShooterToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(48, 95.8), new Pose(48, 66), new Pose(34, 66))
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(90))
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
                        new BezierLine(new Pose(48, 95.8), new Pose(52.892, 35.005))
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
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180))
                .build();

        BlueFarIntakeBeginFromShooter = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48, 95.8),
                                new Pose(36, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(0))
                .build();

        BlueIntakeBegin = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60, 18.000),
                                new Pose(36, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))
                .build();

        BlueIntakeEnd = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36, 9.000),
                                new Pose(9, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueIntakeBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9, 9.000),
                                new Pose(36, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueIntakeEnd2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36, 9.000),
                                new Pose(9, 9.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        BlueIntakeToShooter = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9, 9.000),
                                new Pose(60, 18.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(62))
                .build();
    }
}