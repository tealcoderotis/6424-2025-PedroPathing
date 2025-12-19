package org.firstinspires.ftc.teamcode.odometry12BallFixed;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LimelightPoseCorrector;

@Configurable
@Autonomous(name = "Odometry 12 Ball Auton (Fixed)")
public class Auton extends LinearOpMode {
    private Follower follower;
    private ShooterIntakeContinuous shooterIntake;
    private int pathState;
    private Paths paths;
    private Alliance alliance;
    private LimelightPoseCorrector poseCorrector;
    private Timer gateTimer;
    private static final int GATE_TIME = 500;
    private static final double BALL_INTAKE_MOVEMENT_SPEED=0.5;
    private boolean useLimelight = false;

    @Override
    public void runOpMode() {
        //initialization
        gateTimer = new Timer();
        shooterIntake = new ShooterIntakeContinuous(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        paths = new Paths(follower);
        pathState = 0;
        alliance = Alliance.UNKNOWN;
        DcMotor rightFrontDrive = (DcMotor) hardwareMap.get("rightFrontDrive");
        DcMotor rightBackDrive = (DcMotor) hardwareMap.get("rightBackDrive");
        DcMotor leftFrontDrive = (DcMotor) hardwareMap.get("leftFrontDrive");
        DcMotor leftBackDrive = (DcMotor) hardwareMap.get("leftBackDrive");
        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) {
                //Red starting pose
                follower.setStartingPose(new Pose(110.36335877862595, 134.10687022900763, Math.toRadians(0)));
                alliance = Alliance.RED;
                telemetry.addLine("Red alliance");
                telemetry.update();
            } else if (gamepad1.xWasPressed()) {
                //Blue starting pose
                follower.setStartingPose(new Pose(33.85648854961832, 134.10687022900763, Math.toRadians(180)));
                alliance = Alliance.BLUE;
                telemetry.addLine("Blue alliance");
                telemetry.update();
            }
            if (gamepad1.yWasPressed()) {
                useLimelight = !useLimelight;
                if (useLimelight) {
                    telemetry.addLine("Use limelight");
                }
                else {
                    telemetry.addLine("Don't use limelight");
                }
                telemetry.update();
            }
        }
        if (alliance != Alliance.UNKNOWN) {
            Globals.resetGlobals();
            if (useLimelight) {
                poseCorrector = new LimelightPoseCorrector(hardwareMap);
            }
            while (opModeIsActive()) {
                //update shooter and follower
                shooterIntake.update();
                if (useLimelight) {
                    follower.setPose(poseCorrector.correctPose(follower.getPose()));
                }
                follower.update();
                if (alliance == Alliance.RED) {
                    autonomousRedPathUpdate();
                } else if (alliance == Alliance.BLUE) {
                    autonomousBluePathUpdate();
                }
                telemetry.addData("path state", pathState);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.addData("leftFrontDrive", leftFrontDrive.getPower());
                telemetry.addData("leftBackDrive", leftBackDrive.getPower());
                telemetry.addData("rightFrontDrive", rightFrontDrive.getPower());
                telemetry.addData("rightBackDrive", rightBackDrive.getPower());
                telemetry.update();
            }
            Globals.alliance = alliance;
            Globals.endingPose = follower.getPose().copy();
        } else {
            //if we start and do not specify an alliance, the OpMode will stop
            requestOpModeStop();
        }
    }

    //checks our current action and if the follower or shooter/intake are currently active, if not we increment at state counter, moving on to the next action
    //only called if we are the red alliance
    public void autonomousRedPathUpdate() {
        switch (pathState) {
            case 0:
                shooterIntake.beginReving();
                follower.followPath(paths.RedStart);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 2;
                }
                break;
            case 2:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedRow1IntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.RedRow1IntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow1ToShooter);
                    pathState = 8;
                }
                break;
            /*case 5:
                if (!follower.isBusy()) {

                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow1ToGate2);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    gateTimer.resetTimer();
                    pathState = 7;
                }
                break;
            case 7:
                if (gateTimer.getElapsedTime() >= GATE_TIME) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedGateToShooter);
                    pathState = 8;
                }
                break;*/
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 9;
                }
                break;
            case 9:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedRow2IntakeBegin);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.RedRow2IntakeEnd);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow2ToShooter);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 13;
                }
                break;
            case 13:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedRow3IntakeBegin);
                    pathState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.RedRow3IntakeEnd);
                    pathState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow3ToShooter);
                    pathState = 16;
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 17;
                }
                break;
            case 17:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedLeave);
                    pathState = 18;
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    pathState = 19;
                }
                break;
        }
    }

    //checks our current action and if the follower or shooter/intake are currently active, if not we increment at state counter, moving on to the next action
    //only called if we are the blue alliance
    public void autonomousBluePathUpdate() {
        switch (pathState) {
            case 0:
                shooterIntake.beginReving();
                follower.followPath(paths.BlueStart);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 2;
                }
                break;
            case 2:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueRow1IntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.BlueRow1IntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow1ToShooter);
                    pathState = 8;
                }
                break;
            /*case 5:
                if (!follower.isBusy()) {

                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow1ToGate2);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    gateTimer.resetTimer();
                    pathState = 7;
                }
                break;
            case 7:
                if (gateTimer.getElapsedTime() >= GATE_TIME) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueGateToShooter);
                    pathState = 8;
                }
                break;*/
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 9;
                }
                break;
            case 9:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueRow2IntakeBegin);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.BlueRow2IntakeEnd);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow2ToShooter);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 13;
                }
                break;
            case 13:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueRow3IntakeBegin);
                    pathState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.BlueRow3IntakeEnd);
                    pathState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow3ToShooter);
                    pathState = 16;
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 17;
                }
                break;
            case 17:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueLeave);
                    pathState = 18;
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    pathState = 19;
                }
                break;
        }
    }
}