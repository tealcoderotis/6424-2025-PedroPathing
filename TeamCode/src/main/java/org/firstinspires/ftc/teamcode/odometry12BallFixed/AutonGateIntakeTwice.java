package org.firstinspires.ftc.teamcode.odometry12BallFixed;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.ShooterIntakeContinuous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LimelightPoseCorrector;

@Configurable
@Autonomous(name = "Odometry 18 Ball Auton")
public class AutonGateIntakeTwice extends LinearOpMode {
    private Follower follower;
    private ShooterIntakeContinuous shooterIntake;
    private int pathState;
    private Paths paths;
    private Alliance alliance;
    private LimelightPoseCorrector poseCorrector;
    private Timer gateTimer;
    private Timer maxIntakeTimer;
    private static final int GATE_TIME = 5;
    private boolean useLimelight = false;

    @Override
    public void runOpMode() {
        //initialization
        gateTimer = new Timer();
        maxIntakeTimer = new Timer();
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
            shooterIntake.start();
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
                    follower.followPath(paths.RedRow2IntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.RedRow2IntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow2ToShooter);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 6;
                }
                break;
            case 6:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedShooterToGate);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.RedShooterToGate2);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    follower.pausePathFollowing();
                    gateTimer.resetTimer();
                    pathState = 9;
                }
                break;
            case 9:
                if (gateTimer.getElapsedTime() >= GATE_TIME) {
                    follower.resumePathFollowing();
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.RedGateIntake);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    maxIntakeTimer.resetTimer();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedGateIntakeEnd);
                    shooterIntake.beginIntaking(true);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_GATE_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedGateIntakeToShooter);
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
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.RedRow3IntakeEnd);
                    pathState = 15;
                }
                break;
            case 15:
                if ((!follower.isBusy()) || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
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
                    follower.followPath(paths.RedShooterToGate);
                    pathState = 18;
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.RedShooterToGate2);
                    pathState = 19;
                }
                break;
            case 19:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    follower.pausePathFollowing();
                    gateTimer.resetTimer();
                    pathState = 20;
                }
                break;
            case 20:
                if (gateTimer.getElapsedTime() >= GATE_TIME) {
                    follower.resumePathFollowing();
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.RedGateIntake);
                    pathState = 21;
                }
                break;
            case 21:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    maxIntakeTimer.resetTimer();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedGateIntakeEnd);
                    shooterIntake.beginIntaking(true);
                    pathState = 22;
                }
                break;
            case 22:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_GATE_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedGateIntakeToShooter);
                    pathState = 23;
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 24;
                }
                break;
            case 24:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedRow1IntakeBegin);
                    pathState = 25;
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.RedRow1IntakeEnd);
                    pathState = 26;
                }
                break;
            case 26:
                if ((!follower.isBusy()) || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow1ToShooter);
                    pathState = 27;
                }
                break;
            case 27:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 28;
                }
                break;
            case 28:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedLeave);
                    pathState = 29;
                }
                break;
            case 29:
                if (!follower.isBusy()) {
                    pathState = 30;
                }
                break;
        }
    }

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
                    follower.followPath(paths.BlueRow2IntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.BlueRow2IntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow2ToShooter);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 6;
                }
                break;
            case 6:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueShooterToGate);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.BlueShooterToGate2);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    follower.pausePathFollowing();
                    gateTimer.resetTimer();
                    pathState = 9;
                }
                break;
            case 9:
                if (gateTimer.getElapsedTime() >= GATE_TIME) {
                    follower.resumePathFollowing();
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.BlueGateIntake);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    maxIntakeTimer.resetTimer();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueGateIntakeEnd);
                    shooterIntake.beginIntaking(true);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_GATE_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueGateIntakeToShooter);
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
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.BlueRow3IntakeEnd);
                    pathState = 15;
                }
                break;
            case 15:
                if ((!follower.isBusy()) || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
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
                    follower.followPath(paths.BlueShooterToGate);
                    pathState = 18;
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.BlueShooterToGate2);
                    pathState = 19;
                }
                break;
            case 19:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    follower.pausePathFollowing();
                    gateTimer.resetTimer();
                    pathState = 20;
                }
                break;
            case 20:
                if (gateTimer.getElapsedTime() >= GATE_TIME) {
                    follower.resumePathFollowing();
                    maxIntakeTimer.resetTimer();
                    follower.followPath(paths.BlueGateIntake);
                    pathState = 21;
                }
                break;
            case 21:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    maxIntakeTimer.resetTimer();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueGateIntakeEnd);
                    shooterIntake.beginIntaking(true);
                    pathState = 22;
                }
                break;
            case 22:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_GATE_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueGateIntakeToShooter);
                    pathState = 23;
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 24;
                }
                break;
            case 24:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueRow1IntakeBegin);
                    pathState = 25;
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.BlueRow1IntakeEnd);
                    pathState = 26;
                }
                break;
            case 26:
                if ((!follower.isBusy()) || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow1ToShooter);
                    pathState = 27;
                }
                break;
            case 27:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 28;
                }
                break;
            case 28:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueLeave);
                    pathState = 29;
                }
                break;
            case 29:
                if (!follower.isBusy()) {
                    pathState = 30;
                }
                break;
        }
    }
}