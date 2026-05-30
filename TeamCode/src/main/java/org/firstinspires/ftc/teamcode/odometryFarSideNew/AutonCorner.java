package org.firstinspires.ftc.teamcode.odometryFarSideNew;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.ShooterIntakeContinuous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Hood;
import org.firstinspires.ftc.teamcode.util.LimelightPoseCorrector;

@Configurable
@Autonomous(name = "Odometry Far Side Corner")
public class AutonCorner extends LinearOpMode {
    private Follower follower;
    private ShooterIntakeContinuous shooterIntake;
    private int pathState;
    private Paths paths;
    private Alliance alliance;
    private LimelightPoseCorrector poseCorrector;
    private Timer maxIntakeTimer;
    private boolean useLimelight = false;
    private Hood hood;

    @Override
    public void runOpMode() {
        //initialization
        maxIntakeTimer = new Timer();
        shooterIntake = new ShooterIntakeContinuous(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        paths = new Paths(follower);
        pathState = 0;
        alliance = Alliance.UNKNOWN;
        hood = new Hood(hardwareMap.get(Servo.class, "hoodServo"));
        DcMotor rightFrontDrive = (DcMotor) hardwareMap.get("rightFrontDrive");
        DcMotor rightBackDrive = (DcMotor) hardwareMap.get("rightBackDrive");
        DcMotor leftFrontDrive = (DcMotor) hardwareMap.get("leftFrontDrive");
        DcMotor leftBackDrive = (DcMotor) hardwareMap.get("leftBackDrive");
        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) {
                //Red starting pose
                follower.setStartingPose(new Pose(84.000, 12.000, Math.toRadians(0)));
                alliance = Alliance.RED;
                telemetry.addLine("Red alliance");
                telemetry.update();
            } else if (gamepad1.xWasPressed()) {
                //Blue starting pose
                follower.setStartingPose(new Pose(60.000, 12.000, Math.toRadians(180)));
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
            hood.extend();
            shooterIntake.start(Globals.FEEDER_FAR_IDLE_POWER);
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

    public void autonomousRedPathUpdate() {
        switch (pathState) {
            case 0:
                shooterIntake.beginReving(Globals.SHOOTER_FAR_VELOCITY);
                follower.followPath(paths.RedStart);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3, Globals.SHOOTER_FAR_VELOCITY, Globals.INDEX_FAR_TIME);
                    pathState = 2;
                }
                break;
            case 2:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedIntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.RedIntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedIntakeBack);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.RedIntakeEnd);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedIntakeToShooter);
                    pathState = 1;
                }
                break;
        }
    }

    public void autonomousBluePathUpdate() {
        switch (pathState) {
            case 0:
                shooterIntake.beginReving(Globals.SHOOTER_FAR_VELOCITY);
                follower.followPath(paths.BlueStart);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3, Globals.SHOOTER_FAR_VELOCITY, Globals.INDEX_FAR_TIME);
                    pathState = 2;
                }
                break;
            case 2:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueIntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.BlueIntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueIntakeBack);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    maxIntakeTimer.resetTimer();
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.BlueIntakeEnd);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy() || maxIntakeTimer.getElapsedTime() >= Globals.MAX_INTAKE_TIME) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueIntakeToShooter);
                    pathState = 1;
                }
                break;
        }
    }
}