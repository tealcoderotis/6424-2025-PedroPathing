package org.firstinspires.ftc.teamcode.odometry9Ball;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.ShooterIntakeContinuous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LimelightPoseCorrector;

@Configurable
@Autonomous(name = "Odometry 9 Ball Auton (Far Side) (Continuous Shooting)")
public class AutonFarContinuous extends LinearOpMode {
    private Follower follower;
    private ShooterIntakeContinuous shooterIntake;
    private int pathState;
    private Paths paths;
    private Alliance alliance;
    private LimelightPoseCorrector poseCorrector;
    private boolean useLimelight = false;

    @Override
    public void runOpMode() {
        //initialization
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
                follower.setStartingPose(new Pose(84.000, 9.7, Math.toRadians(90)));
                alliance = Alliance.RED;
                telemetry.addLine("Red alliance");
                telemetry.update();
            } else if (gamepad1.xWasPressed()) {
                //Blue starting pose
                follower.setStartingPose(new Pose(60.000, 9.7, Math.toRadians(90)));
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
                shooterIntake.beginReving(2000);
                follower.followPath(paths.RedFarStart);
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
                    follower.followPath(paths.RedFarRow1IntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.RedFarRow1IntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    shooterIntake.stopIntaking();
                    shooterIntake.beginReving(2000);
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedFarRow1ToShooter);
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
                    follower.followPath(paths.RedFarRow2IntakeBegin);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.RedFarRow2IntakeEnd);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.stopIntaking();
                    shooterIntake.beginReving(2000);
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedFarRow2ToShooter);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 10;
                }
                break;
            case 10:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedFarLeave);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    pathState = 12;
                }
                break;
        }
    }

    //checks our current action and if the follower or shooter/intake are currently active, if not we increment at state counter, moving on to the next action
    //only called if we are the blue alliance
    public void autonomousBluePathUpdate() {
        switch (pathState) {
            case 0:
                shooterIntake.beginReving(2000);
                follower.followPath(paths.BlueFarStart);
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
                    follower.followPath(paths.BlueFarRow1IntakeBegin);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.BlueFarRow1IntakeEnd);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    shooterIntake.stopIntaking();
                    shooterIntake.beginReving(2000);
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueFarRow1ToShooter);
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
                    follower.followPath(paths.BlueFarRow2IntakeBegin);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(Globals.INTAKE_SPEED);
                    follower.followPath(paths.BlueFarRow2IntakeEnd);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.stopIntaking();
                    shooterIntake.beginReving(2000);
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueFarRow2ToShooter);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 10;
                }
                break;
            case 10:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueFarLeave);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    pathState = 12;
                }
                break;
        }
    }
}