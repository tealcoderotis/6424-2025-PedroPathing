package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PathsEfficient;
import org.firstinspires.ftc.teamcode.util.LimelightPoseCorrector;

@Configurable
@Autonomous(name = "Odometry 9 Ball Auton")
public class OdometryAuton9Ball extends LinearOpMode {
    private Follower follower;
    private ShooterIntakeEfficient shooterIntake;
    private int pathState;
    private PathsEfficient paths;
    //-1 unknown; 0 red; 1 blue
    private int alliance;
    private LimelightPoseCorrector poseCorrector;

    @Override
    public void runOpMode() {
        //initialization
        shooterIntake = new ShooterIntakeEfficient(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        paths = new PathsEfficient(follower);
        pathState = 0;
        alliance = -1;
        DcMotor rightFrontDrive = (DcMotor) hardwareMap.get("rightFrontDrive");
        DcMotor rightBackDrive = (DcMotor) hardwareMap.get("rightBackDrive");
        DcMotor leftFrontDrive = (DcMotor) hardwareMap.get("leftFrontDrive");
        DcMotor leftBackDrive = (DcMotor) hardwareMap.get("leftBackDrive");
        poseCorrector = new LimelightPoseCorrector(hardwareMap);
        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) {
                //Red starting pose
                follower.setStartingPose(new Pose(110.36335877862595, 134.10687022900763, Math.toRadians(0)));
                alliance = 0;
                telemetry.addLine("Red alliance");
                telemetry.update();
            } else if (gamepad1.xWasPressed()) {
                //Blue starting pose
                follower.setStartingPose(new Pose(33.85648854961832, 134.10687022900763, Math.toRadians(180)));
                alliance = 1;
                telemetry.addLine("Blue alliance");
                telemetry.update();
            }
        }
        if (alliance != -1) {
            while (opModeIsActive()) {
                //update shooter and follower
                shooterIntake.update();
                follower.setPose(poseCorrector.correctPose(follower.getPose()));
                follower.update();
                if (alliance == 0) {
                    autonomousRedPathUpdate();
                } else if (alliance == 1) {
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
                    follower.followPath(paths.RedRow2IntakeBegin);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.RedRow2IntakeEnd);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow2ToShooter);
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
                    follower.followPath(paths.RedLeave);
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
                    follower.followPath(paths.BlueRow2IntakeBegin);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.BlueRow2IntakeEnd);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.beginReving();
                    shooterIntake.stopIntaking();
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow2ToShooter);
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
                    follower.followPath(paths.BlueLeave);
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