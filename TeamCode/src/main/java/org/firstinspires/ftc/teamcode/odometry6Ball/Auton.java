package org.firstinspires.ftc.teamcode.odometry6Ball;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Odometry 6 Ball Auton")
public class Auton extends LinearOpMode {
    private Follower follower;
    private ShooterIntake shooterIntake;
    private int pathState;
    private Paths paths;
    //-1 unknown; 0 red; 1 blue
    private int alliance;

    @Override
    public void runOpMode() {
        //initialization
        shooterIntake = new ShooterIntake(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        paths = new Paths(follower);
        pathState = 0;
        alliance = -1;
        DcMotor rightFrontDrive = (DcMotor)hardwareMap.get("rightFrontDrive");
        DcMotor rightBackDrive = (DcMotor)hardwareMap.get("rightBackDrive");
        DcMotor leftFrontDrive = (DcMotor)hardwareMap.get("leftFrontDrive");
        DcMotor leftBackDrive = (DcMotor)hardwareMap.get("leftBackDrive");
        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) {
                //Red starting pose
                follower.setStartingPose(new Pose(110.36335877862595, 134.10687022900763, Math.toRadians(0)));
                alliance = 0;
                telemetry.addLine("Red alliance");
                telemetry.update();
            }
            else if (gamepad1.xWasPressed()) {
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
                follower.update();
                if (alliance == 0) {
                    autonomousRedPathUpdate();
                }
                else if (alliance == 1) {
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
        }
        else {
            //if we start and do not specify an alliance, the OpMode will stop
            requestOpModeStop();
        }
    }

    //checks our current action and if the follower or shooter/intake are currently active, if not we increment at state counter, moving on to the next action
    //only called if we are the red alliance
    public void autonomousRedPathUpdate() {
        switch (pathState) {
            case 0:
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
                    shooterIntake.stopIntaking();
                    pathState = 5;
                }
                break;
            case 5:
                if (!shooterIntake.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow1ToShooter);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 7;
                }
                break;
            case 7:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedRow2IntakeBegin);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.RedRow2IntakeEnd);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    shooterIntake.stopIntaking();
                    pathState = 10;
                }
                break;
            case 10:
                if (!shooterIntake.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.RedRow2ToShooter);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 12;
                }
                break;
            case 12:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.RedLeave);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    pathState = 14;
                }
                break;
        }
    }

    //checks our current action and if the follower or shooter/intake are currently active, if not we increment at state counter, moving on to the next action
    //only called if we are the blue alliance
    public void autonomousBluePathUpdate() {
        switch (pathState) {
            case 0:
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
                    shooterIntake.stopIntaking();
                    pathState = 5;
                }
                break;
            case 5:
                if (!shooterIntake.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow1ToShooter);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 7;
                }
                break;
            case 7:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueRow2IntakeBegin);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooterIntake.beginIntaking(true);
                    follower.setMaxPower(0.25);
                    follower.followPath(paths.BlueRow2IntakeEnd);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    shooterIntake.stopIntaking();
                    pathState = 10;
                }
                break;
            case 10:
                if (!shooterIntake.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.BlueRow2ToShooter);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    shooterIntake.beginShooting(3);
                    pathState = 12;
                }
                break;
            case 12:
                if (!shooterIntake.isBusy()) {
                    follower.followPath(paths.BlueLeave);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    pathState = 14;
                }
                break;
        }
    }
}
