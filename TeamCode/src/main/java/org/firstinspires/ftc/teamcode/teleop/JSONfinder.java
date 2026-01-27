package org.firstinspires.ftc.teamcode.teleop;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@Configurable
@TeleOp(name = "JSONfinder")
public class JSONfinder extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private Limelight3A limelight;
    private Follower follower;
    private IMU imu;
    private Alliance alliance = Alliance.UNKNOWN;
    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        follower = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(1); //1 check per second
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // only april tag 22
        limelight.start();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));//Initialize while facing right? maybe forward? Not 45 degrees.

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }
    @Override
    public void start() {
        limelight.start();
        follower.startTeleOpDrive(false);
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            if (results != null) {
                LLResultTypes.FiducialResult firstresult = results.get(0); //There should never be more than 1 result because it is filtered in the pipeline
                if (firstresult != null) {
                    List<List<Double>> targetcorners = firstresult.getTargetCorners();
                    telemetry.addData("x0", targetcorners.get(0).get(0));
                    telemetry.addData("y0", targetcorners.get(0).get(1));
                    telemetry.addData("x1", targetcorners.get(1).get(0));
                    telemetry.addData("y1", targetcorners.get(1).get(1));
                    telemetry.addData("x2", targetcorners.get(2).get(0));
                    telemetry.addData("y2", targetcorners.get(2).get(1));
                    telemetry.addData("x3", targetcorners.get(3).get(0));
                    telemetry.addData("y3", targetcorners.get(3).get(1));
                }
                else {
                    telemetry.addLine("First Fiducial Result is null");
                }
            }
            else {
                telemetry.addLine("getFiducialResults returns null, have you 'enabled in output tab'?");
            }
        }
        else {
            telemetry.addLine("No valid apriltags");
        }
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }
}