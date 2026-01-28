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

    private double distfrompoint(double tpx, int corner) {
        //Height of apriltag above ground = 29.5
        //Side length of apriltag = 6.5
        double cornerheight;
        if ((corner == 0) || (corner == 1)){ // Maybe 0,1
            cornerheight = 29.5+3.25;
        } else {
            cornerheight = 29.5-3.25;
        }
        //960 y axis pixels -> 42 degrees
        return (cornerheight-13)/Math.tan((tpx*42/960)*Math.PI/180);//Measured 20.9, changed experimentally to 22 TODO: Don't hardcode
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
                    telemetry.addData("x0, y0:", targetcorners.get(0).toString());
                    telemetry.addData("x1, y1:", targetcorners.get(1).toString());
                    telemetry.addData("x2, y2:", targetcorners.get(2).toString());
                    telemetry.addData("x3, y3:", targetcorners.get(3).toString());
                    double thickness = Math.abs(((targetcorners.get(0).get(0) + targetcorners.get(3).get(0))-(targetcorners.get(1).get(0) + targetcorners.get(2).get(0)))/2);
                    double angle = thickness * 54.5/1280; //Angle of triangle at limelight, Will not use
                    double R = (distfrompoint(targetcorners.get(1).get(1), 1)+distfrompoint(targetcorners.get(2).get(1), 2))/2;
                    double L = (distfrompoint(targetcorners.get(0).get(1), 0)+distfrompoint(targetcorners.get(3).get(1), 3))/2;
                    telemetry.addData("R", R);
                    telemetry.addData("L", L);
                    double x = (Math.pow(L, 2)-Math.pow(R, 2))/(4*3.25);
                    double y = Math.sqrt(Math.pow(R, 2)-Math.pow(x-3.25, 2));
                    telemetry.addData("Distance from tag's center", Math.sqrt(Math.pow(x,2)+Math.pow(y, 2)));
                    telemetry.addData("Perpendicular", y);
                    telemetry.addData("Parallel", x);
                }
                else
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