package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.ftc.InvertedFTCCoordinates;
//import com.pedropathing.ftc.PedroCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Objects;

@TeleOp(name = "MegaTag2")
public class MegaTag2 extends OpMode {
    private String alliance = "UNKNOWN";
    private IMU imu;
    private Limelight3A limelight;
    LLResult previousResult = null; //Used to only update things when new result occurs.

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private boolean fieldCentric = true;

    Pose3D otherBotPose;
    Pose3D botPoseRaw;
    Pose2D botPoseFlat;
    Pose camPose;
    @Override
    public void init() {
        //IMU and Limelight Setup
        imu = (IMU) hardwareMap.get("imu");
        imu.resetYaw();
        limelight = (Limelight3A) hardwareMap.get("limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(10);//This definitely should be tuned
        telemetry.setMsTransmissionInterval(10);

        //Drive Motor Setup
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

        telemetry.addData("Status", "Initialized");
    }
    public void init_loop() {
        if (gamepad1.bWasPressed()) {
            alliance = "RED";
        }
        if (gamepad1.xWasPressed()) {
            alliance = "BLUE";
        }
        if (gamepad1.dpadUpWasPressed()) {
            fieldCentric = false;
        }
        if (gamepad1.dpadDownWasPressed()) {
            fieldCentric = true;
        }
    }
    public void start() {
        limelight.start();
    }
    public void loop() {
        double rotate = gamepad1.right_stick_x; // Edit for goal tracking?
        FieldDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw();
        telemetry.addData("yaw", yaw);
        limelight.updateRobotOrientation(yaw);
        LLResult result = limelight.getLatestResult();
        if (result.isValid() && !Objects.equals(previousResult, result)) {
            previousResult = result;
            telemetry.addData("tracking", true);
            otherBotPose = result.getBotpose();
            botPoseRaw = result.getBotpose_MT2();

            botPoseFlat = new Pose2D(DistanceUnit.METER, botPoseRaw.getPosition().x, botPoseRaw.getPosition().y, AngleUnit.DEGREES, botPoseRaw.getOrientation().getYaw());
            camPose = PoseConverter.pose2DToPose(botPoseFlat, InvertedFTCCoordinates.INSTANCE);

            //telemetry.addData("PedroPose", camPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
        } else if (!result.isValid()){
            telemetry.addData("tracking", false);
        }
        telemetry.addData("MT1", otherBotPose);
        telemetry.addData("raw pose", botPoseRaw);
        telemetry.addData("cam pose", camPose);


        telemetry.addData("alliance", alliance);
        telemetry.update();
    }
    void FieldDrive(double forward, double strafe, double rotate) {
        if (fieldCentric) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            strafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            forward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        }
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
