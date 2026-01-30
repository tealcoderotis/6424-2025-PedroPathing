package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Regression;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@TeleOp(name = "OlyCowZTeleOp (Android Studio)")
public class OlyCowZTeleOpTwo extends OpMode {
    //Variable Init
    final double LAUNCHER_MAX_VELOCITY = 1950;
    final double LAUNCHER_MIN_VELOCITY = 1500;
    final double FEEDER_INTAKE_VELOCITY = 3000;
    final double FEEDER_LAUNCH_VELOCITY = 1200;
    final double FEEDER_STOP_VELOCITY = 0;

    private Servo stopper;
    private DcMotorEx launcher = null;
    private DcMotorEx feeder = null;
    private Limelight3A limelight;
    int state;

    double flywheelVelocity = 0;
    double xGoal = 144;
    Alliance alliance = Alliance.UNKNOWN;
    private Follower follower;
    Pose recentPoseEstimate = new Pose(97.108, 59.579, Math.toRadians(0));
    boolean visualTrack = false;
    final double PGain = 1;
    final double DGain = 0.2;
    Pose pCoordinates = new Pose(0, 0, 0);

    public void init() {
        state = 1;//Intake
        follower = Constants.createFollower(hardwareMap);

        stopper = hardwareMap.get(Servo.class, "gateServo");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        stopper.setPosition(1);

        launcher.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(15);
        telemetry.setMsTransmissionInterval(200);
        limelight.pipelineSwitch(0);

        feeder.setPower(0);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        limelight.start();
        follower.startTeleOpDrive(false);
    }

    public void init_loop() {
        if (gamepad1.bWasPressed()) {
            //Red starting pose
            recentPoseEstimate = new Pose(97.108, 59.579, Math.toRadians(0));
            follower.setStartingPose(recentPoseEstimate);
            alliance = Alliance.RED;
            xGoal = 144;
        } else if (gamepad1.xWasPressed()) {
            //Blue starting pose
            recentPoseEstimate = new Pose(46.892, 59.798, Math.toRadians(180));
            follower.setStartingPose(recentPoseEstimate);
            alliance = Alliance.BLUE;
            xGoal = 0;
        }
        if (gamepad1.aWasPressed()) {
            visualTrack = true;
            telemetry.addData("Tracking", "visual");
        } else if (gamepad1.yWasPressed()) {
            visualTrack = false;
            telemetry.addData("Tracking", "localization");
        }
        telemetry.addData("alliance", alliance.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        follower.update();
    }

    private void PoseAverager(Pose newPose, double proportion) { // Only x and y averaged
        double newX = newPose.getX() * proportion + follower.getPose().getX() * (1 - proportion);
        double newY = newPose.getY() * proportion + follower.getPose().getY() * (1 - proportion);
        follower.setPose(new Pose(newX, newY));
    }

    private Pose Converter(double perpendicular, double parallel) {
        perpendicular = perpendicular * 0.70710678118;
        parallel = parallel * 0.70710678118;
        if (alliance == Alliance.BLUE) {
            recentPoseEstimate = new Pose(16.3582677 + perpendicular - parallel, 130.3740157 - perpendicular - parallel);
        } else {
            recentPoseEstimate = new Pose(127.6417323 - perpendicular - parallel, 130.3740157 - perpendicular + parallel);
        }
        return recentPoseEstimate;
    }

    public double LocalizerZ(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid() && Math.abs(result.getTx())<7) {
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            if (results != null) {
                LLResultTypes.FiducialResult firstResult = results.get(0); //There should never be more than 1 result because it is filtered in the pipeline
                if (firstResult != null) {
                    List<List<Double>> targetCorners = firstResult.getTargetCorners();
                    double R = (distFromPoint(targetCorners.get(1).get(1), 1) + distFromPoint(targetCorners.get(2).get(1), 2)) / 2;
                    double L = (distFromPoint(targetCorners.get(0).get(1), 0) + distFromPoint(targetCorners.get(3).get(1), 3)) / 2;
                    double x = (Math.pow(L, 2) - Math.pow(R, 2)) / (4 * 3.25);
                    double y = Math.sqrt(Math.pow(R, 2) - Math.pow(x - 3.25, 2));
                    PoseAverager(Converter(y, x), 0.3);
                    return Math.sqrt(Math.pow(x, 2) + Math.pow(y + 21.3, 2));
                } else {
                    telemetry.addLine("First Fiducial Result is null");
                }
            } else {
                telemetry.addLine("getFiducialResults returns null, have you 'enabled in output tab'?");
            }
        } else {
            telemetry.addLine("No valid apriltags");
        }
        return 0;
    }

    private double distFromPoint(double tpx, int corner) {
        //Height of apriltag above ground = 29.5
        //Side length of apriltag = 6.5
        double cornerHeight;
        if ((corner == 0) || (corner == 1)) { // Maybe 0,1
            cornerHeight = 29.5 + 3.25;
        } else {
            cornerHeight = 29.5 - 3.25;
        }
        //960 y axis pixels -> 42 degrees
        return (cornerHeight - 13) / Math.tan((20.9 + tpx * 42 / 960 - 21) * Math.PI / 180);//Measured 20.9
    }

    @Override
    public void loop() {
        telemetry.addData("PoseEstimate", recentPoseEstimate);
        LLResult result = limelight.getLatestResult();
        if (state == 1) { //Lineup Phase
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);//Field-Centric
            if (gamepad1.a) {
                double angle;
                if (visualTrack) {
                    angle = result.getTx();
                } else {
                    angle = follower.getPose().getHeading() - Math.atan2(144 - follower.getPose().getY(), xGoal - follower.getPose().getX());
                }
                double pi = Math.PI;
                angle = ((angle + pi) % (2 * pi)) - pi; //Makes angle between -pi and pi
                telemetry.addData("angle", angle);
                telemetry.addData("angleVelocity", follower.getAngularVelocity());
                double rotate = PGain * angle + DGain * follower.getAngularVelocity();
                follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate, false);
            }
            if (gamepad2.dpad_down) {
                flywheelVelocity = LAUNCHER_MIN_VELOCITY;
            } else if (gamepad2.dpad_up) {
                flywheelVelocity = LAUNCHER_MAX_VELOCITY;
            } else if (gamepad2.dpad_left) {
                flywheelVelocity = Regression.getVelocityForDistance(Math.sqrt(Math.pow(144 - follower.getPose().getY(), 2) + Math.pow(xGoal - follower.getPose().getX(), 2)));
            }
            stopper.setPosition(0.5);
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setVelocity(FEEDER_INTAKE_VELOCITY);
            launcher.setDirection(DcMotor.Direction.FORWARD);
            if (gamepad2.a) {
                feeder.setVelocity(FEEDER_STOP_VELOCITY);
            }
            if (gamepad2.rightBumperWasPressed()) {
                state = 2;
            }
        } else if (state == 2) { //shooting
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.4, false);//Field-Centric
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
            launcher.setDirection(DcMotor.Direction.FORWARD);
            stopper.setPosition(1);
            if (gamepad2.rightBumperWasPressed()) {
                state = 1;
            }
        }
        if (result.isValid() && Math.abs(result.getTx())<7) {
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            if (results != null) {
                LLResultTypes.FiducialResult firstResult = results.get(0); //There should never be more than 1 result because it is filtered in the pipeline
                if (firstResult != null) {
                    List<List<Double>> targetCorners = firstResult.getTargetCorners();
                    telemetry.addData("x0, y0:", targetCorners.get(0).toString());
                    telemetry.addData("x1, y1:", targetCorners.get(1).toString());
                    telemetry.addData("x2, y2:", targetCorners.get(2).toString());
                    telemetry.addData("x3, y3:", targetCorners.get(3).toString());
                    double thickness = Math.abs(((targetCorners.get(0).get(0) + targetCorners.get(3).get(0)) - (targetCorners.get(1).get(0) + targetCorners.get(2).get(0))) / 2);
                    double angle = thickness * 54.5 / 1280; //Angle of triangle at limelight, Will not use
                    telemetry.addData("angle", angle);
                    double R = (distFromPoint(targetCorners.get(1).get(1), 1) + distFromPoint(targetCorners.get(2).get(1), 2)) / 2;
                    double L = (distFromPoint(targetCorners.get(0).get(1), 0) + distFromPoint(targetCorners.get(3).get(1), 3)) / 2;
                    telemetry.addData("R", R);
                    telemetry.addData("L", L);
                    double x = (Math.pow(L, 2) - Math.pow(R, 2)) / (4 * 3.25);
                    double y = Math.sqrt(Math.pow(R, 2) - Math.pow(x - 3.25, 2));
                    telemetry.addData("Distance from tag's center", Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
                    telemetry.addData("Perpendicular", y); //This works
                    telemetry.addData("Parallel", x); //This does not work? Maybe? It should work, we need to test
                    double guessedDist = Math.sqrt(Math.pow(x, 2) + Math.pow(y + 21.3, 2));
                    telemetry.addData("Guessed total distance", guessedDist);
                    telemetry.addData("Odometry Distance", Math.sqrt(Math.pow(follower.getPose().getX(), 2) + Math.pow(follower.getPose().getY(), 2)));
                    if (!Converter(y, x).equals(pCoordinates)) {
                        PoseAverager(Converter(y, x), 0.3); //Only update position if limelight is up to date
                    }
                    pCoordinates = Converter(y, x);
                } else {
                    telemetry.addLine("First Fiducial Result is null");
                }
            } else {
                telemetry.addLine("getFiducialResults returns null, have you 'enabled in output tab'?");
            }
        } else {
            telemetry.addLine("No valid apriltags");
        }
        launcher.setVelocity(flywheelVelocity);
        telemetry.addData("state", state);
        telemetry.addData("flywheelVelocity", flywheelVelocity);
        telemetry.update();
    }
}