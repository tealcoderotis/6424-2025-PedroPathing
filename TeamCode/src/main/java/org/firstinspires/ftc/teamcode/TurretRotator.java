package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseTrig;

public class TurretRotator {
    //TODO get starting angle and current from encoder
    //TODO add op mode to reset encoder
    //TODO make turret motor actually move
    private final DcMotorEx turretRotate;
    private static final Pose RED_GOAL_POSE = new Pose(144, 144);
    private static final Pose BLUE_GOAL_POSE = new Pose(0, 144);
    private double currentAngle;
    private Telemetry telemetry;
    private Alliance alliance;
    public TurretRotator(HardwareMap hardwareMap, Alliance alliance) {
        turretRotate = (DcMotorEx)hardwareMap.get(Globals.TURRET_ROTATE_HARDWARE_MAP_NAME);
        turretRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.alliance = alliance;
    }

    public TurretRotator(HardwareMap hardwareMap, Alliance alliance, Telemetry telemetry) {
        this(hardwareMap, alliance);
        this.telemetry = telemetry;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public double getAngleRadians() {
        return this.currentAngle;
    }

    public void update(Pose botPose) {
        if (alliance != Alliance.UNKNOWN) {
            double targetAngle;
            if (alliance == Alliance.RED) {
                targetAngle = PoseTrig.angleBetweenPoses(botPose, RED_GOAL_POSE);
            }
            else {
                targetAngle = PoseTrig.angleBetweenPoses(botPose, BLUE_GOAL_POSE);
            }
            if (telemetry != null) {
                telemetry.addData("Angle to goal", Math.toDegrees(targetAngle));
            }
        }
    }
}
