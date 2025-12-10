package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LimelightPoseCorrector {
    private Limelight3A limelight;
    private static final String LIMELIGHT_HARDWARE_MAP_NAME = "limelight";
    private long lastTimeStamp = 0;
    private Telemetry telemetry;

    public LimelightPoseCorrector(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_HARDWARE_MAP_NAME);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LimelightPoseCorrector(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public Pose correctPose(Pose pose) {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid() && result.getControlHubTimeStamp() > lastTimeStamp) {
                Pose limelightPose = PoseConverter.pose2DToPose(PoseDimensionConverter.pose3DToPose2D(result.getBotpose()), InvertedFTCCoordinates.INSTANCE);
                pose = pose.setHeading(limelightPose.getHeading());
                lastTimeStamp = result.getControlHubTimeStamp();
            }
        }
        return pose;
    }

    public Pose getLimelightPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                if (telemetry != null) {
                    telemetry.addData("x", result.getBotpose().getPosition().x);
                    telemetry.addData("y", result.getBotpose().getPosition().y);
                    telemetry.addData("heading", result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
                }
                Pose limelightPose = PoseConverter.pose2DToPose(PoseDimensionConverter.pose3DToPose2D(result.getBotpose()), InvertedFTCCoordinates.INSTANCE);
                return limelightPose;
            }
        }
        return null;
    }
}
