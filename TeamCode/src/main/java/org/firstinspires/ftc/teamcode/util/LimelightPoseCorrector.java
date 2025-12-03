package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightPoseCorrector {
    private Limelight3A limelight;
    private static final String LIMELIGHT_HARDWARE_MAP_NAME = "limelight";

    public LimelightPoseCorrector(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_HARDWARE_MAP_NAME);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Pose correctPose(Pose pose) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            pose = PoseConverter.pose2DToPose(PoseDimensionConverter.pose3DToPose2D(result.getBotpose()), InvertedFTCCoordinates.INSTANCE);
        }
        return pose;
    }
}
