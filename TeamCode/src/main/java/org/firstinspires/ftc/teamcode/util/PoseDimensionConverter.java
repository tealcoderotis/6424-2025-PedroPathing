package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class PoseDimensionConverter {
    public static Pose2D pose3DToPose2D(Pose3D pose) {
        return new Pose2D(pose.getPosition().unit, pose.getPosition().x, pose.getPosition().y, AngleUnit.RADIANS, pose.getOrientation().getYaw(AngleUnit.RADIANS));
    }
}
