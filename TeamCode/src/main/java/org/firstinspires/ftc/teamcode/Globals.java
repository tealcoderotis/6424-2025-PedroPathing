package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class Globals {
    public static Alliance alliance = Alliance.UNKNOWN;
    public static Pose endingPose = null;

    public static void resetGlobals() {
        alliance = Alliance.UNKNOWN;
        endingPose = null;
    }
}
