package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Reset Globals")
public class ResetGlobals extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.resetGlobals();
        requestOpModeStop();
    }
}
