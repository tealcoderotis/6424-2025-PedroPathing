package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {
    private final Servo hood;
    private final Timer hoodTimer;
    private boolean isBusy = false;
    private boolean hasExtended = false;
    private boolean hasRetracted = true;
    private final int MOVEMENT_TIME = 500;
    private long stopTime;
    private long timeLeft = MOVEMENT_TIME;
    private HoodState hoodState = HoodState.STOPPED;
    private enum HoodState {
            STOPPED,
            RETRACTING,
            EXTENDING
    }

    public Hood(Servo hood) {
        hoodTimer = new Timer();
        this.hood = hood;
        stopTime = MOVEMENT_TIME;
        hood.setPosition(0.5);
    }

    public void retract() {
        if (!hasRetracted) {
            if (hoodState == HoodState.EXTENDING) {
                timeLeft = stopTime;
            }
            else if (hoodState == HoodState.STOPPED) {
                timeLeft = MOVEMENT_TIME;
            }
            hoodTimer.resetTimer();
            hood.setPosition(0.25);
            isBusy = true;
            hoodState = HoodState.RETRACTING;
        }
    }

    public void extend() {
        if (!hasExtended) {
            if (hoodState == HoodState.RETRACTING) {
                timeLeft = stopTime;
            }
            else if (hoodState == HoodState.STOPPED) {
                timeLeft = MOVEMENT_TIME;
            }
            hoodTimer.resetTimer();
            hood.setPosition(0.75);
            isBusy = true;
            hoodState = HoodState.EXTENDING;
        }
    }

    public void toggle() {
        if (hasExtended || hoodState == HoodState.EXTENDING) {
            retract();
        }
        else {
            extend();
        }
    }

    public void stop() {
        hood.setPosition(0.5);
        hasExtended = false;
        hasRetracted = false;
        isBusy = false;
        stopTime = hoodTimer.getElapsedTime();
    }

    public void update() {
        if (isBusy && hoodTimer.getElapsedTime() >= timeLeft) {
            hood.setPosition(0.5);
            if (hoodState == HoodState.EXTENDING) {
                hasExtended = true;
                hasRetracted = false;
            }
            else if (hoodState == HoodState.RETRACTING) {
                hasRetracted = true;
                hasExtended = false;
            }
            stopTime = MOVEMENT_TIME;
            isBusy = false;
        }
    }
}