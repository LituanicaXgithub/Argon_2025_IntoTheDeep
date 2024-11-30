package org.firstinspires.ftc.teamcode.eventListeners;

import org.firstinspires.ftc.teamcode.enums.LiftStatus;

public interface LiftPositionListener {
    void onPositionReached(LiftStatus status);
}
