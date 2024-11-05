package org.firstinspires.ftc.teamcode.events;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
public interface LiftEventListener {
    void onLiftTargetReached(LiftStatus status);
}
