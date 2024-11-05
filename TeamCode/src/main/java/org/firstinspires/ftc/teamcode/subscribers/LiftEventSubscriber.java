package org.firstinspires.ftc.teamcode.subscribers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.events.LiftEventListener;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

public class LiftEventSubscriber implements LiftEventListener {
    private final OpMode opMode;

    public LiftEventSubscriber(final OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void onLiftTargetReached(LiftStatus status) {
        opMode.telemetry.addData("Lift", "Reached target: " + status);
        opMode.telemetry.update();
        // Send target reached data to dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Lift", "Reached target: " + status);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}