package org.firstinspires.ftc.teamcode.eventListeners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

public class LiftStatusOutput implements LiftPositionListener {
    private final OpMode opMode;

    public LiftStatusOutput(final OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void onPositionReached(LiftStatus status) {
        opMode.telemetry.addData("Lift", "Reached target: " + status);
        opMode.telemetry.update();
        // Send target reached data to dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Lift", "Reached target: " + status);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}