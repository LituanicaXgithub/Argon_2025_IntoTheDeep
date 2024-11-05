package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.constants.LiftConstants;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.events.LiftEventListener;

public class LiftControl {

    private final DcMotor leftLiftMotor;
    private final DcMotor rightLiftMotor;
    private final DcMotor horizontalLiftMotor;
    private LiftStatus liftStatus;
    private LiftEventListener liftEventListener;
    private final FtcDashboard dashboard;
    private final OpMode opMode;

    public LiftControl(final OpMode opMode) {
        this.opMode = opMode;
        leftLiftMotor = opMode.hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = opMode.hardwareMap.get(DcMotor.class, "rightLiftMotor");
        horizontalLiftMotor = opMode.hardwareMap.get(DcMotor.class, "horizontalLiftMotor");
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftStatus = LiftStatus.IDLE;
        dashboard = FtcDashboard.getInstance();
    }

    public void setLiftEventListener(LiftEventListener listener) {
        this.liftEventListener = listener;
    }

    public void moveLiftToHeight(LiftStatus targetStatus) {
        int targetPosition;
        switch (targetStatus) {
            case LOW:
                targetPosition = LiftConstants.LOW_POSITION;
                break;
            case MID:
                targetPosition = LiftConstants.MID_POSITION;
                break;
            case HIGH:
                targetPosition = LiftConstants.HIGH_POSITION;
                break;
            default:
                targetPosition = 0;
                break;
        }

        leftLiftMotor.setTargetPosition(targetPosition);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setPower(LiftConstants.LIFT_POWER);

        // Set right motor power to the same, but reverse direction
        rightLiftMotor.setPower(-LiftConstants.LIFT_POWER);

        liftStatus = targetStatus;
    }

    public void moveHorizontalLiftToHeight(LiftStatus targetStatus) {
        int targetPosition;
        switch (targetStatus) {
            case LOW:
                targetPosition = LiftConstants.HORIZONTAL_LIFT_LOW_POSITION;
                break;
            case MID:
                targetPosition = LiftConstants.HORIZONTAL_LIFT_MID_POSITION;
                break;
            case HIGH:
                targetPosition = LiftConstants.HORIZONTAL_LIFT_HIGH_POSITION;
                break;
            default:
                targetPosition = 0;
                break;
        }

        horizontalLiftMotor.setTargetPosition(targetPosition);
        horizontalLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalLiftMotor.setPower(LiftConstants.HORIZONTAL_LIFT_POWER);
    }

    public void moveHorizontalLift(double power) {
        horizontalLiftMotor.setPower(power * LiftConstants.HORIZONTAL_LIFT_POWER);
    }

    public void update() {
        int currentPosition = leftLiftMotor.getCurrentPosition();
        opMode.telemetry.addData("Lift Position", currentPosition);
        opMode.telemetry.addData("Lift Status", liftStatus);
        opMode.telemetry.update();

        // Send data to the dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Lift Position", currentPosition);
        packet.put("Lift Status", liftStatus);
        dashboard.sendTelemetryPacket(packet);

        if (liftStatus != LiftStatus.IDLE && !leftLiftMotor.isBusy()) {
            leftLiftMotor.setPower(0);
            rightLiftMotor.setPower(0);
            if (liftEventListener != null) {
                liftEventListener.onLiftTargetReached(liftStatus);
            }
            liftStatus = LiftStatus.IDLE;
        }
        if (!horizontalLiftMotor.isBusy()) {
            horizontalLiftMotor.setPower(0);
        }
    }
}
