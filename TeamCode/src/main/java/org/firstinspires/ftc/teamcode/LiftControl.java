package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.constants.LiftConstants;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.events.LiftEventListener;

public class LiftControl {

    // Motors for lift control
    private final DcMotor leftLiftMotor;
    private final DcMotor rightLiftMotor;
    private final DcMotor horizontalLiftMotor;
    private LiftStatus liftStatus;
    private LiftEventListener liftEventListener;
    private final FtcDashboard dashboard;
    private final OpMode opMode;

    // Variables for PID control
    private double integralSum = 0;
    private double lastError = 0;
    private long lastUpdateTime = 0;

    public LiftControl(final OpMode opMode) {
        this.opMode = opMode;
        // Initialize motors and set their behaviors
        leftLiftMotor = opMode.hardwareMap.get(DcMotor.class, "leftLiftMotor");
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor = opMode.hardwareMap.get(DcMotor.class, "rightLiftMotor");
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        //rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalLiftMotor = opMode.hardwareMap.get(DcMotor.class, "horizontalLiftMotor");
        horizontalLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders for all motors
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftStatus = LiftStatus.IDLE;
        dashboard = FtcDashboard.getInstance();
        lastUpdateTime = System.currentTimeMillis();
    }

    // Set a listener for lift events
    public void setLiftEventListener(LiftEventListener listener) {
        this.liftEventListener = listener;
    }

    // Move the lift to a specific height based on the target status
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
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftStatus = targetStatus;
    }

    // Move the horizontal lift to a specific height based on the target status
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

    // Manually move the horizontal lift with a specific power
    public void moveHorizontalLift(double power) {
        horizontalLiftMotor.setPower(power * LiftConstants.HORIZONTAL_LIFT_POWER);
    }

    // Update method to be called in the main loop to manage lift behavior and PID control
    public void update() {
        int currentPosition = leftLiftMotor.getCurrentPosition();
        int targetPosition = leftLiftMotor.getTargetPosition();
        double error = targetPosition - currentPosition;

        // Calculate time difference for PID calculations
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
        lastUpdateTime = currentTime;

        // PID calculations
        integralSum += error * deltaTime;  // Integral part
        double derivative = (error - lastError) / deltaTime;  // Derivative part
        lastError = error;

        // Calculate power using PID formula
        double power = LiftConstants.LIFT_KP * error + LiftConstants.LIFT_KI * integralSum + LiftConstants.LIFT_KD * derivative;

        // Respect the maximum lift power defined in the constants
        power = Math.max(-LiftConstants.LIFT_POWER, Math.min(LiftConstants.LIFT_POWER, power));

        // Set motor powers based on PID control
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(-power);  // Reverse power for right motor

        // Update telemetry to display current lift status and power
        opMode.telemetry.addData("Lift Position", currentPosition);
        opMode.telemetry.addData("Lift Target", targetPosition);
        opMode.telemetry.addData("Lift Power", power);
        opMode.telemetry.addData("Lift Status", liftStatus);
        opMode.telemetry.update();

        // Send data to the dashboard for monitoring
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Lift Position", currentPosition);
        packet.put("Lift Target", targetPosition);
        packet.put("Lift Power*100", power*100);
        packet.put("Lift Status", liftStatus);
        dashboard.sendTelemetryPacket(packet);

        // Check if lift has reached target position within tolerance
        if (liftStatus != LiftStatus.IDLE && Math.abs(error) < 10) {  // Consider target reached if within 10 units
            leftLiftMotor.setPower(0);
            rightLiftMotor.setPower(0);
            if (liftEventListener != null) {
                liftEventListener.onLiftTargetReached(liftStatus);  // Notify listener that target has been reached
            }
            liftStatus = LiftStatus.IDLE;  // Set lift status to IDLE
        }

        // Stop horizontal lift motor if it is not busy
        if (!horizontalLiftMotor.isBusy()) {
            horizontalLiftMotor.setPower(0);
        }
    }
}
