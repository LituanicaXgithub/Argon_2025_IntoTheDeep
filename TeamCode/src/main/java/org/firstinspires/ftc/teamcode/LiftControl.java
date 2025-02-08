package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.constants.LiftConstants;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.eventHandlers.LiftEventHandler;
import org.firstinspires.ftc.teamcode.eventListeners.LiftStatusOutput;

public class LiftControl {

    // Motors for lift control
    private final DcMotor leftLiftMotor;
    private final DcMotor rightLiftMotor;
    private final DcMotor horizontalLiftMotor;
    private LiftStatus liftStatus;
    private int targetPosition;
    private final LiftEventHandler liftEventHandler = new LiftEventHandler();
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
        rightLiftMotor = opMode.hardwareMap.get(DcMotor.class, "rightLiftMotor");
        horizontalLiftMotor = opMode.hardwareMap.get(DcMotor.class, "horizontalLiftMotor");

        // Reset encoders for all motors
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Run without encoders, using custom PID
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftStatus = LiftStatus.IDLE;
        dashboard = FtcDashboard.getInstance();
        lastUpdateTime = System.currentTimeMillis();
    }

    // Move the lift to a specific height based on the target status
    public void moveLiftToHeight(LiftStatus targetStatus) {

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

        liftStatus = targetStatus;
    }



    // Manually move the horizontal lift with a specific power
    public void moveHorizontalLift(double power) {
        horizontalLiftMotor.setPower(power * LiftConstants.HORIZONTAL_LIFT_POWER);
    }



    // Update method to be called in the main loop to manage lift behavior and PID control
    public void update() {
        int currentPosition = rightLiftMotor.getCurrentPosition();
        //int targetPosition = rightLiftMotor.getTargetPosition();
        double error = targetPosition - currentPosition;

        // Calculate time difference for PID calculations
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
        lastUpdateTime = currentTime;

        double kp, ki, kd;
        if (error > 0) {  // Moving upward
            kp = LiftConstants.LIFT_UP_KP;
            ki = LiftConstants.LIFT_UP_KI;
            kd = LiftConstants.LIFT_UP_KD;
        } else {          // Moving downward (or zero error)
            kp = LiftConstants.LIFT_DOWN_KP;
            ki = LiftConstants.LIFT_DOWN_KI;
            kd = LiftConstants.LIFT_DOWN_KD;
        }

// PID calculations
        integralSum += error * deltaTime;

// Optional: Clamp the integral to prevent windup
        double maxIntegral = 1000; // Adjust this value based on testing
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));

        double derivative = (error - lastError) / deltaTime;
        lastError = error;

// Compute output power from PID formula
        double power = kp * error + ki * integralSum + kd * derivative;

// Clamp the output power to the maximum allowed
        power = Math.max(-LiftConstants.LIFT_POWER, Math.min(LiftConstants.LIFT_POWER, power));


        // Set motor powers based on PID control
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);  // Reverse power for right motor

        // Update telemetry to display current lift status and power
        opMode.telemetry.addData("Lift Position", currentPosition);
        opMode.telemetry.addData("Lift Target", targetPosition);
        opMode.telemetry.addData("Lift Power", power);
        opMode.telemetry.addData("Lift Status", liftStatus);
        //opMode.telemetry.update();

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
            liftEventHandler.notifyListeners(liftStatus);
            liftStatus = LiftStatus.IDLE;  // Set lift status to IDLE
        }

    }
}
