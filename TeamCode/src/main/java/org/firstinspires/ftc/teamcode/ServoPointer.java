package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
public class ServoPointer {

    private Servo servo;
    private double positionRef; // Reference to the position variable
    private static final double DEAD_ZONE_THRESHOLD = 0.5; // Minimum joystick magnitude to update servo
    private final FtcDashboard dashboard;

    /**
     * Constructor for the ServoPointer class.
     *
     * @param opMode      The OpMode instance to retrieve the hardware map from.
     * @param servoName   The name of the servo in the configuration.
     * @param positionRef Reference to the position variable to update.
     */
    public ServoPointer(OpMode opMode, String servoName, double positionRef) {
        this.servo = opMode.hardwareMap.get(Servo.class, servoName);
        this.positionRef = positionRef;
        this.servo.setPosition(positionRef); // Set the initial position of the servo
        dashboard = FtcDashboard.getInstance();
    }

    /**
     * Updates the servo position based on joystick x and y inputs.
     *
     * @param x Joystick x-axis value (-1 to 1).
     * @param y Joystick y-axis value (-1 to 1).
     */
    public void update(double x, double y) {
        // Calculate joystick magnitude (distance from center)
        double magnitude = Math.hypot(x, y);

        // Check if the joystick magnitude exceeds the threshold
        if (magnitude < DEAD_ZONE_THRESHOLD) {
            // Joystick is within the dead zone; maintain last valid position
            servo.setPosition(positionRef);
            return;
        }

        // Normalize angle to servo position (0.0 to 1.0)
        positionRef = mapAngleToServoPosition(x, y); // Update position reference

        // Set servo position
        servo.setPosition(positionRef);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Servo position", positionRef);
        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Maps joystick x, y inputs to a servo position (0.0 to 1.0)
     * for a full 360-degree circle with 0 degrees at South.
     *
     * @param x The joystick x-axis input (-1.0 to 1.0).
     * @param y The joystick y-axis input (-1.0 to 1.0).
     * @return The normalized servo position (0.0 to 1.0).
     */
    public double mapAngleToServoPosition(double x, double y) {
        // Rotate coordinates 90 degrees counter-clockwise to align South with 0 degrees
        double rotatedX = -y;
        double rotatedY = x;

        // Calculate angle in radians
        double angle = Math.atan2(rotatedY, rotatedX);

        // Normalize angle to [0, 2π]
        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        // Map angle to servo position in [0.0, 1.0]
        return angle / (2 * Math.PI);
    }




}
