package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.eventListeners.LiftPositionListener;
import org.firstinspires.ftc.teamcode.eventListeners.LiftStatusOutput;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private MecanumDrive mecanumDrive;
    private LiftControl liftControl;
    private ServoPointer servoPointer0;
    private ServoPointer servoPointer1;
    private ServoPointer servoPointer2;
    private ServoPointer servoPointer3;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(this);
        liftControl = new LiftControl(this);
        servoPointer0 = new ServoPointer(this, "servo0", ServoConstants.Servo0_Position);
        servoPointer1 = new ServoPointer(this, "servo1", ServoConstants.Servo1_Position);
        servoPointer2 = new ServoPointer(this, "servo2", ServoConstants.Servo2_Position);
        servoPointer3 = new ServoPointer(this, "servo3", ServoConstants.Servo3_Position);

        // Register test listener to Lift event
        LiftPositionListener liftStatusListener = new LiftStatusOutput(this);
        liftControl.setLiftEventListener(liftStatusListener);
    }

    @Override
    public void loop() {
        // Drive control
        double y = -gamepad1.left_stick_y;  // Forward and backward
        double x = gamepad1.left_stick_x;   // Strafing left and right
        double rotation = gamepad1.right_stick_x;  // Rotation
        mecanumDrive.drive(y, x, rotation);

        // Lift control
        if (gamepad2.a) {
            liftControl.moveLiftToHeight(LiftStatus.LOW);
        } else if (gamepad2.b) {
            liftControl.moveLiftToHeight(LiftStatus.MID);
        } else if (gamepad2.y) {
            liftControl.moveLiftToHeight(LiftStatus.HIGH);
        }

        // Horizontal lift control
        if (gamepad2.dpad_up) {
            liftControl.moveHorizontalLiftToHeight(LiftStatus.HIGH);
        } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
            liftControl.moveHorizontalLiftToHeight(LiftStatus.MID);
        } else if (gamepad2.dpad_down) {
            liftControl.moveHorizontalLiftToHeight(LiftStatus.LOW);
        }


        // Get joystick inputs
        double x2 = gamepad2.left_stick_x; // Left/right axis
        double y2 = -gamepad2.left_stick_y; // Forward/backward axis (negate for correct direction)

        // Update the servo position using the ServoPointer class
        servoPointer0.update(x2, y2);
        servoPointer1.update(x2, y2);
        servoPointer2.update(x2, y2);
        servoPointer3.update(x2, y2);



        // Manual horizontal lift adjustment
        double horizontalPower = gamepad2.left_stick_y;
        liftControl.moveHorizontalLift(horizontalPower);

        // Update lift control
        liftControl.update();
    }
}
