package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.subscribers.LiftEventSubscriber;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private MecanumDrive mecanumDrive;
    private LiftControl liftControl;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(this);
        liftControl = new LiftControl(this);

        // Register LiftSubscriber as the listener for lift events
        LiftEventSubscriber liftSubscriber = new LiftEventSubscriber(this);
        liftControl.setLiftEventListener(liftSubscriber);
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

        // Manual horizontal lift adjustment
        double horizontalPower = gamepad2.left_stick_y;
        liftControl.moveHorizontalLift(horizontalPower);

        // Update lift control
        liftControl.update();
    }
}
