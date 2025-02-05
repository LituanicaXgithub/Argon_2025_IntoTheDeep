package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.enums.LiftStatus;
import org.firstinspires.ftc.teamcode.eventListeners.LiftPositionListener;
import org.firstinspires.ftc.teamcode.eventListeners.LiftStatusOutput;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.constants.LiftConstants;
import org.firstinspires.ftc.teamcode.eventHandlers.LiftEventHandler;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private MecanumDrive mecanumDrive;
    private LiftControl liftControl;
    private Servo basketRight;
    private Servo basketLeft;
    private Servo intakeBelt;
    private Servo intakeTilt;

    private boolean previousXState = false;

    private final LiftEventHandler liftEventHandler = new LiftEventHandler();

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(this);
        liftControl = new LiftControl(this);
        LiftStatusOutput liftStatusOutput = new LiftStatusOutput(this);
        liftEventHandler.addListener(liftStatusOutput);

        basketRight = hardwareMap.get(Servo.class, "BasketRight");
        basketLeft = hardwareMap.get(Servo.class, "BasketLeft");
        intakeBelt = hardwareMap.get(Servo.class, "IntakeBelt");
        intakeTilt = hardwareMap.get(Servo.class, "IntakeTilt");
        basketLeft.setPosition(ServoConstants.basketRight_Position_DOWN);
        basketLeft.setPosition(ServoConstants.basketLeft_Position_DOWN);
        intakeTilt.setPosition(ServoConstants.intakeTilt_Position_MID);
    }

    @Override
    public void loop() {

        // Drive control
        double y = -gamepad1.left_stick_y;  // Forward and backward
        double x = gamepad1.left_stick_x;   // Strafing left and right
        double rotation = gamepad1.right_stick_x;  // Rotation

        //Slow mode
        if(gamepad1.left_bumper){
            y = y * 0.2;
            x = x * 0.2;
            rotation = rotation *0.2;
        }
        mecanumDrive.drive(y, x, rotation);

        // Lift control
        if (gamepad1.a) {
            liftControl.moveLiftToHeight(LiftStatus.LOW);
        } else if (gamepad1.b) {
            liftControl.moveLiftToHeight(LiftStatus.MID);
        } else if (gamepad1.y) {
            liftControl.moveLiftToHeight(LiftStatus.HIGH);
        }

        // Update lift control
        liftControl.update();

        //Basket control
        if (gamepad1.x && !previousXState) {
            if (ServoConstants.basket_up) {
                basketRight.setPosition(ServoConstants.basketRight_Position_UP);
                basketLeft.setPosition(ServoConstants.basketLeft_Position_UP);
                ServoConstants.basket_up = false;
            } else {
                basketRight.setPosition(ServoConstants.basketRight_Position_DOWN);
                basketLeft.setPosition(ServoConstants.basketLeft_Position_DOWN);
                ServoConstants.basket_up = true;
            }
        }
        previousXState = gamepad1.x;

        //Intake Belt control. sukimas. Reversas nuspaudus bumperi.
        if(gamepad1.right_bumper)
        {
            intakeBelt.setPosition(0.0);  // pilnas gazas reversas
        }
        else
        {
            intakeBelt.setPosition((1.0));  //pilnas gazas į priekį.
        }

        //intake Tilt control
        // Up pozicija išlaikoma
        // Mid pozicija visada, jeigu ne UP
        //Down pozicija tik laikant nuspaudus left trigerį.
        if(gamepad1.right_trigger > 0.5)
        {
            intakeTilt.setPosition(ServoConstants.intakeTilt_Position_BASKET);
            ServoConstants.intake_Position_BASKET = true;
        }
        else
        {
            if(gamepad1.left_trigger> 0.5)
            {
                intakeTilt.setPosition(ServoConstants.intakeTilt_Position_DOWN);
                ServoConstants.intake_Position_BASKET = false;
            }
            else if(!ServoConstants.intake_Position_BASKET)
            {
                intakeTilt.setPosition(ServoConstants.intakeTilt_Position_MID);
            }
        }


        // horizontal lift control
        int intakePosition;
        if(gamepad1.dpad_up)
        {
            intakePosition = mecanumDrive.frontRight.getCurrentPosition();

            if(intakePosition < LiftConstants.HORIZONTAL_LIFT_MAX_OUT)
            {
                liftControl.moveHorizontalLift(1.0);
            }
            else
            {
                liftControl.moveHorizontalLift(0.0);
            }
        }
        else if(gamepad1.dpad_down)
        {
            intakePosition = mecanumDrive.frontRight.getCurrentPosition();
            if(intakePosition > LiftConstants.HORIZONTAL_LIFT_MAX_IN)
            {
                liftControl.moveHorizontalLift(-1.0);
            }
            else
            {
                liftControl.moveHorizontalLift(0.0);
            }
        }
        else
        {
            liftControl.moveHorizontalLift(0.0);
        }


    }
}
