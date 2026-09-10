package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo 2 Botones")
public class moverServo extends OpMode {
    private Servo servo;

    // posiciones del servo
    private double POS_IZQ = 0.0;
    private double POS_DER = 1.0;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "sr");
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            servo.setPosition(POS_IZQ);
        }

        if (gamepad1.b) {
            servo.setPosition(POS_DER);
        }
    }
}
