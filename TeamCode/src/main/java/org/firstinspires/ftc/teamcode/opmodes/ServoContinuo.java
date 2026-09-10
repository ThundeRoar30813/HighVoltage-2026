package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Servo Continuo 2 Botones")
public class ServoContinuo extends LinearOpMode {

    private CRServo servo;

    @Override
    public void runOpMode() {

        // Nombre configurado en el Driver Hub
        servo = hardwareMap.get(CRServo.class, "servo0");

        telemetry.addLine("Listo");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Botón A = una dirección
            if (gamepad1.a) {
                servo.setPower(1.0);
            }

            // Botón B = dirección contraria
            else if (gamepad1.b) {
                servo.setPower(-1.0);
            }

            // Ningún botón = detener
            else {
                servo.setPower(0.0);
            }
        }

        servo.setPower(0.0);
    }
}