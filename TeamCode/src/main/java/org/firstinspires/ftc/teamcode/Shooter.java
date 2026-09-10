package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter PIDF 2 Motors")
public class Shooter extends OpMode {

    // ===== MOTORES =====
    DcMotorEx leftMotor, rightMotor;

    // ===== VELOCIDADES (ticks/seg) =====
    double highVelocity = 2900; // 6000 RPM
    double lowVelocity  = 1500; // ~3200 RPM
    double curTargetVelocity = highVelocity;

    // ===== PIDF =====
    double P = 0;
    double F = 0.12;

    // ===== AJUSTE FINO =====
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    boolean yPressed = false;
    boolean bPressed = false;

    @Override
    public void init() {
        leftMotor  = hardwareMap.get(DcMotorEx.class, "school");
        rightMotor = hardwareMap.get(DcMotorEx.class, "shore");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPIDF();

        telemetry.update();
    }

    @Override
    public void loop() {

        // ===== CAMBIAR VELOCIDAD =====
        if (gamepad1.y && !yPressed) {
            curTargetVelocity =
                    (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
            yPressed = true;
        }
        if (!gamepad1.y) yPressed = false;

        // ===== CAMBIAR STEP =====
        if (gamepad2.b && !bPressed) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            bPressed = true;
        }
        if (!gamepad2.b) bPressed = false;

        // ===== AJUSTAR F =====
        if (gamepad2.dpad_left)  F -= stepSizes[stepIndex];
        if (gamepad2.dpad_right) F += stepSizes[stepIndex];

        // ===== AJUSTAR P =====
        if (gamepad2.dpad_up)   P += stepSizes[stepIndex];
        if (gamepad2.dpad_down) P -= stepSizes[stepIndex];

        applyPIDF();

        // ===== APLICAR VELOCIDAD =====
        leftMotor.setVelocity(curTargetVelocity);
        rightMotor.setVelocity(curTargetVelocity);

        // ===== TELEMETRY =====
        double leftVel  = leftMotor.getVelocity();
        double rightVel = rightMotor.getVelocity();
        double avgVel   = (leftVel + rightVel) / 2;

        telemetry.addData("Target (ticks/s)", curTargetVelocity);
        telemetry.addData("Left Vel", String.format("%.1f", leftVel));
        telemetry.addData("Right Vel", String.format("%.1f", rightVel));
        telemetry.addData("Avg Vel", String.format("%.1f", avgVel));
        telemetry.addLine("-------------------------");
        telemetry.addData("P", String.format("%.5f", P));
        telemetry.addData("F", String.format("%.5f", F));
        telemetry.addData("Step", stepSizes[stepIndex]);
        telemetry.update();
    }

    // ===== APLICAR PIDF A AMBOS =====
    private void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}
