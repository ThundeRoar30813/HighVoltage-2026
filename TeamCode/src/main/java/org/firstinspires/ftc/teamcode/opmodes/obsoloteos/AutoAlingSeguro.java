package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "AutoAling SeguroPROBAR4", group = "Teleop")
public class AutoAlingSeguro extends LinearOpMode {
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    // Posición simulada (cambia con joysticks)
    private double robotX = 9;
    private double robotY = 9;
    private double robotHeading = 90;

    // Objetivo
    private double targetX = 8;
    private double targetY = 136;

    // Parámetros auto-aling
    private final double DEADBAND = Math.toRadians(3);
    private final double MIN_SPEED = 0.1;
    private final double MAX_SPEED = 1.0;
    private final double MAX_ERROR = Math.toRadians(90);

    @Override
    public void runOpMode() {

        // Inicializar motores
        motorFrontLeft = hardwareMap.get(DcMotor.class, "mfl");
        motorFrontRight = hardwareMap.get(DcMotor.class, "mfr");
        motorBackLeft = hardwareMap.get(DcMotor.class, "mbl");
        motorBackRight = hardwareMap.get(DcMotor.class, "mbr");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "✅ LISTO");
        telemetry.addData("CONTROLES", "");
        telemetry.addData("  LEFT TRIGGER", "Auto-Aling (GIRA SOLO)");
        telemetry.addData("  LEFT STICK", "Movimiento (adelante/strafe)");
        telemetry.addData("  X", "Objetivo AZUL");
        telemetry.addData("  B", "Objetivo ROJO");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========== SIMULAR MOVIMIENTO ==========
            if (gamepad2.left_stick_y != 0) {
                robotX += gamepad2.left_stick_y * 2;
            }
            if (gamepad2.left_stick_x != 0) {
                robotY += gamepad2.left_stick_x * 2;
            }

            robotX = Math.max(0, Math.min(144, robotX));
            robotY = Math.max(0, Math.min(144, robotY));

            // ========== CAMBIAR OBJETIVO ==========
            if (gamepad2.x) {
                targetX = 8;
                targetY = 136;
                sleep(150);
            }
            if (gamepad2.b) {
                targetX = 136;
                targetY = 8;
                sleep(150);
            }

            // ========== CALCULAR EL GIRO DEL AUTO-ALING ==========
            double giro = 0;
            boolean autoAlingActivo = false;

            if (gamepad2.left_trigger > 0.5) {
                autoAlingActivo = true;

                double deltaX = targetX - robotX;
                double deltaY = targetY - robotY;

                if (Math.abs(deltaX) > 0.01 || Math.abs(deltaY) > 0.01) {
                    double targetAngle = Math.atan2(deltaY, deltaX);
                    double error = targetAngle - robotHeading;
                    error = Math.atan2(Math.sin(error), Math.cos(error));

                    if (Math.abs(error) > DEADBAND) {
                        double speed = Math.abs(error) / MAX_ERROR;
                        if (speed < MIN_SPEED) speed = MIN_SPEED;
                        if (speed > MAX_SPEED) speed = MAX_SPEED;
                        giro = speed * Math.signum(error);

                        // APLICAR EL GIRO AL HEADING SIMULADO
                        robotHeading += giro * 0.05;
                        robotHeading = Math.atan2(Math.sin(robotHeading), Math.cos(robotHeading));
                    }
                }
            }

            // ========== MOVIMIENTO DEL ROBOT ==========
            double adelante = -gamepad2.left_stick_x;
            double strafe = -gamepad2.left_stick_y;

            // Si auto-aling está activo, USA el giro calculado en lugar del manual
            double giroFinal = autoAlingActivo ? giro : gamepad2.right_stick_x;

            // Fórmula mecanum
            double fl = adelante + strafe + giroFinal;
            double fr = adelante - strafe - giroFinal;
            double bl = adelante - strafe + giroFinal;
            double br = adelante + strafe - giroFinal;

            motorFrontLeft.setPower(fl);
            motorFrontRight.setPower(fr);
            motorBackLeft.setPower(bl);
            motorBackRight.setPower(br);

            // ========== TELEMETRÍA ==========
            double distancia = Math.hypot(targetX - robotX, targetY - robotY);
            double errorGrados = 0;
            if (Math.abs(targetX - robotX) > 0.01 || Math.abs(targetY - robotY) > 0.01) {
                double targetAngle = Math.atan2(targetY - robotY, targetX - robotX);
                double error = targetAngle - robotHeading;
                error = Math.atan2(Math.sin(error), Math.cos(error));
                errorGrados = Math.toDegrees(error);
            }

            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("📍 POSICION", "");
            telemetry.addData("   X", "%.1f", robotX);
            telemetry.addData("   Y", "%.1f", robotY);
            telemetry.addData("   Heading", "%.0f°", Math.toDegrees(robotHeading));
            telemetry.addLine("");
            telemetry.addData("🎯 OBJETIVO", "");
            telemetry.addData("   Distancia", "%.1f in", distancia);
            telemetry.addData("   Error", "%.1f°", errorGrados);
            telemetry.addLine("");
            telemetry.addData("🎮 MODO", autoAlingActivo ? "🔥 AUTO-ALING" : "MANUAL");
            telemetry.addData("🔄 GIRO", "%.2f", giroFinal);


            telemetry.update();

            sleep(20);
        }

        // Detener motores
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}