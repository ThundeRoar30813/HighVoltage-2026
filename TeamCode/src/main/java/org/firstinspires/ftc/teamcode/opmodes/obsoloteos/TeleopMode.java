package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import   com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled

@TeleOp(name = "Simple Teleop", group = "Teleop")
public class TeleopMode extends LinearOpMode {

    // ========== DECLARACIÓN DE MOTORES ==========
    // Motores del shooter (2 motores)
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private double shooterTargetVelocity = 0;      // Velocidad objetivo en ticks/segundo
    private final double SHOOTER_MAX_VELOCITY = 6500;
    private final double SHOOTER_MIN_VELOCITY = 0;

    // Constantes para el PID del shooter
    private final double SHOOTER_KP = 0.0005;      // Ganancia proporcional

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== INICIALIZACIÓN DE MOTORES ==========
        // Shooter izquierdo
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shore");
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Shooter derecho
        shooterRight = hardwareMap.get(DcMotorEx.class, "school");
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);  // Invertido para que giren juntos
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ========== ESPERAR A QUE INICIE EL DRIVER ==========
        telemetry.addData("Status", "Inicializado. Listo para iniciar");
        telemetry.addData("Controles", "D-Pad Abajo = +100 ticks Shooter");
        telemetry.addData("Controles", "Right Bumper = Intake Reversa");
        telemetry.update();

        waitForStart();

        // ========== BUCLE PRINCIPAL ==========
        while (opModeIsActive()) {
            // ==========================================
            // 1. CONTROL DEL SHOOTER (D-Pad Abajo)
            // ==========================================
            if (gamepad2.dpad_down) {
                shooterTargetVelocity += 150;
                if (shooterTargetVelocity > SHOOTER_MAX_VELOCITY) {
                    shooterTargetVelocity = SHOOTER_MAX_VELOCITY;
                }


                // Pequeña pausa para no sumar múltiples veces por una sola presión
                sleep(150);
            }

            // D-Pad Arriba para disminuir (opcional)
            if (gamepad2.dpad_up) {
                shooterTargetVelocity -= 150;
                if (shooterTargetVelocity < SHOOTER_MIN_VELOCITY) {
                    shooterTargetVelocity = SHOOTER_MIN_VELOCITY;
                }
                telemetry.addData("INFO", "Velocidad disminuida a: " + shooterTargetVelocity);
                sleep(150);
            }

            // Botón Y para detener el shooter
            if (gamepad2.y) {
                shooterTargetVelocity = 0;
                telemetry.addData("INFO", "Shooter detenido");
                sleep(150);
            }

            // ===== CONTROL DEL SHOOTER (PID SIMPLE) =====
            // Obtener velocidad actual (promedio de los dos motores)
            double leftVelocity = shooterLeft.getVelocity();
            double rightVelocity = shooterRight.getVelocity();
            double currentVelocity = (leftVelocity + rightVelocity) / 2;

            // Calcular error
            double error = shooterTargetVelocity - currentVelocity;

            // Calcular potencia con P-gain
            double power = error * SHOOTER_KP;

            // Limitar potencia entre -1 y 1
            if (power > 1.0) power = 1.0;
            if (power < -1.0) power = -1.0;

            shooterLeft.setPower(power);
            shooterRight.setPower(power);



            telemetry.addData("INFO", "Velocidad aumentada a: " + shooterTargetVelocity);
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Velocidad Actual", "%.0f ticks/seg", currentVelocity);
            telemetry.addData("Velocidad Objetivo", "%.0f ticks/seg", shooterTargetVelocity);
            telemetry.addData("Potencia", "%.2f", power);
            telemetry.addData("Error", "%.0f", error);
            telemetry.addData("=== CONTROLES ===", "");
            telemetry.addData("Shooter +100", "D-Pad Abajo");
            telemetry.addData("Shooter -100", "D-Pad Arriba");
            telemetry.addData("Shooter Stop", "Y");
            telemetry.addData("Intake Reversa", "Right Bumper (mantener)");
            telemetry.addData("Intake Adelante", "Left Bumper (mantener)");
            telemetry.addData("Intake 1 seg", "X");
            telemetry.addData("Intake Stop", "B");
            telemetry.update();
            sleep(20);
        }

        // ========== AL DETENER EL OPMODE ==========
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}

















