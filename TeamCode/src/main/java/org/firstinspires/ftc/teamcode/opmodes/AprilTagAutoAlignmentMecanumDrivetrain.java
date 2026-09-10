package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;

@Disabled

@TeleOp(name="AprilTag Auto-Alignment Limelight 3A", group="Tutorials")
public class AprilTagAutoAlignmentMecanumDrivetrain extends OpMode {

    private final DriveSubsystem drive = new DriveSubsystem();
    private Limelight3A limelight;

    // ----------------------------------------------------
    // VARIABLES DEL CONTROLADOR PD (Ajustadas para Limelight)
    // ----------------------------------------------------
    // Con Limelight (grados), puedes empezar probando con 0.01 o mantener tu valor e ir subiendo con el control.
    double KP = 0.01;
    double KD = 0.0005;
    double error = 0;
    double lastError = 0;

    double goalX = 0;           // Centrado
    double angleTolerance = 0.5; // Tolerancia en grados (Limelight es bastante precisa)

    double curTime = 0;
    double lastTime = 0;

    double forward, strafe, rotate;

    // Sintonización en tiempo real
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;
    boolean lastB = false, lastDpadUp = false, lastDpadDown = false, lastDpadLeft = false, lastDpadRight = false;

    @Override
    public void init() {
        drive.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Asegúrate de que el pipeline 0 esté configurado para AprilTags
        limelight.start();

        telemetry.addLine("¡Limelight y Chasis Inicializados!");
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        // 1. LECTURA DE CONTROLES
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        LLResult result = limelight.getLatestResult();

        // 2. LÓGICA DE CONTROL PD
        if (gamepad1.left_trigger > 0.3) {

            if (result != null && result.isValid()) {

                // CORRECCIÓN DE SIGNO:
                // Si TX es positivo (derecha), queremos rotar positivo (derecha).
                // Por lo tanto: error = Valor Actual - Meta
                error = result.getTx() - goalX;

                // --- Cálculo del término Proporcional ---
                double pTerm = error * KP;

                // --- Cálculo del término Derivativo ---
                curTime = getRuntime();
                double dt = curTime - lastTime;
                if (dt <= 0) dt = 0.001;

                double dTerm = ((error - lastError) / dt) * KD;

                // Guardar lecturas de este ciclo para el siguiente (SIEMPRE se deben guardar si el tag es válido)
                lastError = error;
                lastTime = curTime;

                // Verificar tolerancia de éxito
                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    // Clampar la potencia para mantener seguridad de movimiento
                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);
                }

            } else {
                // Si se pierde el Tag mientras presionas el gatillo, reseteamos para evitar picos
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            // Si el gatillo no está presionado, mantenemos el histórico al día
            lastTime = getRuntime();
            lastError = 0;
        }

        // 3. ENVIAR POTENCIAS AL SUBSISTEMA
        drive.drive(forward, strafe, rotate);

        // ----------------------------------------------------
        // AJUSTES EN VIVO (Gamepad 1)
        // ----------------------------------------------------
        double currentStep = stepSizes[stepIndex];

        if (gamepad1.b && !lastB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        lastB = gamepad1.b;

        if (gamepad1.dpad_right && !lastDpadRight) KP += currentStep;
        if (gamepad1.dpad_left && !lastDpadLeft) KP -= currentStep;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_up && !lastDpadUp) KD += currentStep;
        if (gamepad1.dpad_down && !lastDpadDown) KD -= currentStep;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // ----------------------------------------------------
        // TELEMETRÍA
        // ----------------------------------------------------
        telemetry.addData("Modo", (gamepad1.left_trigger > 0.3 && result != null && result.isValid()) ? "AUTO-ALINEANDO" : "MANUAL");
        telemetry.addData("Limelight Detectando", result != null && result.isValid());

        if (result != null && result.isValid()) {
            telemetry.addData("TX", result.getTx());
            telemetry.addData("Error Calculado", error);
        }

        telemetry.addData("Potencia Giro (Rotate)", rotate);
        telemetry.addData("KP (Ajustar con D-pad Izq/Der)", KP);
        telemetry.addData("KD (Ajustar con D-pad Arra/Aba)", KD);
        telemetry.addData("Paso de Ajuste (B)", currentStep);
        telemetry.update();
    }
}