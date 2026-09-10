package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;



@TeleOp(name = "Color TeleOp Simple")
public class ColorTeleOpSimple extends LinearOpMode {

    private ColorSensor sensorc1;

    @Override
    public void runOpMode() {

        // Inicializar sensor
        sensorc1 = hardwareMap.get(ColorSensor.class, "sensorc1");

        waitForStart();

        while (opModeIsActive()) {

            // Leer RGB
            int r = sensorc1.red();
            int g = sensorc1.green();
            int b = sensorc1.blue();

            // Detectar color (simple)
            String colorDetectado;

            if (r > g && r > b) {
                colorDetectado = "ROJO";
            }
            else if (g > r && g > b) {
                colorDetectado = "VERDE";
            }
            else if (b > r && b > g) {
                colorDetectado = "AZUL";
            }
            else if (r > 150 && g > 150 && b < 120) {
                colorDetectado = "AMARILLO";
            }
            else if (r > 200 && g > 200 && b > 200) {
                colorDetectado = "BLANCO";
            }
            else {
                colorDetectado = "NEGRO / OTRO";
            }

            // Telemetría
            telemetry.addData("RGB", "R: %d G: %d B: %d", r, g, b);
            telemetry.addData("Color detectado", colorDetectado);
            telemetry.update();
        }
    }
}