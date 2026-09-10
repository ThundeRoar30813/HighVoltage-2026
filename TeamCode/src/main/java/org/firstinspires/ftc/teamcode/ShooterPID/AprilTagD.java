package org.firstinspires.ftc.teamcode.ShooterPID;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "AprilTag Distancia REAL")
public class AprilTagD extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // Nombre de configuración
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Pipeline de AprilTags
        limelight.pipelineSwitch(0);

        // Iniciar cámara
        limelight.start();

        telemetry.addLine("Esperando AprilTag...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // Obtener AprilTags detectados
                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                if (!tags.isEmpty()) {

                    // Primer tag detectado
                    LLResultTypes.FiducialResult tag = tags.get(0);

                    // Posición relativa cámara -> tag
                    double x =
                            tag.getTargetPoseCameraSpace()
                                    .getPosition().x;

                    double y =
                            tag.getTargetPoseCameraSpace()
                                    .getPosition().y;

                    double z =
                            tag.getTargetPoseCameraSpace()
                                    .getPosition().z;

                    // Distancia real usando Pitágoras 3D
                    double distancia =
                            Math.sqrt((x * x) + (y * y) + (z * z));

                    telemetry.addLine("APRILTAG DETECTADO");

                    telemetry.addData("Tag ID",
                            tag.getFiducialId());

                    telemetry.addData("X", "%.2f m", x);
                    telemetry.addData("Y", "%.2f m", y);
                    telemetry.addData("Z", "%.2f m", z);

                    telemetry.addData("Distancia REAL",
                            "%.2f metros", distancia);

                    telemetry.addData("TX",
                            "%.2f", result.getTx());

                    telemetry.addData("TY",
                            "%.2f", result.getTy());

                } else {

                    telemetry.addLine("No hay AprilTags");
                }

            } else {

                telemetry.addLine("No detecta AprilTag");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}