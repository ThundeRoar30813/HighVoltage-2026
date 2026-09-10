package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "AutonomoRuta2", group = "Pedro")
public class AutonomoRuta2 extends LinearOpMode {

    private Follower follower;

    private PathChain mainChain;


    // =====================================================
    // POSICION INICIAL
    // =====================================================

    private final Pose startPose = new Pose(
            22.719,
            123.272,
            Math.toRadians(142)
    );


    // =====================================================
    // CONSTRUIR RUTA
    // =====================================================

    public void buildPaths() {

        mainChain = follower.pathBuilder()

                // ==========================================
                // PATH 1
                //
                // (22.719, 123.272)
                //          ↓
                // (38.147, 110.276)
                //
                // Heading = 142°
                // ==========================================

                .addPath(
                        new BezierLine(
                                new Pose(22.719, 123.272),
                                new Pose(38.147, 110.276)
                        )
                )

                .setConstantHeadingInterpolation(
                        Math.toRadians(142)
                )


                // ==========================================
                // PATH 2
                //
                // Bezier Curve
                //
                // Inicio:
                // (38.147, 110.276)
                //
                // Control:
                // (76.620, 81.486)
                //
                // Final:
                // (43.069, 81.773)
                //
                // Heading:
                // 142° → 0°
                // ==========================================

                .addPath(
                        new BezierCurve(
                                new Pose(38.147, 110.276),
                                new Pose(76.620, 81.486),
                                new Pose(43.069, 81.773)
                        )
                )

                .setLinearHeadingInterpolation(
                        Math.toRadians(142),
                        Math.toRadians(0)
                )


                // ==========================================
                // PATH 3
                //
                // (43.069, 81.773)
                //          ↓
                // (15.380, 82.494)
                //
                // Heading = 0°
                // ==========================================

                .addPath(
                        new BezierLine(
                                new Pose(43.069, 81.773),
                                new Pose(15.380, 82.494)
                        )
                )

                .setConstantHeadingInterpolation(
                        Math.toRadians(0)
                )

                .build();
    }


    // =====================================================
    // OPMODE
    // =====================================================

    @Override
    public void runOpMode() {

        // Crear follower
        follower = Constants.createFollower(hardwareMap);


        // Construir rutas
        buildPaths();


        // Posición inicial
        follower.setStartingPose(startPose);


        telemetry.addLine("Ruta Autonomo 2 lista");

        telemetry.addData(
                "Start X",
                startPose.getX()
        );

        telemetry.addData(
                "Start Y",
                startPose.getY()
        );

        telemetry.addData(
                "Start Heading",
                Math.toDegrees(
                        startPose.getHeading()
                )
        );

        telemetry.update();


        waitForStart();


        if (isStopRequested()) {
            return;
        }


        // Ejecutar todo el PathChain
        follower.followPath(mainChain);


        // =================================================
        // LOOP
        // =================================================

        while (opModeIsActive()) {

            // Actualizar posición y follower
            follower.update();


            telemetry.addData(
                    "X",
                    follower.getPose().getX()
            );

            telemetry.addData(
                    "Y",
                    follower.getPose().getY()
            );

            telemetry.addData(
                    "Heading",
                    Math.toDegrees(
                            follower.getPose().getHeading()
                    )
            );

            telemetry.addData(
                    "Busy",
                    follower.isBusy()
            );

            telemetry.update();
        }
    }
}