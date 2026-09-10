package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;

import org.firstinspires.ftc.teamcode.commands.shooter.ShooterCmd;

@Disabled


@Autonomous(
        name = "AutonomoRuta1",
        group = "Pedro"
)
public class AutonomoRuta1 extends LinearOpMode {


    // =====================================================
    // PEDRO
    // =====================================================

    private Follower follower;


    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;


    // =====================================================
    // SUBSYSTEMS
    // =====================================================

    private ShooterSubsystem shooter;

    private IntakeSubsystem intake;

    private ShooterCmd shooterCmd;


    // =====================================================
    // LIMELIGHT
    // =====================================================

    private Limelight3A limelight;


    // =====================================================
    // START
    // =====================================================

    private final Pose startPose =
            new Pose(
                    22.719,
                    119.966,
                    Math.toRadians(142)
            );


    // =====================================================
    // CONSTRUIR PATHS
    // =====================================================

    public void buildPaths() {


        // =================================================
        // PATH 1
        //
        // INICIO → PRIMER DISPARO
        // =================================================

        path1 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                22.719,
                                                119.966
                                        ),

                                        new Pose(
                                                41.233,
                                                103.885
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(142)
                        )

                        .build();



        // =================================================
        // PATH 2
        // =================================================

        path2 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                41.233,
                                                103.885
                                        ),

                                        new Pose(
                                                41.419,
                                                83.083
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();



        // =================================================
        // PATH 3
        //
        // RECOGER PELOTAS
        // =================================================

        path3 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                41.419,
                                                83.083
                                        ),

                                        new Pose(
                                                18.446,
                                                82.751
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();



        // =================================================
        // PATH 4
        //
        // REGRESAR A DISPARAR
        // =================================================

        path4 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                18.446,
                                                82.751
                                        ),

                                        new Pose(
                                                40.051,
                                                104.994
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(

                                Math.toRadians(0),

                                Math.toRadians(142)
                        )

                        .build();
    }



    // =====================================================
    // SEGUIR PATH
    // =====================================================

    private void followPath(
            PathChain path,
            boolean collect
    ) {

        follower.followPath(
                path
        );


        while (
                opModeIsActive()
                        &&
                        follower.isBusy()
        ) {

            follower.update();


            // =============================================
            // SHOOTER SIEMPRE PRENDIDO A 80
            // =============================================

            shooterCmd.autonomousIdle();


            // =============================================
            // RECOGER PELOTAS
            // =============================================

            if (collect) {

                intake.autoCollect();

            } else {

                intake.stop();
            }


            telemetry.addData(
                    "X",
                    follower.getPose().getX()
            );

            telemetry.addData(
                    "Y",
                    follower.getPose().getY()
            );

            telemetry.addData(
                    "Shooter",
                    shooter.getAvgVelocity()
            );

            telemetry.addData(
                    "Sensor",
                    intake.getDistance()
            );

            telemetry.update();
        }


        intake.stop();
    }



    // =====================================================
    // DISPARAR 3 PELOTAS
    // =====================================================

    private void shootThreeBalls() {


        ElapsedTime shootTimer =
                new ElapsedTime();


        ElapsedTime debounceTimer =
                new ElapsedTime();


        int ballsPassed =
                0;


        boolean previousDetected =
                intake.hasPiece();


        shootTimer.reset();

        debounceTimer.reset();


        // =================================================
        // MÁXIMO 1.5 SEGUNDOS
        // =================================================

        while (
                opModeIsActive()

                        &&

                        ballsPassed < 3

                        &&

                        shootTimer.seconds() < 1.5
        ) {


            // =============================================
            // LIMELIGHT + TABLA + INTERPOLACIÓN
            // =============================================

            shooterCmd.autonomousShoot();


            // =============================================
            // ALIMENTAR SHOOTER
            // =============================================

            intake.feedShooter();


            // =============================================
            // SENSOR REV
            // =============================================

            boolean detected =
                    intake.hasPiece();


            /*
             * Contamos cuando:
             *
             * sensor veía pelota
             *
             *       TRUE
             *
             * y después deja de verla
             *
             *       FALSE
             *
             * Eso significa que la pelota
             * terminó de pasar por el sensor.
             */

            if (
                    previousDetected
                            &&
                            !detected
                            &&
                            debounceTimer.milliseconds() > 70
            ) {

                ballsPassed++;

                debounceTimer.reset();
            }


            previousDetected =
                    detected;


            telemetry.addLine(
                    "DISPARANDO"
            );


            telemetry.addData(
                    "Pelotas",
                    ballsPassed
            );


            telemetry.addData(
                    "Tiempo",
                    shootTimer.seconds()
            );


            telemetry.addData(
                    "Distancia Limelight",
                    shooterCmd.getLastDistance()
            );


            telemetry.addData(
                    "Target Shooter",
                    shooterCmd.getLastAutoVelocity()
            );


            telemetry.addData(
                    "Shooter Real",
                    shooter.getAvgVelocity()
            );


            telemetry.addData(
                    "Sensor REV",
                    intake.getDistance()
            );


            telemetry.update();
        }


        // =================================================
        // TERMINÓ EL DISPARO
        // =================================================

        intake.stop();


        // Regresar inmediatamente al idle
        shooterCmd.autonomousIdle();
    }



    // =====================================================
    // OPMODE
    // =====================================================

    @Override
    public void runOpMode() {


        // =================================================
        // PEDRO
        // =================================================

        follower =
                Constants.createFollower(
                        hardwareMap
                );


        // =================================================
        // SHOOTER
        // =================================================

        shooter =
                new ShooterSubsystem();

        shooter.init(
                hardwareMap
        );


        // =================================================
        // INTAKE
        // =================================================

        intake =
                new IntakeSubsystem();

        intake.init(
                hardwareMap
        );


        // =================================================
        // LIMELIGHT
        // =================================================

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );


        limelight.pipelineSwitch(
                0
        );

        limelight.start();


        // =================================================
        // SHOOTER COMMAND
        //
        // gamepad2 solamente se pasa para mantener
        // compatibilidad con tu constructor actual.
        // =================================================

        shooterCmd =
                new ShooterCmd(
                        shooter,
                        limelight,
                        gamepad1,
                        hardwareMap
                );


        // =================================================
        // RUTAS
        // =================================================

        buildPaths();


        follower.setStartingPose(
                startPose
        );


        telemetry.addLine(
                "Autonomo listo"
        );

        telemetry.addData(
                "Shooter IDLE",
                80
        );

        telemetry.update();



        // =================================================
        // START
        // =================================================

        waitForStart();


        if (isStopRequested()) {

            return;
        }


        // =================================================
        // SHOOTER PRENDIDO DESDE EL INICIO
        // =================================================

        shooterCmd.autonomousIdle();



        // =================================================
        // PATH 1
        //
        // Ir hacia primer disparo
        // =================================================

        followPath(
                path1,
                false
        );



        // =================================================
        // DISPARO 1
        //
        // LIMELIGHT
        // +
        // TABLA
        // +
        // SENSOR
        // =================================================

        shootThreeBalls();



        // =================================================
        // PATH 2
        //
        // EMPEZAMOS A RECOGER
        // =================================================

        followPath(
                path2,
                true
        );



        // =================================================
        // PATH 3
        //
        // SEGUIR RECOGIENDO
        // =================================================

        followPath(
                path3,
                true
        );



        // =================================================
        // PATH 4
        //
        // Regresar hacia zona de tiro.
        //
        // Shooter sigue a 80.
        // =================================================

        followPath(
                path4,
                false
        );



        // =================================================
        // DISPARO 2
        // =================================================

        shootThreeBalls();



        // =================================================
        // TERMINAMOS
        // =================================================

        intake.stop();

        shooterCmd.autonomousIdle();


        while (opModeIsActive()) {

            follower.update();

            shooterCmd.autonomousIdle();


            telemetry.addLine(
                    "AUTONOMO TERMINADO"
            );

            telemetry.addData(
                    "Shooter",
                    shooter.getAvgVelocity()
            );

            telemetry.update();
        }
    }
}