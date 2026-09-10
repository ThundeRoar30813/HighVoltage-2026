package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierCurve;
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
        name = "AutonomoRuta3",
        group = "Pedro"
)
public class AutonomoRuta3 extends LinearOpMode {


    // =====================================================
    // PEDRO
    // =====================================================

    private Follower follower;

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;


    // =====================================================
    // SUBSYSTEMS
    // =====================================================

    private ShooterSubsystem shooter;

    private IntakeSubsystem intake;


    // =====================================================
    // COMMANDS
    // =====================================================

    private ShooterCmd shooterCmd;


    // =====================================================
    // LIMELIGHT
    // =====================================================

    private Limelight3A limelight;


    // =====================================================
    // TIEMPOS
    // =====================================================

    private static final double WAIT_BEFORE_SHOOT = 0.30;

    private static final double MAX_SHOOT_TIME = 1.50;

    private static final double WAIT_AFTER_SHOOT = 0.20;

    private static final double WAIT_BETWEEN_PATHS = 0.10;


    // Pedro Visualizer:
    // 2000 ms después del Path 6
    private static final double WAIT_AFTER_PATH_6 = 2.00;


    // =====================================================
    // START POSE
    // =====================================================

    private final Pose startPose =
            new Pose(
                    21.837,
                    120.847,
                    Math.toRadians(140)
            );


    // =====================================================
    // BUILD PATHS
    // =====================================================

    public void buildPaths() {


        // =================================================
        // PATH 1
        // =================================================

        path1 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                21.837,
                                                120.847
                                        ),

                                        new Pose(
                                                46.302,
                                                101.240
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(140)
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
                                                46.302,
                                                101.240
                                        ),

                                        new Pose(
                                                45.764,
                                                81.713
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(0),
                                Math.toRadians(0)
                        )

                        .build();



        // =================================================
        // PATH 3
        // =================================================

        path3 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                45.764,
                                                81.713
                                        ),

                                        new Pose(
                                                16.346,
                                                82.299
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();



        // =================================================
        // PATH 4
        // =================================================

        path4 =
                follower.pathBuilder()

                        .addPath(

                                new BezierCurve(

                                        new Pose(
                                                16.346,
                                                82.299
                                        ),

                                        new Pose(
                                                14.655,
                                                111.344
                                        ),

                                        new Pose(
                                                45.633,
                                                101.378
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(0),
                                Math.toRadians(140)
                        )

                        .build();



        // =================================================
        // PATH 5
        // =================================================

        path5 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                45.633,
                                                101.378
                                        ),

                                        new Pose(
                                                45.116,
                                                59.163
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(0),
                                Math.toRadians(0)
                        )

                        .build();



        // =================================================
        // PATH 6
        // =================================================

        path6 =
                follower.pathBuilder()

                        .addPath(

                                new BezierLine(

                                        new Pose(
                                                45.116,
                                                59.163
                                        ),

                                        new Pose(
                                                17.213,
                                                59.055
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();



        // =================================================
        // PATH 7
        // =================================================

        path7 =
                follower.pathBuilder()

                        .addPath(

                                new BezierCurve(

                                        new Pose(
                                                17.213,
                                                59.055
                                        ),

                                        new Pose(
                                                37.051,
                                                102.084
                                        ),

                                        new Pose(
                                                46.562,
                                                101.848
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(0),
                                Math.toRadians(140)
                        )

                        .build();
    }


    // =====================================================
    // INTAKE AUTOMÁTICO
    //
    // SIEMPRE ACTIVO
    //
    // intake1:
    // siempre 1.0
    //
    // intake2:
    //
    // si HAY pieza:
    // 0
    //
    // si NO hay pieza:
    // -0.50
    // =====================================================

    private void updateIntakeAutomatic() {


        // =================================================
        // MOTOR PRINCIPAL
        //
        // SIEMPRE PRENDIDO
        // =================================================

        intake.intake1.setPower(
                1.0
        );


        // =================================================
        // INDEXER CONTROLADO POR SENSOR
        // =================================================

        if (intake.hasPiece()) {


            // Ya hay una pelota arriba.
            //
            // Detenemos solamente
            // el segundo motor.

            intake.intake2.setPower(
                    0
            );

        } else {


            // No hay pelota todavía.
            //
            // Seguimos subiéndola.

            intake.intake2.setPower(
                    -0.50
            );
        }
    }


    // =====================================================
    // WAIT
    //
    // Shooter e Intake siguen trabajando
    // durante las esperas.
    // =====================================================

    private void waitSeconds(double seconds) {


        ElapsedTime timer =
                new ElapsedTime();


        timer.reset();


        while (
                opModeIsActive()
                        &&
                        timer.seconds() < seconds
        ) {


            // =============================================
            // SHOOTER IDLE
            // =============================================

            shooterCmd.autonomousIdle();


            // =============================================
            // INTAKE SIEMPRE ACTIVO
            // =============================================

            updateIntakeAutomatic();


            // =============================================
            // TELEMETRÍA
            // =============================================

            telemetry.addData(
                    "Esperando",
                    "%.2f / %.2f",
                    timer.seconds(),
                    seconds
            );


            telemetry.addData(
                    "Sensor CM",
                    intake.getDistance()
            );


            telemetry.addData(
                    "Hay pieza",
                    intake.hasPiece()
            );


            telemetry.addData(
                    "Intake 1",
                    intake.intake1.getPower()
            );


            telemetry.addData(
                    "Intake 2",
                    intake.intake2.getPower()
            );


            telemetry.addData(
                    "Shooter",
                    shooter.getAvgVelocity()
            );


            telemetry.update();


            idle();
        }
    }


    // =====================================================
    // FOLLOW PATH
    //
    // EL INTAKE SIEMPRE ESTÁ ACTIVO
    // =====================================================

    private void followPath(
            PathChain path
    ) {


        follower.followPath(
                path
        );


        while (
                opModeIsActive()
                        &&
                        follower.isBusy()
        ) {


            // =============================================
            // PEDRO
            // =============================================

            follower.update();


            // =============================================
            // SHOOTER
            //
            // IDLE 80
            // =============================================

            shooterCmd.autonomousIdle();


            // =============================================
            // INTAKE
            //
            // SIEMPRE ACTIVO
            //
            // PERO EL SEGUNDO MOTOR
            // USA EL SENSOR
            // =============================================

            updateIntakeAutomatic();


            // =============================================
            // TELEMETRÍA
            // =============================================

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
                            follower
                                    .getPose()
                                    .getHeading()
                    )
            );


            telemetry.addData(
                    "Pedro Busy",
                    follower.isBusy()
            );


            telemetry.addData(
                    "Sensor REV CM",
                    intake.getDistance()
            );


            telemetry.addData(
                    "Hay Pieza",
                    intake.hasPiece()
            );


            telemetry.addData(
                    "Intake 1",
                    intake.intake1.getPower()
            );


            telemetry.addData(
                    "Intake 2",
                    intake.intake2.getPower()
            );


            telemetry.addData(
                    "Shooter",
                    shooter.getAvgVelocity()
            );


            telemetry.update();


            idle();
        }


        // =================================================
        // IMPORTANTE
        //
        // NO HACEMOS intake.stop()
        //
        // El intake continúa trabajando
        // entre paths.
        // =================================================

        updateIntakeAutomatic();


        shooterCmd.autonomousIdle();
    }


    // =====================================================
    // SHOOT THREE BALLS
    // =====================================================

    private void shootThreeBalls() {


        // =================================================
        // PREPARAR SHOOTER
        // =================================================

        ElapsedTime readyTimer =
                new ElapsedTime();


        readyTimer.reset();


        while (
                opModeIsActive()
                        &&
                        readyTimer.seconds()
                                <
                                WAIT_BEFORE_SHOOT
        ) {


            // =============================================
            // LIMELIGHT
            //
            // DISTANCIA
            //
            // TABLA
            //
            // INTERPOLACIÓN
            // =============================================

            shooterCmd.autonomousShoot();


            // =============================================
            // DURANTE PREPARACIÓN
            //
            // INTAKE SIGUE ACTIVO
            //
            // El sensor mantiene la pelota
            // lista sin mandarla todavía.
            // =============================================

            updateIntakeAutomatic();


            telemetry.addLine(
                    "PREPARANDO SHOOTER"
            );


            telemetry.addData(
                    "Tiempo",
                    "%.2f / %.2f",
                    readyTimer.seconds(),
                    WAIT_BEFORE_SHOOT
            );


            telemetry.addData(
                    "Distancia",
                    shooterCmd.getLastDistance()
            );


            telemetry.addData(
                    "Target",
                    shooterCmd.getLastAutoVelocity()
            );


            telemetry.addData(
                    "Shooter Real",
                    shooter.getAvgVelocity()
            );


            telemetry.addData(
                    "Sensor",
                    intake.getDistance()
            );


            telemetry.update();


            idle();
        }


        // =================================================
        // DISPARO
        // =================================================

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
        // TERMINA SI:
        //
        // 3 pelotas
        //
        // O
        //
        // 1.5 segundos
        // =================================================

        while (
                opModeIsActive()
                        &&
                        ballsPassed < 3
                        &&
                        shootTimer.seconds()
                                <
                                MAX_SHOOT_TIME
        ) {


            // =============================================
            // SHOOTER
            // =============================================

            shooterCmd.autonomousShoot();


            // =============================================
            // DISPARO
            //
            // DURANTE ESTO IGNORAMOS
            // LA PARADA AUTOMÁTICA.
            //
            // Queremos que las pelotas
            // sigan entrando.
            // =============================================

            intake.feedShooter();


            // =============================================
            // SENSOR REV
            // =============================================

            boolean detected =
                    intake.hasPiece();


            /*
             *
             * TRUE
             *
             * sensor ve pelota
             *
             * ↓
             *
             * FALSE
             *
             * pelota terminó de pasar
             *
             * ↓
             *
             * contador +1
             *
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


            // =============================================
            // TELEMETRÍA
            // =============================================

            telemetry.addLine(
                    "DISPARANDO"
            );


            telemetry.addData(
                    "Pelotas",
                    "%d / 3",
                    ballsPassed
            );


            telemetry.addData(
                    "Tiempo",
                    "%.2f / %.2f",
                    shootTimer.seconds(),
                    MAX_SHOOT_TIME
            );


            telemetry.addData(
                    "Sensor CM",
                    intake.getDistance()
            );


            telemetry.addData(
                    "Distancia Limelight",
                    shooterCmd.getLastDistance()
            );


            telemetry.addData(
                    "Base Tabla",
                    shooterCmd.getLastBaseVelocity()
            );


            telemetry.addData(
                    "Target Shooter",
                    shooterCmd.getLastAutoVelocity()
            );


            telemetry.addData(
                    "Shooter Real",
                    shooter.getAvgVelocity()
            );


            telemetry.update();


            idle();
        }


        // =================================================
        // TERMINÓ DISPARO
        //
        // NO APAGAMOS INTAKE
        //
        // Volvemos al modo automático.
        // =================================================

        updateIntakeAutomatic();


        // =================================================
        // SHOOTER VUELVE A 80
        // =================================================

        shooterCmd.autonomousIdle();


        // =================================================
        // ESPERA
        // =================================================

        waitSeconds(
                WAIT_AFTER_SHOOT
        );
    }


    // =====================================================
    // RUN OPMODE
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
        // SHOOTER CMD
        // =================================================

        shooterCmd =
                new ShooterCmd(
                        shooter,
                        limelight,
                        gamepad1,
                        hardwareMap
                );


        // =================================================
        // PATHS
        // =================================================

        buildPaths();


        // =================================================
        // START POSE
        // =================================================

        follower.setStartingPose(
                startPose
        );


        // =================================================
        // TELEMETRÍA INIT
        // =================================================

        telemetry.addLine(
                "AUTONOMO RUTA 3 LISTO"
        );


        telemetry.addData(
                "Start X",
                startPose.getX()
        );


        telemetry.addData(
                "Start Y",
                startPose.getY()
        );


        telemetry.addData(
                "Heading",
                Math.toDegrees(
                        startPose.getHeading()
                )
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
        // ACTIVAR SHOOTER IDLE
        // =================================================

        shooterCmd.autonomousIdle();


        // =================================================
        // ACTIVAR INTAKE
        //
        // DESDE AQUÍ YA NO SE APAGA
        // =================================================

        updateIntakeAutomatic();



        // =====================================================
        //
        // PATH 1
        //
        // Intake automático
        //
        // =====================================================

        followPath(
                path1
        );


        waitSeconds(
                WAIT_BETWEEN_PATHS
        );


        // =====================================================
        // DISPARO #1
        // =====================================================

        shootThreeBalls();



        // =====================================================
        //
        // PATH 2
        //
        // Intake automático
        //
        // =====================================================

        followPath(
                path2
        );


        waitSeconds(
                WAIT_BETWEEN_PATHS
        );



        // =====================================================
        //
        // PATH 3
        //
        // Intake automático
        //
        // =====================================================

        followPath(
                path3
        );


        waitSeconds(
                WAIT_BETWEEN_PATHS
        );



        // =====================================================
        //
        // PATH 4
        //
        // Intake automático
        //
        // =====================================================

        followPath(
                path4
        );


        waitSeconds(
                WAIT_BETWEEN_PATHS
        );


        // =====================================================
        // DISPARO #2
        // =====================================================

        shootThreeBalls();



        // =====================================================
        //
        // PATH 5
        //
        // Intake automático
        //
        // =====================================================

        followPath(
                path5
        );


        waitSeconds(
                WAIT_BETWEEN_PATHS
        );



        // =====================================================
        //
        // PATH 6
        //
        // Intake automático
        //
        // =====================================================

        followPath(
                path6
        );



        // =====================================================
        //
        // WAIT 2000 ms
        //
        // Intake SIGUE activo
        //
        // Shooter SIGUE activo
        //
        // =====================================================

        waitSeconds(
                WAIT_AFTER_PATH_6
        );



        // =====================================================
        //
        // PATH 7
        //
        // Intake automático
        //
        // =====================================================

        followPath(
                path7
        );


        waitSeconds(
                WAIT_BETWEEN_PATHS
        );



        // =====================================================
        // DISPARO #3
        // =====================================================

        shootThreeBalls();



        // =====================================================
        // TERMINÓ LA RUTA
        //
        // Mantener intake automático
        // y shooter idle.
        // =====================================================

        while (opModeIsActive()) {


            follower.update();


            // =============================================
            // SHOOTER 80
            // =============================================

            shooterCmd.autonomousIdle();


            // =============================================
            // INTAKE SIGUE ACTIVO
            // =============================================

            updateIntakeAutomatic();


            telemetry.addLine(
                    "RUTA 3 TERMINADA"
            );


            telemetry.addData(
                    "Shooter",
                    shooter.getAvgVelocity()
            );


            telemetry.addData(
                    "Sensor",
                    intake.getDistance()
            );


            telemetry.addData(
                    "Hay pieza",
                    intake.hasPiece()
            );


            telemetry.addData(
                    "Intake 1",
                    intake.intake1.getPower()
            );


            telemetry.addData(
                    "Intake 2",
                    intake.intake2.getPower()
            );


            telemetry.update();


            idle();
        }
    }
}