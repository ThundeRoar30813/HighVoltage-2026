package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;

import org.firstinspires.ftc.teamcode.commands.shooter.ShooterCmd;


@Autonomous(
        name = "AutoFarBlue",
        group = "Pedro"
)
public class AutoFarBlue extends LinearOpMode {


    // =========================================================
    // PEDRO
    // =========================================================

    private Follower follower;

    private PathChain path1;
    private PathChain path2;


    // =========================================================
    // SUBSYSTEMS
    // =========================================================

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;


    // =========================================================
    // SHOOTER COMMAND
    // =========================================================

    private ShooterCmd shooterCmd;


    // =========================================================
    // LIMELIGHT
    // =========================================================

    private Limelight3A limelight;


    // =========================================================
    // SHOOTER / AIM SETTINGS
    // =========================================================

    private static final double MAX_READY_TIME = 1.50;

    private static final double READY_STABLE_TIME = 0.005;

    private static final double CAPUCHA_SETTLE_TIME = 0.03;

    private static final double CAPUCHA_POSITION_EPSILON = 0.001;

    private static final double MAX_FEED_TIME = 0.90;

    private static final double RPM_TOLERANCE = 25.0;


    // =========================================================
    // LIMELIGHT AUTO AIM
    // =========================================================

    private static final double AIM_KP = 0.0152;

    private static final double AIM_KD = 0.0016;

    private static final double ANGLE_TOLERANCE = 0.4;

    private static final double MAX_TURN_POWER = 0.45;


    // =========================================================
    // SENSOR
    // =========================================================

    private static final double SENSOR_DEBOUNCE_MS = 25;


    // =========================================================
    // START POSE
    // =========================================================

    private final Pose startPose =
            new Pose(
                    55.559,
                    9.322,
                    Math.toRadians(90)
            );


    // =========================================================
    // BUILD PATHS
    // =========================================================

    private void buildPaths() {


        // =====================================================
        // PATH 1
        //
        // (55.559, 9.322)
        // ->
        // (56.220, 21.233)
        //
        // Heading:
        // 90° -> 110°
        // =====================================================

        path1 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                55.559,
                                                9.322
                                        ),

                                        new Pose(
                                                56.220,
                                                21.233
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(90),
                                Math.toRadians(110)
                        )

                        .build();


        // =====================================================
        // PATH 2
        //
        // (56.220, 21.233)
        // ->
        // (57.144, 32.225)
        //
        // Tangent Heading
        // =====================================================

        path2 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                56.220,
                                                21.233
                                        ),

                                        new Pose(
                                                57.144,
                                                32.225
                                        )
                                )
                        )

                        .setTangentHeadingInterpolation()

                        .build();
    }


    // =========================================================
    // FOLLOW PATH
    // =========================================================

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


            follower.update();


            // Shooter apagado mientras sale
            shooterCmd.autonomousIdle();


            // Intake apagado mientras solo nos posicionamos
            intake.stop();


            telemetry.addLine(
                    "MOVIENDOSE A POSICION FAR BLUE"
            );


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


            telemetry.update();


            idle();
        }


        shooterCmd.autonomousIdle();

        intake.stop();
    }


    // =========================================================
    // AUTO AIM + SHOOT
    // =========================================================

    private void shootThreeBalls() {


        // =====================================================
        // TIMERS
        // =====================================================

        ElapsedTime readyTimer =
                new ElapsedTime();


        ElapsedTime stableTimer =
                new ElapsedTime();


        ElapsedTime capuchaTimer =
                new ElapsedTime();


        readyTimer.reset();

        stableTimer.reset();

        capuchaTimer.reset();


        double previousCapuchaPosition =
                shooterCmd.getCapuchaPosition();


        double lastAimError =
                0;


        double lastAimTime =
                0;


        boolean wasReady =
                false;


        boolean conditionsStable =
                false;


        // =====================================================
        // PREPARAR DISPARO
        //
        // LIMELIGHT
        // RPM
        // CAPUCHA
        // =====================================================

        while (
                opModeIsActive()
                        &&
                        !conditionsStable
                        &&
                        readyTimer.seconds()
                                <
                                MAX_READY_TIME
        ) {


            // =================================================
            // SHOOTER USA LIMELIGHT + TABLA
            // =================================================

            shooterCmd.autonomousShoot();


            // Mantener intake quieto
            intake.stop();


            // =================================================
            // CAPUCHA
            // =================================================

            double currentCapuchaPosition =
                    shooterCmd.getCapuchaPosition();


            if (
                    Math.abs(
                            currentCapuchaPosition
                                    -
                                    previousCapuchaPosition
                    )
                            >
                            CAPUCHA_POSITION_EPSILON
            ) {


                capuchaTimer.reset();


                previousCapuchaPosition =
                        currentCapuchaPosition;
            }


            boolean capuchaReady =
                    capuchaTimer.seconds()
                            >=
                            CAPUCHA_SETTLE_TIME;


            // =================================================
            // RPM
            // =================================================

            double targetVelocity =
                    shooterCmd
                            .getLastAutoVelocity();


            double realVelocity =
                    shooter
                            .getAvgVelocity();


            double velocityError =
                    Math.abs(
                            targetVelocity
                                    -
                                    realVelocity
                    );


            boolean rpmReady =
                    velocityError
                            <=
                            RPM_TOLERANCE;


            // =================================================
            // LIMELIGHT AUTO AIM
            // =================================================

            LLResult result =
                    limelight
                            .getLatestResult();


            boolean aimReady =
                    false;


            double tx =
                    0;


            double turn =
                    0;


            if (
                    result != null
                            &&
                            result.isValid()
            ) {


                tx =
                        result.getTx();


                double aimError =
                        tx;


                double currentTime =
                        readyTimer.seconds();


                double dt =
                        currentTime
                                -
                                lastAimTime;


                if (
                        dt <= 0
                ) {


                    dt =
                            0.001;
                }


                // =============================================
                // YA ESTÁ ALINEADO
                // =============================================

                if (
                        Math.abs(
                                aimError
                        )
                                <=
                                ANGLE_TOLERANCE
                ) {


                    aimReady =
                            true;


                    turn =
                            0;

                } else {


                    double pTerm =
                            aimError
                                    *
                                    AIM_KP;


                    double dTerm =
                            (
                                    (
                                            aimError
                                                    -
                                                    lastAimError
                                    )
                                            /
                                            dt
                            )
                                    *
                                    AIM_KD;


                    turn =
                            Range.clip(
                                    pTerm
                                            +
                                            dTerm,

                                    -MAX_TURN_POWER,

                                    MAX_TURN_POWER
                            );
                }


                lastAimError =
                        aimError;


                lastAimTime =
                        currentTime;

            } else {


                aimReady =
                        false;


                turn =
                        0;


                lastAimError =
                        0;


                lastAimTime =
                        readyTimer.seconds();
            }


            // =================================================
            // SOLO GIRAR PARA APUNTAR
            // =================================================

            drive.drive(
                    0,
                    0,
                    turn
            );


            // =================================================
            // VALIDACIONES
            // =================================================

            boolean readyNow =
                    rpmReady
                            &&
                            aimReady
                            &&
                            capuchaReady;


            if (
                    readyNow
            ) {


                if (
                        !wasReady
                ) {


                    stableTimer.reset();
                }


                if (
                        stableTimer.seconds()
                                >=
                                READY_STABLE_TIME
                ) {


                    conditionsStable =
                            true;
                }

            } else {


                stableTimer.reset();
            }


            wasReady =
                    readyNow;


            // =================================================
            // TELEMETRY
            // =================================================

            telemetry.addLine(
                    "AUTO AJUSTANDO FAR BLUE"
            );


            telemetry.addData(
                    "TX",
                    tx
            );


            telemetry.addData(
                    "Aim Ready",
                    aimReady
            );


            telemetry.addData(
                    "Distancia",
                    shooterCmd.getLastDistance()
            );


            telemetry.addData(
                    "Target RPM",
                    targetVelocity
            );


            telemetry.addData(
                    "Real RPM",
                    realVelocity
            );


            telemetry.addData(
                    "RPM Ready",
                    rpmReady
            );


            telemetry.addData(
                    "Capucha",
                    shooterCmd.getCapuchaAngle()
            );


            telemetry.addData(
                    "Capucha Ready",
                    capuchaReady
            );


            telemetry.addData(
                    "TODO READY",
                    conditionsStable
            );


            telemetry.update();


            idle();
        }


        // =====================================================
        // DETENER CHASIS
        // =====================================================

        drive.drive(
                0,
                0,
                0
        );


        // =====================================================
        // DISPARAR 3 PELOTAS
        // =====================================================

        ElapsedTime feedTimer =
                new ElapsedTime();


        ElapsedTime debounceTimer =
                new ElapsedTime();


        feedTimer.reset();

        debounceTimer.reset();


        int ballsPassed =
                0;


        boolean previousDetected =
                intake.hasPiece();


        while (
                opModeIsActive()
                        &&
                        ballsPassed < 3
                        &&
                        feedTimer.seconds()
                                <
                                MAX_FEED_TIME
        ) {


            // Mantener shooter ajustado
            shooterCmd.autonomousShoot();


            // =================================================
            // INTAKE + INDEXER AL SHOOTER
            // =================================================

            intake.set(
                    1.0,
                    -1.0
            );


            // =================================================
            // SENSOR
            // =================================================

            boolean detected =
                    intake.hasPiece();


            if (
                    previousDetected
                            &&
                            !detected
                            &&
                            debounceTimer.milliseconds()
                                    >
                                    SENSOR_DEBOUNCE_MS
            ) {


                ballsPassed++;


                debounceTimer.reset();
            }


            previousDetected =
                    detected;


            telemetry.addLine(
                    "DISPARANDO FAR BLUE"
            );


            telemetry.addData(
                    "Pelotas",
                    "%d / 3",
                    ballsPassed
            );


            telemetry.addData(
                    "Target",
                    shooterCmd.getLastAutoVelocity()
            );


            telemetry.addData(
                    "Real",
                    shooter.getAvgVelocity()
            );


            telemetry.addData(
                    "Sensor",
                    intake.getDistance()
            );


            telemetry.addData(
                    "Feed Time",
                    "%.3f",
                    feedTimer.seconds()
            );


            telemetry.update();


            idle();
        }


        // =====================================================
        // TERMINÓ DISPARO
        // =====================================================

        drive.drive(
                0,
                0,
                0
        );


        intake.stop();


        shooterCmd.autonomousIdle();
    }


    // =========================================================
    // RUN
    // =========================================================

    @Override
    public void runOpMode() {


        // =====================================================
        // PEDRO
        // =====================================================

        follower =
                Constants.createFollower(
                        hardwareMap
                );


        // =====================================================
        // DRIVE
        // =====================================================

        drive =
                new DriveSubsystem();


        drive.init(
                hardwareMap
        );


        // =====================================================
        // SHOOTER
        // =====================================================

        shooter =
                new ShooterSubsystem();


        shooter.init(
                hardwareMap
        );


        // =====================================================
        // INTAKE
        // =====================================================

        intake =
                new IntakeSubsystem();


        intake.init(
                hardwareMap
        );


        // =====================================================
        // LIMELIGHT
        // =====================================================

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );


        limelight.pipelineSwitch(
                0
        );


        limelight.start();


        // =====================================================
        // SHOOTER CMD
        // =====================================================

        shooterCmd =
                new ShooterCmd(
                        shooter,
                        limelight,
                        gamepad1,
                        hardwareMap
                );


        // =====================================================
        // PATHS
        // =====================================================

        buildPaths();


        // =====================================================
        // START POSE
        // =====================================================

        follower.setStartingPose(
                startPose
        );


        // =====================================================
        // INIT TELEMETRY
        // =====================================================

        telemetry.addLine(
                "AUTO FAR BLUE LISTO"
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
                "Start Heading",
                Math.toDegrees(
                        startPose.getHeading()
                )
        );


        telemetry.update();


        // =====================================================
        // START
        // =====================================================

        waitForStart();


        if (
                isStopRequested()
        ) {


            return;
        }


        // =====================================================
        // ASEGURAR TODO APAGADO
        // =====================================================

        shooterCmd.autonomousIdle();

        intake.stop();


        // =====================================================
        // SALIR
        // =====================================================

        followPath(
                path1
        );


        followPath(
                path2
        );


        // =====================================================
        // AUTO AIM + RPM + DISPARAR
        // =====================================================

        shootThreeBalls();


        // =====================================================
        // FIN
        // =====================================================

        shooterCmd.autonomousIdle();

        intake.stop();


        while (
                opModeIsActive()
        ) {


            telemetry.addLine(
                    "AUTO FAR BLUE TERMINADO"
            );


            telemetry.addData(
                    "X Final",
                    follower.getPose().getX()
            );


            telemetry.addData(
                    "Y Final",
                    follower.getPose().getY()
            );


            telemetry.addData(
                    "Heading Final",
                    Math.toDegrees(
                            follower
                                    .getPose()
                                    .getHeading()
                    )
            );


            telemetry.update();


            idle();
        }
    }
}