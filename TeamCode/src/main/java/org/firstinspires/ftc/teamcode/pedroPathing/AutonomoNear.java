package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
        name = "AutonomoNear",
        group = "Pedro"
)
public class AutonomoNear extends LinearOpMode {


    // =========================================================
    // PEDRO
    // =========================================================

    private Follower follower;

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;
    private PathChain path8;
    private PathChain path9;
    private PathChain path10;


    // =========================================================
    // SUBSYSTEMS
    // =========================================================

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;


    // =========================================================
    // COMMANDS
    // =========================================================

    private ShooterCmd shooterCmd;


    // =========================================================
    // LIMELIGHT
    // =========================================================

    private Limelight3A limelight;


    // =========================================================
    // TIEMPOS RÁPIDOS
    // =========================================================

    private static final double PATH_WAIT = 0.01;

    private static final double MAX_READY_TIME = 1.20;

    private static final double READY_STABLE_TIME = 0.005;

    private static final double CAPUCHA_SETTLE_TIME = 0.03;

    private static final double CAPUCHA_POSITION_EPSILON = 0.001;

    private static final double MAX_FEED_TIME = 0.75;


    // =========================================================
    // SHOOTER
    // =========================================================

    private static final double RPM_TOLERANCE = 25.0;


    // =========================================================
    // AUTO AIM
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
    // CORRECCIÓN FINAL
    // =========================================================

    private static final double SHOOT_POSITION_TOLERANCE = 0.75;

    private static final double SHOOT_HEADING_TOLERANCE = 2.0;


    // =========================================================
    // START POSE
    // =========================================================

    private final Pose startPose =
            new Pose(
                    24.702,
                    122.170,
                    Math.toRadians(140)
            );


    // =========================================================
    // SHOOT POSE
    // =========================================================

    private final Pose shootPose =
            new Pose(
                    50.490,
                    104.987,
                    Math.toRadians(140)
            );


    // =========================================================
    // BUILD PATHS
    // =========================================================

    private void buildPaths() {


        // =====================================================
        // PATH 1
        // =====================================================

        path1 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                24.702,
                                                122.170
                                        ),
                                        new Pose(
                                                50.490,
                                                104.987
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(140)
                        )

                        .build();


        // =====================================================
        // PATH 2
        // =====================================================

        path2 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.490,
                                                104.987
                                        ),
                                        new Pose(
                                                51.844,
                                                88.270
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(140),
                                Math.toRadians(0)
                        )

                        .build();


        // =====================================================
        // PATH 3
        // NUEVO PUNTO
        // =====================================================

        path3 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                51.844,
                                                88.270
                                        ),
                                        new Pose(
                                                14.482,
                                                87.747
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();


        // =====================================================
        // PATH 4
        // REGRESA AL MISMO SHOOT POSE
        // =====================================================

        path4 =
                follower.pathBuilder()

                        .addPath(
                                new BezierCurve(
                                        new Pose(
                                                14.482,
                                                87.747
                                        ),
                                        new Pose(
                                                17.335,
                                                113.465
                                        ),
                                        new Pose(
                                                50.490,
                                                104.987
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(0),
                                Math.toRadians(140)
                        )

                        .build();


        // =====================================================
        // PATH 5
        // =====================================================

        path5 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.490,
                                                104.987
                                        ),
                                        new Pose(
                                                50.265,
                                                68.181
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(140),
                                Math.toRadians(0)
                        )

                        .build();


        // =====================================================
        // PATH 6
        // =====================================================

        path6 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.265,
                                                68.181
                                        ),
                                        new Pose(
                                                16.164,
                                                68.202
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();


        // =====================================================
        // PATH 7
        // REGRESA AL MISMO SHOOT POSE
        // =====================================================

        path7 =
                follower.pathBuilder()

                        .addPath(
                                new BezierCurve(
                                        new Pose(
                                                16.164,
                                                68.202
                                        ),
                                        new Pose(
                                                19.765,
                                                94.742
                                        ),
                                        new Pose(
                                                50.490,
                                                104.987
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(0),
                                Math.toRadians(140)
                        )

                        .build();


        // =====================================================
        // PATH 8
        // =====================================================

        path8 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.490,
                                                104.987
                                        ),
                                        new Pose(
                                                49.453,
                                                46.737
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(140),
                                Math.toRadians(0)
                        )

                        .build();


        // =====================================================
        // PATH 9
        // NUEVO PUNTO
        // =====================================================

        path9 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                49.453,
                                                46.737
                                        ),
                                        new Pose(
                                                13.930,
                                                46.200
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();


        // =====================================================
        // PATH 10
        // REGRESA AL MISMO SHOOT POSE
        // =====================================================

        path10 =
                follower.pathBuilder()

                        .addPath(
                                new BezierCurve(
                                        new Pose(
                                                13.930,
                                                46.200
                                        ),
                                        new Pose(
                                                27.697,
                                                88.916
                                        ),
                                        new Pose(
                                                50.490,
                                                104.987
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(0),
                                Math.toRadians(140)
                        )

                        .build();
    }


    // =========================================================
    // INTAKE AUTOMÁTICO
    // =========================================================

    private void updateIntakeAutomatic() {

        intake.intake1.setPower(
                1.0
        );


        if (
                intake.hasPiece()
        ) {

            intake.intake2.setPower(
                    0
            );

        } else {

            intake.intake2.setPower(
                    -0.50
            );
        }
    }


    // =========================================================
    // WAIT
    // =========================================================

    private void waitSeconds(
            double seconds
    ) {

        ElapsedTime timer =
                new ElapsedTime();


        timer.reset();


        while (
                opModeIsActive()
                        &&
                        timer.seconds() < seconds
        ) {

            shooterCmd.autonomousIdle();

            updateIntakeAutomatic();


            telemetry.addData(
                    "Wait",
                    "%.3f / %.3f",
                    timer.seconds(),
                    seconds
            );


            telemetry.update();

            idle();
        }
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


            // Shooter NO se prende antes
            shooterCmd.autonomousIdle();


            updateIntakeAutomatic();


            telemetry.addLine(
                    "SIGUIENDO PATH"
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


            telemetry.addData(
                    "Sensor",
                    intake.getDistance()
            );


            telemetry.update();


            idle();
        }


        updateIntakeAutomatic();

        shooterCmd.autonomousIdle();
    }


    // =========================================================
    // DISTANCIA AL SHOOT POSE
    // =========================================================

    private double getDistanceToShootPose() {

        Pose current =
                follower.getPose();


        double dx =
                shootPose.getX()
                        -
                        current.getX();


        double dy =
                shootPose.getY()
                        -
                        current.getY();


        return Math.sqrt(
                dx * dx
                        +
                        dy * dy
        );
    }


    // =========================================================
    // ERROR DE HEADING
    // =========================================================

    private double getShootHeadingError() {

        double currentHeading =
                follower
                        .getPose()
                        .getHeading();


        double targetHeading =
                shootPose
                        .getHeading();


        double error =
                Math.toDegrees(
                        targetHeading
                                -
                                currentHeading
                );


        while (
                error > 180
        ) {

            error -= 360;
        }


        while (
                error < -180
        ) {

            error += 360;
        }


        return Math.abs(
                error
        );
    }


    // =========================================================
    // CORRECCIÓN FINAL
    // =========================================================

    private void correctToShootPose() {

        double positionError =
                getDistanceToShootPose();


        double headingError =
                getShootHeadingError();


        if (
                positionError <= SHOOT_POSITION_TOLERANCE
                        &&
                        headingError <= SHOOT_HEADING_TOLERANCE
        ) {

            return;
        }


        Pose current =
                follower.getPose();


        PathChain correction =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        current,
                                        shootPose
                                )
                        )

                        .setLinearHeadingInterpolation(
                                current.getHeading(),
                                shootPose.getHeading()
                        )

                        .build();


        follower.followPath(
                correction
        );


        while (
                opModeIsActive()
                        &&
                        follower.isBusy()
        ) {

            follower.update();


            shooterCmd.autonomousShoot();


            updateIntakeAutomatic();


            telemetry.addLine(
                    "CORRIGIENDO SHOOT POSE"
            );


            telemetry.addData(
                    "Error Posicion",
                    getDistanceToShootPose()
            );


            telemetry.addData(
                    "Error Heading",
                    getShootHeadingError()
            );


            telemetry.update();


            idle();
        }
    }


    // =========================================================
    // SHOOT 3 BALLS
    // =========================================================

    private void shootThreeBalls() {

        correctToShootPose();


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

            // Aquí sí prender shooter
            shooterCmd.autonomousShoot();


            updateIntakeAutomatic();


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
                    shooterCmd.getLastAutoVelocity();


            double realVelocity =
                    shooter.getAvgVelocity();


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
            // LIMELIGHT
            // =================================================

            LLResult result =
                    limelight.getLatestResult();


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

                    dt = 0.001;
                }


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
                                    pTerm + dTerm,
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
            // SOLO ROTACIÓN
            // =================================================

            drive.drive(
                    0,
                    0,
                    turn
            );


            // =================================================
            // VALIDACIÓN RÁPIDA
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
                    "PREPARANDO DISPARO"
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
                    "Target",
                    targetVelocity
            );


            telemetry.addData(
                    "Real",
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
        // STOP DRIVE
        // =====================================================

        drive.drive(
                0,
                0,
                0
        );


        // =====================================================
        // FEED RÁPIDO
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

            shooterCmd.autonomousShoot();


            // Intake + indexer máximo
            intake.set(
                    1.0,
                    -1.0
            );


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
                    "DISPARANDO"
            );


            telemetry.addData(
                    "Pelotas",
                    "%d / 3",
                    ballsPassed
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
        // FIN SHOOT
        // =====================================================

        drive.drive(
                0,
                0,
                0
        );


        updateIntakeAutomatic();


        shooterCmd.autonomousIdle();
    }


    // =========================================================
    // RUN
    // =========================================================

    @Override
    public void runOpMode() {


        follower =
                Constants.createFollower(
                        hardwareMap
                );


        drive =
                new DriveSubsystem();


        drive.init(
                hardwareMap
        );


        shooter =
                new ShooterSubsystem();


        shooter.init(
                hardwareMap
        );


        intake =
                new IntakeSubsystem();


        intake.init(
                hardwareMap
        );


        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );


        limelight.pipelineSwitch(
                0
        );


        limelight.start();


        shooterCmd =
                new ShooterCmd(
                        shooter,
                        limelight,
                        gamepad1,
                        hardwareMap
                );


        buildPaths();


        follower.setStartingPose(
                startPose
        );


        telemetry.addLine(
                "AUTONOMO NEAR LISTO"
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
                "Shoot X",
                shootPose.getX()
        );


        telemetry.addData(
                "Shoot Y",
                shootPose.getY()
        );


        telemetry.update();


        waitForStart();


        if (
                isStopRequested()
        ) {

            return;
        }


        shooterCmd.autonomousIdle();


        updateIntakeAutomatic();


        // =====================================================
        // DISPARO 1
        // =====================================================

        followPath(
                path1
        );


        shootThreeBalls();


        // =====================================================
        // RECOLECCIÓN 1
        // =====================================================

        followPath(
                path2
        );


        waitSeconds(
                PATH_WAIT
        );


        followPath(
                path3
        );


        followPath(
                path4
        );


        shootThreeBalls();


        // =====================================================
        // RECOLECCIÓN 2
        // =====================================================

        followPath(
                path5
        );


        waitSeconds(
                PATH_WAIT
        );


        followPath(
                path6
        );


        waitSeconds(
                PATH_WAIT
        );


        followPath(
                path7
        );


        shootThreeBalls();


        // =====================================================
        // RECOLECCIÓN 3
        // =====================================================

        followPath(
                path8
        );


        waitSeconds(
                PATH_WAIT
        );


        followPath(
                path9
        );


        waitSeconds(
                PATH_WAIT
        );


        followPath(
                path10
        );


        shootThreeBalls();


        // =====================================================
        // FIN
        // =====================================================

        shooterCmd.autonomousIdle();


        updateIntakeAutomatic();


        while (
                opModeIsActive()
        ) {

            telemetry.addLine(
                    "AUTONOMO TERMINADO"
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