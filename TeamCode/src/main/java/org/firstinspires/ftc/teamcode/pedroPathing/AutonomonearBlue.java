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
        name = "AutonomoNearBlue",
        group = "Pedro"
)
public class AutonomonearBlue extends LinearOpMode {

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

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    private ShooterCmd shooterCmd;

    private Limelight3A limelight;


    // =========================================================
    // TIEMPOS
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
    // CORRECCIÓN SHOOT POSE
    // =========================================================

    private static final double SHOOT_POSITION_TOLERANCE = 0.75;

    private static final double SHOOT_HEADING_TOLERANCE = 2.0;


    // =========================================================
    // START POSE BLUE
    // =========================================================

    private final Pose startPose =
            new Pose(
                    24.702,
                    122.170,
                    Math.toRadians(140)
            );


    // =========================================================
    // SHOOT POSE BLUE
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


        // PATH 1
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


        // PATH 2
        path2 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.490,
                                                104.987
                                        ),
                                        new Pose(
                                                50.081,
                                                82.539
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(140),
                                Math.toRadians(0)
                        )

                        .build();


        // PATH 3
        path3 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.081,
                                                82.539
                                        ),
                                        new Pose(
                                                23.298,
                                                82.457
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();


        // PATH 4
        path4 =
                follower.pathBuilder()

                        .addPath(
                                new BezierCurve(
                                        new Pose(
                                                23.298,
                                                82.457
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


        // PATH 5
        path5 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.490,
                                                104.987
                                        ),
                                        new Pose(
                                                49.604,
                                                60.246
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(140),
                                Math.toRadians(0)
                        )

                        .build();


        // PATH 6
        path6 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                49.604,
                                                60.246
                                        ),
                                        new Pose(
                                                24.539,
                                                60.267
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();


        // PATH 7
        path7 =
                follower.pathBuilder()

                        .addPath(
                                new BezierCurve(
                                        new Pose(
                                                24.539,
                                                60.267
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


        // PATH 8
        path8 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                50.490,
                                                104.987
                                        ),
                                        new Pose(
                                                48.792,
                                                34.835
                                        )
                                )
                        )

                        .setLinearHeadingInterpolation(
                                Math.toRadians(140),
                                Math.toRadians(0)
                        )

                        .build();


        // PATH 9
        path9 =
                follower.pathBuilder()

                        .addPath(
                                new BezierLine(
                                        new Pose(
                                                48.792,
                                                34.835
                                        ),
                                        new Pose(
                                                20.101,
                                                34.959
                                        )
                                )
                        )

                        .setConstantHeadingInterpolation(
                                Math.toRadians(0)
                        )

                        .build();


        // PATH 10
        path10 =
                follower.pathBuilder()

                        .addPath(
                                new BezierCurve(
                                        new Pose(
                                                20.101,
                                                34.959
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

        intake.intake1.setPower(1.0);

        if (intake.hasPiece()) {

            intake.intake2.setPower(0);

        } else {

            intake.intake2.setPower(-0.50);
        }
    }


    // =========================================================
    // WAIT
    // =========================================================

    private void waitSeconds(double seconds) {

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

            idle();
        }
    }


    // =========================================================
    // FOLLOW PATH
    // =========================================================

    private void followPath(PathChain path) {

        follower.followPath(path);

        while (
                opModeIsActive()
                        &&
                        follower.isBusy()
        ) {

            follower.update();

            shooterCmd.autonomousIdle();

            updateIntakeAutomatic();

            telemetry.addLine(
                    "SIGUIENDO PATH BLUE"
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
                            follower.getPose().getHeading()
                    )
            );

            telemetry.addData(
                    "Sensor",
                    intake.getDistance()
            );

            telemetry.addData(
                    "Tiene Pelota",
                    intake.hasPiece()
            );

            telemetry.update();

            idle();
        }

        shooterCmd.autonomousIdle();

        updateIntakeAutomatic();
    }


    // =========================================================
    // DISTANCIA SHOOT
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
    // ERROR HEADING
    // =========================================================

    private double getShootHeadingError() {

        double currentHeading =
                follower.getPose().getHeading();

        double targetHeading =
                shootPose.getHeading();

        double error =
                Math.toDegrees(
                        targetHeading
                                -
                                currentHeading
                );

        while (error > 180) {
            error -= 360;
        }

        while (error < -180) {
            error += 360;
        }

        return Math.abs(error);
    }


    // =========================================================
    // CORREGIR SHOOT POSE
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
                    "CORRIGIENDO SHOOT BLUE"
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


        double lastAimError = 0;

        double lastAimTime = 0;


        boolean wasReady = false;

        boolean conditionsStable = false;


        // =====================================================
        // PREPARAR DISPARO
        // =====================================================

        while (
                opModeIsActive()
                        &&
                        !conditionsStable
                        &&
                        readyTimer.seconds() < MAX_READY_TIME
        ) {

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


            boolean rpmReady =
                    Math.abs(
                            targetVelocity
                                    -
                                    realVelocity
                    )
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


            drive.drive(
                    0,
                    0,
                    turn
            );


            boolean readyNow =
                    rpmReady
                            &&
                            aimReady
                            &&
                            capuchaReady;


            if (readyNow) {

                if (!wasReady) {

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


            telemetry.addLine(
                    "PREPARANDO DISPARO BLUE"
            );


            telemetry.addData(
                    "TX",
                    tx
            );


            telemetry.addData(
                    "Distancia",
                    shooterCmd.getLastDistance()
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
                    "Aim Ready",
                    aimReady
            );


            telemetry.addData(
                    "RPM Ready",
                    rpmReady
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
        // FEED
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
                        feedTimer.seconds() < MAX_FEED_TIME
        ) {

            shooterCmd.autonomousShoot();


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
                    "DISPARANDO BLUE"
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


            telemetry.addData(
                    "Sensor",
                    intake.getDistance()
            );


            telemetry.update();

            idle();
        }


        drive.drive(
                0,
                0,
                0
        );


        updateIntakeAutomatic();


        shooterCmd.autonomousIdle();
    }


    // =========================================================
    // RUN OPMODE
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
                "AUTONOMO NEAR BLUE LISTO"
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
        // Y ≈ 82
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


        waitSeconds(
                PATH_WAIT
        );


        followPath(
                path4
        );


        // =====================================================
        // DISPARO 2
        // =====================================================

        shootThreeBalls();


        // =====================================================
        // RECOLECCIÓN 2
        // Y ≈ 60
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


        // =====================================================
        // DISPARO 3
        // =====================================================

        shootThreeBalls();


        // =====================================================
        // RECOLECCIÓN 3
        // Y ≈ 35
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


        // =====================================================
        // DISPARO 4
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
                    "AUTONOMO BLUE TERMINADO"
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
                            follower.getPose().getHeading()
                    )
            );


            telemetry.update();

            idle();
        }
    }
}


