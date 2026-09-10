package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeCmd;


@TeleOp(
        name = "TeleopLimeDistanciapROBAR1",
        group = "TeleOp"
)
public class TeleopLimeDistancia extends LinearOpMode {


    // =====================================================
    // SHOOTER
    // =====================================================

    private ShooterSubsystem shooter;


    // Velocidad inicial
    private double shooterTargetVelocity = 200;


    // Cuánto aumenta/disminuye por click
    private final double VELOCITY_STEP = 5;


    // Límites
    private final double SHOOTER_MAX_VELOCITY = 6500;
    private final double SHOOTER_MIN_VELOCITY = 0;


    // Detectar un solo click
    private boolean lastUp = false;
    private boolean lastDown = false;



    // =====================================================
    // CAPUCHA
    // =====================================================

    private Servo capucha;


    // Posición inicial
    private final double CAPUCHA_BASE = 0.30;


    /*
     * Aproximadamente 40 grados
     * para servo de 270°
     */
    private final double CAPUCHA_STEP =
            40.0 / 270.0;


    /*
     * 0 = fase 1
     * 1 = fase 2
     * 2 = fase 3
     * 3 = fase 4
     */
    private int capuchaFase = 0;


    private double capuchaPosition =
            CAPUCHA_BASE;


    private boolean lastCapuchaRight = false;
    private boolean lastCapuchaLeft = false;



    // =====================================================
    // INTAKE SUBSYSTEM + COMMAND
    // =====================================================

    private IntakeSubsystem intake;

    private IntakeCmd intakeCmd;



    // =====================================================
    // LIMELIGHT + IMU
    // =====================================================

    private Limelight3A limelight;

    private IMU imu;



    // =====================================================
    // RUN OPMODE
    // =====================================================

    @Override
    public void runOpMode()
            throws InterruptedException {


        // =====================================================
        // INIT SHOOTER
        // =====================================================

        shooter =
                new ShooterSubsystem();


        shooter.init(
                hardwareMap
        );


        // =====================================================
        // INIT CAPUCHA
        // =====================================================

        capucha =
                hardwareMap.get(
                        Servo.class,
                        "cap"
                );


        // =====================================================
        // POSICIÓN INICIAL CAPUCHA
        // =====================================================

        capuchaFase = 0;

        capuchaPosition =
                CAPUCHA_BASE;


        capucha.setPosition(
                capuchaPosition
        );


        // =====================================================
        // INIT INTAKE SUBSYSTEM
        // =====================================================

        intake =
                new IntakeSubsystem();


        intake.init(
                hardwareMap
        );


        // =====================================================
        // INIT INTAKE COMMAND
        //
        // Usamos gamepad2
        // =====================================================

        intakeCmd =
                new IntakeCmd(
                        intake,
                        gamepad2
                );


        // =====================================================
        // INIT LIMELIGHT
        // =====================================================

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );


        limelight.pipelineSwitch(
                0
        );


        // =====================================================
        // INIT IMU
        // =====================================================

        imu =
                hardwareMap.get(
                        IMU.class,
                        "imu"
                );


        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(

                        RevHubOrientationOnRobot
                                .LogoFacingDirection
                                .RIGHT,

                        RevHubOrientationOnRobot
                                .UsbFacingDirection
                                .UP
                );


        imu.initialize(
                new IMU.Parameters(
                        orientationOnRobot
                )
        );


        // =====================================================
        // TELEMETRY ANTES DE START
        // =====================================================

        telemetry.addLine(
                "ROBOT INICIALIZADO"
        );


        telemetry.addLine("");


        telemetry.addData(
                "Shooter inicial",
                shooterTargetVelocity
        );


        telemetry.addData(
                "Capucha fase",
                capuchaFase + 1
        );


        telemetry.addData(
                "Capucha posicion",
                capuchaPosition
        );


        telemetry.addLine("");


        telemetry.addLine(
                "D-PAD RIGHT = Capucha +1 fase"
        );


        telemetry.addLine(
                "D-PAD LEFT = Capucha -1 fase"
        );


        telemetry.addLine(
                "D-PAD UP = Shooter +5"
        );


        telemetry.addLine(
                "D-PAD DOWN = Shooter -5"
        );


        telemetry.addLine("");


        telemetry.addLine(
                "X = Intake automatico"
        );


        telemetry.addLine(
                "B = Intake reversa"
        );


        telemetry.addLine(
                "Y = Intake manual adelante"
        );


        telemetry.addLine("");


        telemetry.addLine(
                "Esperando START..."
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
        // START LIMELIGHT
        // =====================================================

        limelight.start();



        // =====================================================
        // LOOP PRINCIPAL
        // =====================================================

        while (
                opModeIsActive()
        ) {


            // =================================================
            // SHOOTER +
            // =================================================

            if (
                    gamepad2.dpad_up
                            &&
                            !lastUp
            ) {


                shooterTargetVelocity +=
                        VELOCITY_STEP;


                if (
                        shooterTargetVelocity
                                >
                                SHOOTER_MAX_VELOCITY
                ) {


                    shooterTargetVelocity =
                            SHOOTER_MAX_VELOCITY;
                }
            }


            // =================================================
            // SHOOTER -
            // =================================================

            if (
                    gamepad2.dpad_down
                            &&
                            !lastDown
            ) {


                shooterTargetVelocity -=
                        VELOCITY_STEP;


                if (
                        shooterTargetVelocity
                                <
                                SHOOTER_MIN_VELOCITY
                ) {


                    shooterTargetVelocity =
                            SHOOTER_MIN_VELOCITY;
                }
            }


            lastUp =
                    gamepad2.dpad_up;


            lastDown =
                    gamepad2.dpad_down;



            // =================================================
            // ACTIVAR SHOOTER
            // =================================================

            if (
                    gamepad2.right_trigger > 0.1
            ) {


                shooter.setVelocity(
                        shooterTargetVelocity
                );

            } else {


                shooter.stop();
            }



            // =================================================
            // CAPUCHA +
            // =================================================

            if (
                    gamepad2.dpad_right
                            &&
                            !lastCapuchaRight
            ) {


                if (
                        capuchaFase < 3
                ) {


                    capuchaFase++;


                    capuchaPosition =
                            CAPUCHA_BASE
                                    +
                                    (
                                            capuchaFase
                                                    *
                                                    CAPUCHA_STEP
                                    );


                    if (
                            capuchaPosition > 1.0
                    ) {


                        capuchaPosition =
                                1.0;
                    }


                    capucha.setPosition(
                            capuchaPosition
                    );
                }
            }



            // =================================================
            // CAPUCHA -
            // =================================================

            if (
                    gamepad2.dpad_left
                            &&
                            !lastCapuchaLeft
            ) {


                if (
                        capuchaFase > 0
                ) {


                    capuchaFase--;


                    capuchaPosition =
                            CAPUCHA_BASE
                                    +
                                    (
                                            capuchaFase
                                                    *
                                                    CAPUCHA_STEP
                                    );


                    if (
                            capuchaPosition < 0.0
                    ) {


                        capuchaPosition =
                                0.0;
                    }


                    capucha.setPosition(
                            capuchaPosition
                    );
                }
            }


            lastCapuchaRight =
                    gamepad2.dpad_right;


            lastCapuchaLeft =
                    gamepad2.dpad_left;



            // =================================================
            // INTAKE
            //
            // TODO EL CONTROL AHORA VIENE
            // DE IntakeCmd
            // =================================================

            intakeCmd.execute();



            // =================================================
            // LIMELIGHT + IMU
            // =================================================

            YawPitchRollAngles robotOrientation =
                    imu
                            .getRobotYawPitchRollAngles();


            limelight.updateRobotOrientation(
                    robotOrientation.getYaw()
            );


            // =================================================
            // LEER LIMELIGHT
            // =================================================

            LLResult llResult =
                    limelight
                            .getLatestResult();


            if (
                    llResult != null
                            &&
                            llResult.isValid()
            ) {


                Pose3D botPose =
                        llResult
                                .getBotpose();


                Position posePos =
                        botPose
                                .getPosition()
                                .toUnit(
                                        DistanceUnit.INCH
                                );


                telemetry.addLine(
                        "=== APRILTAG ==="
                );


                telemetry.addData(
                        "Posicion",
                        posePos
                );


                telemetry.addData(
                        "Tx",
                        llResult.getTx()
                );


                telemetry.addData(
                        "Ty",
                        llResult.getTy()
                );


                telemetry.addData(
                        "Ta",
                        llResult.getTa()
                );

            } else {


                telemetry.addLine(
                        "=== APRILTAG ==="
                );


                telemetry.addLine(
                        "No Tag Detectado"
                );
            }



            // =================================================
            // VELOCIDAD SHOOTER
            // =================================================

            double leftVelocity =
                    shooter
                            .left
                            .getVelocity();


            double rightVelocity =
                    shooter
                            .right
                            .getVelocity();


            double averageVelocity =
                    shooter
                            .getAvgVelocity();


            double error =
                    shooterTargetVelocity
                            -
                            averageVelocity;



            // =================================================
            // TELEMETRY SHOOTER
            // =================================================

            telemetry.addLine("");


            telemetry.addLine(
                    "=== SHOOTER ==="
            );


            telemetry.addData(
                    "Target",
                    shooterTargetVelocity
            );


            telemetry.addData(
                    "Left",
                    leftVelocity
            );


            telemetry.addData(
                    "Right",
                    rightVelocity
            );


            telemetry.addData(
                    "Promedio",
                    averageVelocity
            );


            telemetry.addData(
                    "Error",
                    error
            );



            // =================================================
            // TELEMETRY CAPUCHA
            // =================================================

            telemetry.addLine("");


            telemetry.addLine(
                    "=== CAPUCHA ==="
            );


            telemetry.addData(
                    "Fase",
                    "%d / 4",
                    capuchaFase + 1
            );


            telemetry.addData(
                    "Servo Position",
                    "%.3f",
                    capuchaPosition
            );


            double anguloRelativo =
                    capuchaFase
                            *
                            40.0;


            telemetry.addData(
                    "Angulo relativo",
                    "%.0f grados",
                    anguloRelativo
            );



            // =================================================
            // TELEMETRY INTAKE
            // =================================================

            telemetry.addLine("");


            telemetry.addLine(
                    "=== INTAKE ==="
            );


            telemetry.addData(
                    "Intake 1 Power",
                    intake
                            .intake1
                            .getPower()
            );


            telemetry.addData(
                    "Indexer Power",
                    intake
                            .intake2
                            .getPower()
            );


            telemetry.addData(
                    "Distancia Sensor",
                    "%.2f cm",
                    intake.getDistance()
            );


            telemetry.addData(
                    "Detecta Pelota",
                    intake.hasPiece()
            );


            telemetry.addData(
                    "Red",
                    intake.getRed()
            );


            telemetry.addData(
                    "Green",
                    intake.getGreen()
            );


            telemetry.addData(
                    "Blue",
                    intake.getBlue()
            );


            telemetry.addData(
                    "Alpha",
                    intake.getAlpha()
            );


            // =================================================
            // CONTROLES INTAKE
            // =================================================

            telemetry.addLine("");


            telemetry.addLine(
                    "X = Auto Collect"
            );


            telemetry.addLine(
                    "B = Reversa"
            );


            telemetry.addLine(
                    "Y = Adelante manual"
            );


            // =================================================
            // UPDATE
            // =================================================

            telemetry.update();


            // Loop cada 20ms
            sleep(
                    20
            );
        }



        // =====================================================
        // STOP TODO
        // =====================================================

        shooter.stop();


        intake.stop();


        limelight.stop();
    }
}