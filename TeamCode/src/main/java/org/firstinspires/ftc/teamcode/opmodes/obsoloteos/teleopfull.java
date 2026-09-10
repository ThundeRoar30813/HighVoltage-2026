package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.drive.MecanumDriveCmd;

@Disabled
@TeleOp(name = "TeleopLimeDistanciaPROBAR2", group = "TeleOp")
public class teleopfull extends LinearOpMode {
    // =========================
    // DRIVETRAIN
    // =========================


    // =========================
    // SHOOTER
    // =========================
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // =========================
    // INTAKE
    // =========================
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor1;

    // =========================
    // LIMELIGHT + IMU
    // =========================
    private Limelight3A limelight;
    private IMU imu;

    // =========================
    // VARIABLES SHOOTER
    // =========================
    private double shooterTargetVelocity = 0;

    private final double SHOOTER_MAX_VELOCITY = 6500;
    private final double SHOOTER_MIN_VELOCITY = 0;

    private final double SHOOTER_KP = 0.0005;

    @Override
    public void runOpMode() throws InterruptedException {

        // =========================
        // INIT DRIVE
        // =========================



        // =========================
        // INIT SHOOTER
        // =========================
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shore");
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterRight = hardwareMap.get(DcMotorEx.class, "school");
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // =========================
        // INIT INTAKE
        // =========================
        intakeMotor = hardwareMap.get(DcMotorEx.class, "mIntake1");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "mIntake2");
        intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // =========================
        // INIT LIMELIGHT
        // =========================
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // =========================
        // INIT IMU
        // =========================
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // =========================
        // TELEMETRY INIT
        // =========================
        telemetry.addLine("Robot inicializado");
        telemetry.addLine("Esperando start...");
        telemetry.update();

        waitForStart();

        // =========================
        // START LIMELIGHT
        // =========================
        limelight.start();

        // =========================
        // LOOP PRINCIPAL
        // =========================
        while (opModeIsActive()) {

            // =========================
            // DRIVE
            // =========================


            // =========================
            // CONTROL SHOOTER
            // =========================

            // Aumentar velocidad
            if (gamepad2.dpad_down) {
                shooterTargetVelocity += 150;

                if (shooterTargetVelocity > SHOOTER_MAX_VELOCITY) {
                    shooterTargetVelocity = SHOOTER_MAX_VELOCITY;
                }

                sleep(150);
            }

            // Disminuir velocidad
            if (gamepad2.dpad_up) {
                shooterTargetVelocity -= 150;

                if (shooterTargetVelocity < SHOOTER_MIN_VELOCITY) {
                    shooterTargetVelocity = SHOOTER_MIN_VELOCITY;
                }

                sleep(150);
            }

            // Stop shooter
            if (gamepad2.y) {
                shooterTargetVelocity = 0;
                sleep(150);
            }

            // =========================
            // PID SIMPLE SHOOTER
            // =========================
            double leftVelocity = shooterLeft.getVelocity();
            double rightVelocity = shooterRight.getVelocity();

            double currentVelocity =
                    (leftVelocity + rightVelocity) / 2.0;

            double error =
                    shooterTargetVelocity - currentVelocity;

            double power =
                    error * SHOOTER_KP;

            // Clamp
            if (power > 1.0) power = 1.0;
            if (power < -1.0) power = -1.0;

            shooterLeft.setPower(power);
            shooterRight.setPower(power);

            // =========================
            // CONTROL INTAKE
            // =========================

            // Reversa
            if (gamepad2.right_bumper) {

                intakeMotor.setPower(-1.0);
                intakeMotor1.setPower(-1.0);

            }
            // Adelante
            else if (gamepad2.left_bumper) {

                intakeMotor.setPower(1.0);
                intakeMotor1.setPower(1.0);

            }
            // Velocidad media
            else if (gamepad2.x) {

                intakeMotor.setPower(0.8);
                intakeMotor1.setPower(0.8);

            }
            // Stop
            else if (gamepad2.b) {

                intakeMotor.setPower(0);
                intakeMotor1.setPower(0);

            }

            // =========================
            // LIMELIGHT + APRILTAG
            // =========================
            YawPitchRollAngles robotOrientation =
                    imu.getRobotYawPitchRollAngles();

            limelight.updateRobotOrientation(
                    robotOrientation.getYaw()
            );

            LLResult llResult =
                    limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {

                Pose3D botPose =
                        llResult.getBotpose();

                Position posePos =
                        botPose.getPosition();

                telemetry.addLine("=== APRILTAG ===");
                telemetry.addData("Distancia", posePos);
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());

            } else {

                telemetry.addLine("=== APRILTAG ===");
                telemetry.addLine("No Tag Detectado");
            }

            // =========================
            // TELEMETRY SHOOTER
            // =========================
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Velocidad Actual", currentVelocity);
            telemetry.addData("Velocidad Objetivo", shooterTargetVelocity);
            telemetry.addData("Potencia", power);
            telemetry.addData("Error", error);

            // =========================
            // TELEMETRY INTAKE
            // =========================
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("Intake Power", intakeMotor.getPower());

            telemetry.update();

            sleep(20);
        }

        // =========================
        // STOP TODO
        // =========================
        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        intakeMotor.setPower(0);
        intakeMotor1.setPower(0);

        limelight.stop();
    }
}
