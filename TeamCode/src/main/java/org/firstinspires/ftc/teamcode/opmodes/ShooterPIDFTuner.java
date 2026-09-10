package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.drive.MecanumDriveCmd;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCmd;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterCmd;

@Disabled
@TeleOp(name = "Shooter PIDF Tuner")
public class ShooterPIDFTuner extends OpMode {

    DriveSubsystem drive = new DriveSubsystem();
    IntakeSubsystem intake = new IntakeSubsystem();
    Limelight3A limelight;
    DcMotorEx left;
    DcMotorEx right;

    MecanumDriveCmd driveCmd;

    IntakeCmd intakeCmd;

    // =========================
    // TARGET VELOCITY
    // =========================
    double targetVelocity = 800;

    // =========================
    // PIDF
    // =========================
    double p = 4.0;
    double i = 0.0;
    double d = 0.5;
    double f = 12.0;

    // =========================
    // BUTTON MEMORY
    // =========================
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastLeft = false;
    boolean lastRight = false;

    boolean shooterEnabled = false;
    boolean lastA = false; // Memoria para el botón de abajo (A)

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);

        intakeCmd = new IntakeCmd(intake, gamepad1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        left = hardwareMap.get(DcMotorEx.class, "school");
        right = hardwareMap.get(DcMotorEx.class, "shore");

        driveCmd = new MecanumDriveCmd(
                drive,
                gamepad1,
                limelight
        );

        right.setDirection(DcMotorEx.Direction.REVERSE);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        applyPIDF();
    }

    @Override
    public void loop() {
        // intakeCmd.execute(); // Descoméntalo si ya lo tienes inicializado



        // =========================
        // TOGGLE SHOOTER (Botón A / Botón de abajo)
        // =========================
        if (gamepad1.a && !lastA) {
            shooterEnabled = !shooterEnabled;
        }
        lastA = gamepad1.a;

        // =========================
        // RPM CONTROL
        // =========================
        targetVelocity += gamepad1.right_trigger * 15;
        targetVelocity -= gamepad1.left_trigger * 15;

        if (targetVelocity < 0) {
            targetVelocity = 0;
        }

        // =========================
        // PIDF CONTROL (Se quedan intactos en las flechas)
        // =========================

        // DPAD UP = P
        if (gamepad1.dpad_up && !lastUp) {
            p += 0.2;
            applyPIDF();
        }

        // DPAD DOWN = F
        if (gamepad1.dpad_down && !lastDown) {
            f += 0.5;
            applyPIDF();
        }

        // DPAD RIGHT = D
        if (gamepad1.dpad_right && !lastRight) {
            d += 0.1;
            applyPIDF();
        }

        // DPAD LEFT = I
        if (gamepad1.dpad_left && !lastLeft) {
            i += 0.01;
            applyPIDF();
        }

        // =========================
        // SHOOTER EXECUTION (¡Corregido el error del tipo!)
        // =========================
        if (shooterEnabled) {
            left.setVelocity(targetVelocity);
            right.setVelocity(targetVelocity);
        } else {
            left.setVelocity(0);
            right.setVelocity(0);
        }

        // =========================
        // LIMELIGHT DISTANCE
        // =========================
        LLResult result = limelight.getLatestResult();
        double distancia = 0;
        boolean targetVisible = false;

        if (result != null && result.isValid()) {
            targetVisible = true;

            // En el SDK de Limelight para FTC, getBotpose() te permite obtener
            // los ejes de posición directamente mediante .getZ() en entornos 3D.
            if (result.getBotpose() != null) {
                distancia = Math.abs(result.getBotpose().getPosition().z);
            } else {
                // Si el pipeline no es de AprilTags en 3D, usamos el ángulo vertical clásico
                distancia = result.getTy();
            }
        }

        driveCmd.execute();
        intakeCmd.execute();
        // =========================
        // TELEMETRY
        // =========================
        telemetry.addLine("===== SHOOTER PIDF TUNER =====");
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Left Velocity", left.getVelocity());
        telemetry.addData("Right Velocity", right.getVelocity());

        telemetry.addLine("--- LIMELIGHT ---");
        telemetry.addData("¿Target Visible?", targetVisible);
        telemetry.addData("Distancia/Ángulo", "%.2f", distancia);

        telemetry.addLine("");
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);

        telemetry.addLine("");
        telemetry.addLine("A (Botón Abajo) = TOGGLE SHOOTER");
        telemetry.addLine("RT = MORE RPM | LT = LESS RPM");
        telemetry.addLine("UP = ADD P    | DOWN = ADD F");
        telemetry.addLine("RIGHT = ADD D | LEFT = ADD I");

        telemetry.update();

        // =========================
        // SAVE BUTTON STATES
        // =========================
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
    }

    // =========================
    // APPLY PIDF
    // =========================
    public void applyPIDF() {
        left.setVelocityPIDFCoefficients(p, i, d, f);
        right.setVelocityPIDFCoefficients(p, i, d, f);
    }
}