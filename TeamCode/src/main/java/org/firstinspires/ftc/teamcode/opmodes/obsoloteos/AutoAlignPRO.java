package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "AutoAlign PROPROBAR5", group = "Teleop")
public class AutoAlignPRO extends LinearOpMode {

    private DcMotor fl, fr, bl, br;

    // Target fijo (puedes cambiarlo con botones)
    private double targetX = 8;
    private double targetY = 136;

    // Simulación REAL (luego puedes reemplazar por PedroPathing)
    private double robotX = 50;
    private double robotY = 50;
    private double robotHeading = 0; // RADIANES

    private final double DEADBAND = Math.toRadians(3);

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "mfl");
        fr = hardwareMap.get(DcMotor.class, "mfr");
        bl = hardwareMap.get(DcMotor.class, "mbl");
        br = hardwareMap.get(DcMotor.class, "mbr");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // MOVIMIENTO MANUAL
            // =========================
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;

            // =========================
            // AUTO ALIGN
            // =========================
            boolean autoAlign = gamepad1.left_trigger > 0.5;

            double turn = gamepad1.right_stick_x;

            if (autoAlign) {

                double error = getError(targetX, targetY);

                turn = calcTurn(error);
            }

            // =========================
            // DRIVE MECANUM
            // =========================
            double flPower = forward + strafe + turn;
            double frPower = forward - strafe - turn;
            double blPower = forward - strafe + turn;
            double brPower = forward + strafe - turn;

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            // =========================
            // TELEMETRÍA
            // =========================
            telemetry.addData("AutoAlign", autoAlign);
            telemetry.addData("Error deg", Math.toDegrees(getError(targetX, targetY)));
            telemetry.addData("Heading", Math.toDegrees(robotHeading));

            telemetry.update();
        }
    }

    // =========================
    // ERROR ANGULAR
    // =========================
    private double getError(double tx, double ty) {

        double targetAngle = Math.atan2(ty - robotY, tx - robotX);

        double error = targetAngle - robotHeading;

        return Math.atan2(Math.sin(error), Math.cos(error));
    }

    // =========================
    // CONTROL DE GIRO (P SIMPLE ESTABLE)
    // =========================
    private double calcTurn(double error) {

        double errorDeg = Math.toDegrees(Math.abs(error));

        if (errorDeg < 3) return 0;

        double speed = errorDeg / 180.0;

        speed = Math.max(0.09, Math.min(.86, speed));

        return speed * Math.signum(error);
    }
}