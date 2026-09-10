package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import java.util.List;
import java.util.TreeMap;

import org.firstinspires.ftc.teamcode.Subsystem.*;
import org.firstinspires.ftc.teamcode.commands.drive.*;
import org.firstinspires.ftc.teamcode.commands.capucha.*;
@Disabled

@TeleOp(name="TeleOp AUTO AIM PROPROBAR3")
public class TeleOpCB extends OpMode {

    // ================= SUBSYSTEMS =================

    //IntakeSubsystem intake = new IntakeSubsystem();
    CapuchaSubsystem capucha = new CapuchaSubsystem();

    // ================= COMMANDS =================

    //IntakeCmd intakeCmd;
    CapuchaCmd capuchaCmd;

    // ================= SHOOTER =================
    DcMotorEx shooterLeft;
    DcMotorEx shooterRight;

    // ================= LIMELIGHT =================
    Limelight3A limelight;

    // ================= TABLA =================
    TreeMap<Double, Double> shooterTable = new TreeMap<>();

    @Override
    public void init() {


        //intake.init(hardwareMap);


        // SHOOTER
        shooterLeft = hardwareMap.get(DcMotorEx.class, "school");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shore");

        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setVelocityPIDFCoefficients(4.007, 0, 0, 17.120);
        shooterRight.setVelocityPIDFCoefficients(4.007, 0, 0, 17.120);

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        shooterTable.put(1.47, 1000.0);
        shooterTable.put(1.67, 1160.0);
        shooterTable.put(1.87, 1180.0);
        shooterTable.put(2.17, 1225.0);
        shooterTable.put(2.4, 1300.0);
        shooterTable.put(2.5, 1350.0);
        shooterTable.put(3.0, 1670.0);
        // TABLA DISTANCIA → VELOCIDAD
//        shooterTable.put(1.0, 1000.0);
//        shooterTable.put(1.2, 1160.0);
//        shooterTable.put(1.4, 1180.0);
//        shooterTable.put(1.7, 1225.0);
//        shooterTable.put(1.8, 1300.0);
//        shooterTable.put(2.1, 1350.0);
//        shooterTable.put(3.0, 1670.0);


        //intakeCmd = new IntakeCmd(intake, gamepad2, gamepad2);


        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ================= DRIVE =================

        // ================= LIMELIGHT (TU MÉTODO CORRECTO) =================
        LLResult result = limelight.getLatestResult();

        double distance = 0;
        boolean tagDetected = false;

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (!tags.isEmpty()) {

                tagDetected = true;

                LLResultTypes.FiducialResult tag = tags.get(0);

                double x = tag.getTargetPoseCameraSpace().getPosition().x;
                double y = tag.getTargetPoseCameraSpace().getPosition().y;
                double z = tag.getTargetPoseCameraSpace().getPosition().z;

                distance = Math.sqrt(x*x + y*y + z*z);
            }
        }

        // ================= SHOOTER AUTO =================
        if (gamepad2.right_trigger > 0.1) {

            if (tagDetected) {

                double targetVelocity = getInterpolatedVelocity(distance);

                shooterLeft.setVelocity(targetVelocity);
                shooterRight.setVelocity(targetVelocity);

                telemetry.addData("Distance", distance);
                telemetry.addData("Target", targetVelocity);

            } else {
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
            }

        } else {
            shooterLeft.setVelocity(0);
            shooterRight.setVelocity(0);
        }

        // ================= OTROS =================
        //intakeCmd.execute();


        // ================= TELEMETRY =================
        telemetry.addData("Tag", tagDetected);
        telemetry.addData("Distance", distance);
        telemetry.addData("Avg Vel",
                (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0);

        telemetry.update();
    }

    // ================= INTERPOLACIÓN =================
    public double getInterpolatedVelocity(double distance) {

        Double low = shooterTable.floorKey(distance);
        Double high = shooterTable.ceilingKey(distance);

        if (low == null) return shooterTable.firstEntry().getValue();
        if (high == null) return shooterTable.lastEntry().getValue();
        if (low.equals(high)) return shooterTable.get(low);

        double v1 = shooterTable.get(low);
        double v2 = shooterTable.get(high);

        return v1 + (distance - low) * (v2 - v1) / (high - low);
    }
}