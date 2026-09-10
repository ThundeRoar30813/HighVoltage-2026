package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.*;
import org.firstinspires.ftc.teamcode.commands.capucha.CapuchaCmd;
import org.firstinspires.ftc.teamcode.commands.drive.*;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCmd;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterCmd;


@TeleOp(name="TeleOp PRIMEEEE")
public class TeleopIntakeIndexer extends OpMode {

    CapuchaSubsystem capucha = new CapuchaSubsystem();
    DriveSubsystem drive = new DriveSubsystem();
    ShooterSubsystem shooter = new ShooterSubsystem();
    IntakeSubsystem intake = new IntakeSubsystem();

    IntakeCmd intakeCmd;
    CapuchaCmd capuchaCmd;
    MecanumDriveCmd driveCmd;
    ShooterCmd shooterCmd;

    Limelight3A limelight;


    @Override
    public void init() {

        // =====================================================
        // SUBSYSTEMS
        // =====================================================

        drive.init(hardwareMap);

        shooter.init(hardwareMap);

        intake.init(hardwareMap);


        // =====================================================
        // LIMELIGHT
        // =====================================================

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );

        limelight.pipelineSwitch(0);

        limelight.start();


        // =====================================================
        // COMMANDS
        // =====================================================

        intakeCmd =
                new IntakeCmd(
                        intake,
                        gamepad1
                );


        // =====================================================
        // SHOOTER + CAPUCHA
        //
        // IMPORTANTE:
        // PASAMOS hardwareMap
        //
        // ShooterCmd buscará internamente:
        //
        // Servo.class, "cap"
        // =====================================================

        shooterCmd =
                new ShooterCmd(
                        shooter,
                        limelight,
                        gamepad1,
                        hardwareMap
                );


        driveCmd =
                new MecanumDriveCmd(
                        drive,
                        gamepad1,
                        limelight
                );


        telemetry.addLine("READY");

        telemetry.update();
    }


    @Override
    public void loop() {


        // =====================================================
        // EJECUTAR COMMANDS
        // =====================================================

        intakeCmd.execute();

        driveCmd.execute();

        shooterCmd.execute();


        // =====================================================
        // TELEMETRY SHOOTER
        // =====================================================

        telemetry.addData(
                "Manual Mode",
                shooterCmd.isManualMode()
        );


        telemetry.addData(
                "Manual Target",
                shooterCmd.getManualVelocity()
        );


        telemetry.addData(
                "Shooter Vel",
                shooter.getAvgVelocity()
        );


        telemetry.addData(
                "Left",
                shooter.left.getVelocity()
        );


        telemetry.addData(
                "Right",
                shooter.right.getVelocity()
        );


        // =====================================================
        // LIMELIGHT
        // =====================================================

        telemetry.addData(
                "Distancia Limelight",
                "%.3f m",
                shooterCmd.getLastDistance()
        );


        telemetry.addData(
                "Target automático",
                "%.1f",
                shooterCmd.getLastAutoVelocity()
        );


        // =====================================================
        // CAPUCHA
        // =====================================================

        telemetry.addData(
                "Capucha angulo",
                "%d grados",
                shooterCmd.getCapuchaAngle()
        );


        telemetry.addData(
                "Capucha servo",
                "%.3f",
                shooterCmd.getCapuchaPosition()
        );


        // =====================================================
        // AUTO AIM
        // =====================================================

        telemetry.addData(
                "AutoAlign Error",
                driveCmd.getError()
        );


        telemetry.update();
    }
}