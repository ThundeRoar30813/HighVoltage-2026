package org.firstinspires.ftc.teamcode.commands.intake;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class IntakeCmd {

    IntakeSubsystem intake;
    Gamepad g2;

    public IntakeCmd(IntakeSubsystem intake, Gamepad g2) {
        this.intake = intake;
        this.g2 = g2;
    }

    public void execute() {
        boolean hasPiece = intake.hasPiece();

        // 🟦 X = Intake Normal
        if (g2.x) {
            intake.intake1.setPower(1.0);
            if (hasPiece) {
                intake.intake2.setPower(0);
            } else {

                intake.intake2.setPower(-0.5);
            }
        }
        // 🔵 B
        else if (g2.b) {
            intake.set(-1.0, 0.7);
        }
        // 🔺 Y
        else if (g2.y) {
            intake.set(1.0, -0.85 );
        }
        else {
            // PROTECCIÓN: Si estás intentando disparar (gatillo presionado),
            // dejamos que ShooterCmd tome el control total del Intake/Indexer.
            if (g2.right_trigger <= 0.1) {
                intake.stop();
            }
        }
    }
}

