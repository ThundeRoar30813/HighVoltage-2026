package org.firstinspires.ftc.teamcode.commands.drive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime; // Añadido para mejor control del tiempo en FTC

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;

public class MecanumDriveCmd {

    private DriveSubsystem drive;
    private Gamepad gamepad;
    private Limelight3A limelight;
    private ElapsedTime timer; // Temporizador dedicado para el ciclo PD

    /*
     * PD VALUES
     */
    double KP = 0.0152;
    double KD = 0.0016;

    double error = 0;
    double lastError = 0;

    double curTime = 0;
    double lastTime = 0;

    double angleTolerance = 0.4; // Umbral de tolerancia en grados para evitar vibraciones en el centro

    public MecanumDriveCmd(
            DriveSubsystem drive,
            Gamepad gamepad,
            Limelight3A limelight
    ) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.limelight = limelight;
        this.timer = new ElapsedTime(); // Inicializa el cronómetro
        this.timer.reset();
    }

    public void execute() {

        /*
         * MANUAL DRIVE
         */
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        /*
         * AUTO ALIGN
         */
        if (gamepad.left_trigger > 0.3) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();

                /*
                 * CORRECCIÓN DE ERROR (SIGN FIX):
                 * Si el Tag está a la derecha, tx es positivo, queremos rotar positivo (derecha).
                 * Por lo tanto: error = tx - objetivo (0) -> error = tx;
                 */
                error = tx;

                // Si está muy cerca del centro, deja de rotar para mitigar oscilaciones microscópicas
                if (Math.abs(error) < angleTolerance) {
                    turn = 0;
                } else {
                    /*
                     * P TERM
                     */
                    double pTerm = error * KP;

                    /*
                     * D TERM
                     */
                    curTime = timer.seconds(); // Usar segundos del timer de FTC es más preciso que Millis/1000.0
                    double dt = curTime - lastTime;

                    if (dt <= 0) {
                        dt = 0.001;
                    }

                    double dTerm = ((error - lastError) / dt) * KD;

                    /*
                     * FINAL ROTATE
                     */
                    turn = Range.clip(pTerm + dTerm, -0.45, 0.45);
                }

                /*
                 * SAVE VALUES
                 * Siempre guarda el estado si el resultado fue válido para el próximo ciclo
                 */
                lastError = error;
                lastTime = curTime;

            } else {
                // Si está presionando el gatillo pero Limelight no ve nada, resetea el histórico
                lastError = 0;
                lastTime = timer.seconds();
                error = 0; // Resetea el error público para la telemetría
            }

        } else {
            // Si no se presiona el gatillo, mantén el histórico al día para evitar picos derivados
            lastError = 0;
            lastTime = timer.seconds();
            error = 0; // Resetea el error público para la telemetría
        }

        /*
         * DRIVE
         */
        drive.drive(
                forward,
                strafe,
                turn
        );
    }

    /*
     * TELEMETRY VALUES
     */
    public double getError() {
        return error;
    }

    public double getKP() {
        return KP;
    }

    public double getKD() {
        return KD;
    }
}