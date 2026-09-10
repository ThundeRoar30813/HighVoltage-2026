package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    public DcMotor intake1;
    public DcMotor intake2;

    // REV Color Sensor V3
    public RevColorSensorV3 sensor;


    public void init(HardwareMap hw) {

        intake1 = hw.get(DcMotor.class, "mIntake2");
        intake2 = hw.get(DcMotor.class, "mIntake1");

        // IMPORTANTE:
        // En la configuración del robot debe aparecer
        // como "REV Color Sensor V3"
        // y llamarse "sensorc1"
        sensor = hw.get(
                RevColorSensorV3.class,
                "sensorc1"
        );
    }


    // =====================================================
    // DETECTAR PIEZA
    // =====================================================

    public boolean hasPiece() {

        double distance = sensor.getDistance(
                DistanceUnit.CM
        );

        return distance < 4.7;
    }


    // =====================================================
    // OBTENER DISTANCIA
    // =====================================================

    public double getDistance() {

        return sensor.getDistance(
                DistanceUnit.CM
        );
    }


    // =====================================================
    // VALORES DE COLOR
    // =====================================================

    public int getRed() {
        return sensor.red();
    }


    public int getGreen() {
        return sensor.green();
    }


    public int getBlue() {
        return sensor.blue();
    }


    public int getAlpha() {
        return sensor.alpha();
    }


    // =====================================================
    // CONTROL MANUAL DE INTAKE
    // =====================================================

    public void set(
            double a,
            double b
    ) {

        intake1.setPower(a);
        intake2.setPower(b);
    }


    // =====================================================
    // RECOGER AUTOMÁTICAMENTE
    // =====================================================

    public void autoCollect() {

        // Primer motor siempre recoge
        intake1.setPower(1.0);

        // Cuando detecta la pelota/pieza
        if (hasPiece()) {

            // Detiene segundo motor
            intake2.setPower(0);

        } else {

            // Sigue recogiendo
            intake2.setPower(-0.50);
        }
    }


    // =====================================================
    // ALIMENTAR SHOOTER
    // =====================================================

    public void feedShooter() {

        set(
                1.0,
                -1.0
        );
    }


    // =====================================================
    // DETENER
    // =====================================================

    public void stop() {

        set(
                0,
                0
        );
    }
}