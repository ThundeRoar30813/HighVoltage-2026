package org.firstinspires.ftc.teamcode.commands.shooter;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;

import java.util.List;
import java.util.TreeMap;


public class ShooterCmd {

    private final ShooterSubsystem shooter;
    private final Limelight3A limelight;
    private final Gamepad g2;

    private final Servo capucha;

    private final TreeMap<Double, Double> table = new TreeMap<>();


    // =========================================================
    // SHOOTER
    // =========================================================

    private static final double IDLE_VELOCITY =50.0;

    private static final double DECEL_STEP = 8.0;

    private double commandedVelocity = IDLE_VELOCITY;


    // =========================================================
    // MANUAL
    // =========================================================

    private boolean manualMode = false;

    private double manualVelocity = 900;


    // =========================================================
    // OFFSET
    // =========================================================

    private static final double TABLE_STEP = 10.0;

    private double tableOffset = 0.0;


    // =========================================================
    // BOTONES
    // =========================================================

    private boolean lastRight = false;
    private boolean lastUp = false;
    private boolean lastDown = false;


    // =========================================================
    // LIMELIGHT
    // =========================================================

    private double lastAutoVelocity = 215.0;

    private double lastDistance = 0.0;

    private double lastBaseVelocity = 215.0;

    private double lastAdjustedVelocity = 215.0;


    // =========================================================
    // CAPUCHA
    // =========================================================

    /*
     * POSICIONES QUE PROBASTE
     */

    private static final double CAPUCHA_60 = 0.744;

    private static final double CAPUCHA_40 = 0.596;

    private static final double CAPUCHA_20 = 0.448;


    private double capuchaPosition = CAPUCHA_20;

    private int capuchaAngle = 20;


    // =========================================================
    // CAMBIOS DE DISTANCIA
    // =========================================================

    private static final double SWITCH_60_TO_40 = 0.95;

    private static final double SWITCH_40_TO_20 = 1.15;


    private static final double HYSTERESIS = 0.03;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public ShooterCmd(
            ShooterSubsystem shooter,
            Limelight3A limelight,
            Gamepad g2,
            HardwareMap hardwareMap
    ) {

        this.shooter = shooter;

        this.limelight = limelight;

        this.g2 = g2;


        // =====================================================
        // SERVO
        // MISMO NOMBRE QUE TU PROGRAMA QUE SÍ FUNCIONA
        // =====================================================

        capucha = hardwareMap.get(
                Servo.class,
                "cap"
        );


        // =====================================================
        // TABLA SHOOTER
        // =====================================================

        table.put(0.80, 215.0);

        table.put(0.90, 225.0);

        table.put(1.00, 225.0);

        table.put(1.10, 230.0);

        table.put(1.20, 235.0);

        table.put(1.30, 235.0);

        table.put(1.40, 245.0);

        table.put(1.50, 255.0);

        table.put(1.60, 265.0);

        table.put(1.80, 565.0);
        table.put(1.90, 585.0);


        // =====================================================
        // POSICIÓN INICIAL CAPUCHA
        // =====================================================

        capuchaAngle = 20;

        capuchaPosition = CAPUCHA_20;

        capucha.setPosition(
                capuchaPosition
        );
    }


    // =========================================================
    // EXECUTE TELEOP
    // =========================================================

    public void execute() {


        // =====================================================
        // MANUAL / AUTO
        // =====================================================

        if (
                g2.dpad_right
                        &&
                        !lastRight
        ) {

            manualMode = !manualMode;

            if (manualMode) {

                manualVelocity = 900;
            }
        }


        lastRight = g2.dpad_right;


        // =====================================================
        // UP
        // =====================================================

        if (
                g2.dpad_up
                        &&
                        !lastUp
        ) {

            if (manualMode) {

                manualVelocity += 25;

            } else {

                tableOffset += TABLE_STEP;
            }
        }


        // =====================================================
        // DOWN
        // =====================================================

        if (
                g2.dpad_down
                        &&
                        !lastDown
        ) {

            if (manualMode) {

                manualVelocity -= 25;

                if (manualVelocity < 0) {

                    manualVelocity = 0;
                }

            } else {

                tableOffset -= TABLE_STEP;
            }
        }


        lastUp = g2.dpad_up;
        lastDown = g2.dpad_down;


        // =====================================================
        // LIMELIGHT + CAPUCHA
        // =====================================================

        updateAutomaticTarget();


        // =====================================================
        // SHOOTER
        // =====================================================

        if (
                g2.right_trigger > 0.1
        ) {

            if (manualMode) {

                commandedVelocity = manualVelocity;

            } else {

                commandedVelocity = lastAutoVelocity;
            }

        } else {

            if (
                    commandedVelocity
                            >
                            IDLE_VELOCITY
            ) {

                commandedVelocity -= DECEL_STEP;

                if (
                        commandedVelocity
                                <
                                IDLE_VELOCITY
                ) {

                    commandedVelocity =
                            IDLE_VELOCITY;
                }

            } else {

                commandedVelocity =
                        IDLE_VELOCITY;
            }
        }


        shooter.setVelocity(
                commandedVelocity
        );
    }


    // =========================================================
    // ACTUALIZAR LIMELIGHT
    // =========================================================

    private void updateAutomaticTarget() {


        LLResult result =
                limelight.getLatestResult();


        if (
                result == null
                        ||
                        !result.isValid()
        ) {

            return;
        }


        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();


        if (
                tags == null
                        ||
                        tags.isEmpty()
        ) {

            return;
        }


        LLResultTypes.FiducialResult tag =
                tags.get(0);


        if (
                tag.getTargetPoseCameraSpace()
                        ==
                        null
        ) {

            return;
        }


        // =====================================================
        // POSICIÓN DEL TAG
        // =====================================================

        double x =
                tag
                        .getTargetPoseCameraSpace()
                        .getPosition()
                        .x;


        double z =
                tag
                        .getTargetPoseCameraSpace()
                        .getPosition()
                        .z;


        // =====================================================
        // DISTANCIA
        // =====================================================

        double distance =
                Math.sqrt(
                        x * x
                                +
                                z * z
                );


        lastDistance =
                distance;


        // =====================================================
        // VELOCIDAD SHOOTER
        // =====================================================

        double baseVelocity =
                getBaseVelocity(
                        distance
                );


        lastBaseVelocity =
                baseVelocity;


        double targetVelocity =
                baseVelocity
                        +
                        tableOffset;


        if (
                targetVelocity < 0
        ) {

            targetVelocity = 0;
        }


        lastAdjustedVelocity =
                targetVelocity;


        lastAutoVelocity +=
                (
                        targetVelocity
                                -
                                lastAutoVelocity
                )
                        *
                        0.15;


        // =====================================================
        // CAPUCHA
        // =====================================================

        updateCapucha(
                distance
        );
    }


    // =========================================================
    // CAPUCHA
    // =========================================================

    private void updateCapucha(
            double distance
    ) {


        // =====================================================
        // CERCA
        // 60°
        // =====================================================

        if (
                distance
                        <
                        SWITCH_60_TO_40
                                -
                                HYSTERESIS
        ) {

            setCapucha60();

            return;
        }


        // =====================================================
        // MEDIO
        // 40°
        // =====================================================

        if (
                distance
                        <
                        SWITCH_40_TO_20
                                -
                                HYSTERESIS
        ) {

            setCapucha40();

            return;
        }


        // =====================================================
        // LEJOS
        // 20°
        // =====================================================

        setCapucha20();
    }


    // =========================================================
    // 60°
    // =========================================================

    private void setCapucha60() {


        if (
                capuchaAngle == 60
        ) {

            return;
        }


        capuchaAngle = 60;

        capuchaPosition = CAPUCHA_60;


        capucha.setPosition(
                capuchaPosition
        );
    }


    // =========================================================
    // 40°
    // =========================================================

    private void setCapucha40() {


        if (
                capuchaAngle == 40
        ) {

            return;
        }


        capuchaAngle = 40;

        capuchaPosition = CAPUCHA_40;


        capucha.setPosition(
                capuchaPosition
        );
    }


    // =========================================================
    // 20°
    // =========================================================

    private void setCapucha20() {


        if (
                capuchaAngle == 20
        ) {

            return;
        }


        capuchaAngle = 20;

        capuchaPosition = CAPUCHA_20;


        capucha.setPosition(
                capuchaPosition
        );
    }


    // =========================================================
    // INTERPOLACIÓN SHOOTER
    // =========================================================

    private double getBaseVelocity(
            double distance
    ) {


        Double low =
                table.floorKey(
                        distance
                );


        Double high =
                table.ceilingKey(
                        distance
                );


        if (low == null) {

            return table
                    .firstEntry()
                    .getValue();
        }


        if (high == null) {

            return table
                    .lastEntry()
                    .getValue();
        }


        if (
                low.equals(
                        high
                )
        ) {

            return table.get(
                    low
            );
        }


        double v1 =
                table.get(
                        low
                );


        double v2 =
                table.get(
                        high
                );


        return v1
                +
                (
                        distance
                                -
                                low
                )
                        *
                        (
                                v2
                                        -
                                        v1
                        )
                        /
                        (
                                high
                                        -
                                        low
                        );
    }


    // =========================================================
    // AUTÓNOMO IDLE
    // =========================================================

    public void autonomousIdle() {


        updateAutomaticTarget();


        commandedVelocity =
                IDLE_VELOCITY;


        shooter.setVelocity(
                commandedVelocity
        );
    }


    // =========================================================
    // AUTÓNOMO SHOOT
    // =========================================================

    public void autonomousShoot() {


        updateAutomaticTarget();


        commandedVelocity =
                lastAutoVelocity;


        shooter.setVelocity(
                commandedVelocity
        );
    }


    // =========================================================
    // ACTUALIZAR TARGET
    // =========================================================

    public void updateAutonomousTarget() {


        updateAutomaticTarget();
    }


    // =========================================================
    // STOP
    // =========================================================

    public void autonomousStop() {


        commandedVelocity = 0;

        shooter.stop();
    }


    // =========================================================
    // GETTERS
    // =========================================================

    public boolean isManualMode() {

        return manualMode;
    }


    public double getManualVelocity() {

        return manualVelocity;
    }


    public double getLastAutoVelocity() {

        return lastAutoVelocity;
    }


    public double getLastDistance() {

        return lastDistance;
    }


    public double getLastBaseVelocity() {

        return lastBaseVelocity;
    }


    public double getTableOffset() {

        return tableOffset;
    }


    public double getLastAdjustedVelocity() {

        return lastAdjustedVelocity;
    }


    public double getIdleVelocity() {

        return IDLE_VELOCITY;
    }


    public double getCommandedVelocity() {

        return commandedVelocity;
    }


    public double getCapuchaPosition() {

        return capuchaPosition;
    }


    public int getCapuchaAngle() {

        return capuchaAngle;
    }
}