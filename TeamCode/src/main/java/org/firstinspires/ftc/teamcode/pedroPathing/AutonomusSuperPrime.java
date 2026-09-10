package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class AutonomusSuperPrime {
    private Servo gateServo;
    private CRServo flywheelServo;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {

        IOLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE,


    }

    private FlywheelState flywheelState;
    private double GATE_CLOSE_ANGLE = 0;
    private double GATE_OPEN_ANGLE = 90;
    private double GATE_OPEN_TIME = 0.4;
    private double GATE_CLOSE_TIME =0.4;
    private int shotsRemaining = 0;
 private double flywheelVelocity = 800;
 private double MIN_FLYWHEEL_RPM = 1100;
 private double FLYWHEEL_MAX_SPINUP_TIME = 2;

 private void init(HardwareMap hwMap)   {
     gateServo = hwMap.get(Servo.class,"servo");
     flywheelServo =hwMap.get(CRServo.class,"cr_servo");

     flywheelState = FlywheelState.IOLE;

     flywheelServo.setPower(0);
     
 }

}
