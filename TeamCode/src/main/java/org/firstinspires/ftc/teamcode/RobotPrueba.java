package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Mi primer teleoperado")
public class RobotPrueba extends OpMode {


    public DcMotor motor1;

    Gamepad g2;



    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor .class , "motor1");
    }

    @Override
    public void loop() {

        motor1.setPower(0.75);
    }
}
