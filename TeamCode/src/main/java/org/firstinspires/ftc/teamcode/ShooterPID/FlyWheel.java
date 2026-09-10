package org.firstinspires.ftc.teamcode.ShooterPID;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;


@Disabled
public class FlyWheel {
    private DcMotorEx shooter1, shooter2;

    private double encoderCPM = 30;

    private double kV, kS, kP;

    private double gearRatio = 3;

    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotorEx.class, "school");
        shooter2 = hwMap.get(DcMotorEx.class, "shore");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setMotorPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }
    public double getTicksPerSec() {
        return shooter1.getVelocity();

    }
    public double getRPM() {
        return (getTicksPerSec()/encoderCPM * 60) / gearRatio;
    }


}