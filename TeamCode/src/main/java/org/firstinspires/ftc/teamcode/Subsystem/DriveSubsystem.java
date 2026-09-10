package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {

    private DcMotor fl, fr, bl, br;

    public void init(HardwareMap hw) {

        fl = hw.get(DcMotor.class, "mfl");
        fr = hw.get(DcMotor.class, "mfr");
        bl = hw.get(DcMotor.class, "mbl");
        br = hw.get(DcMotor.class, "mbr");

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

//
    }

    public void drive(double f, double s, double t) {

        fl.setPower(f + s + t);
        fr.setPower(f - s - t);
        bl.setPower(f - s + t);
        br.setPower(f + s - t);
    }
}