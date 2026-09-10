package org.firstinspires.ftc.teamcode.ShooterPID;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp

@Disabled
public class ksTuner extends OpMode {
    DcMotor intake, indexer;
    FlyWheel flyWheel = new FlyWheel();
    public double kS = 0;
    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    int incrementIdx = 3;
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "mIntake1");
        indexer = hwMap.get(DcMotor.class, "mIntake2");
        indexer.setDirection(DcMotor.Direction.FORWARD);
        indexer.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void init() {
        flyWheel.init(hardwareMap);
    }
    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed() && incrementIdx < 4) {
            incrementIdx++;
        } else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0) {
            incrementIdx--;
        }
        double currentStep = increments[incrementIdx];
        if (gamepad1.dpadUpWasPressed()) {
            kS += currentStep;
        }
        if (gamepad1.dpadDownWasPressed()) {
            kS -= currentStep;
        }
        if(gamepad1.x) {
            indexer.setPower(1);
        }
        if(gamepad1.b) {
            indexer.setPower(-1);
        }
        if(gamepad1.left_bumper) {
            intake.setPower(1);
        }
        if(gamepad1.right_bumper) {
            intake.setPower(-1);
        }

        flyWheel.setMotorPower(kS);

        telemetry.addData("Step","%.6f", currentStep);
        telemetry.addData("kS", "%.6f", kS );
        telemetry.addData("RPM", flyWheel.getRPM());
        telemetry.addData("Ticks per Sec", flyWheel.getTicksPerSec());
    }
}