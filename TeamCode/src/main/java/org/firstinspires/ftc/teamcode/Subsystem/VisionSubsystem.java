package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class VisionSubsystem {

    private Limelight3A limelight;

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public LLResultTypes.FiducialResult getBestTarget() {

        LLResult result = limelight.getLatestResult();

        if (result == null) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags == null || tags.size() == 0) return null;

        return tags.get(0); // mejor tag
    }
}