package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15)
            .forwardZeroPowerAcceleration(-31.2076591263902)
            .lateralZeroPowerAcceleration(-75.97758469619)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0.007,0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.6,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.030,0,0,0.3,0.02))
            .centripetalScaling(0.0005)
            .useSecondaryDrivePIDF(false);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("mfr")
            .rightRearMotorName("mbr")
            .leftRearMotorName("mbl")
            .leftFrontMotorName("mfl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.22454305333416)
            .yVelocity(55.9084525521346);




    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-240.099998)
            .strafePodX(180.5814885)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pip")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

//    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
//            .forwardTicksToInches(.001989436789)
//            .strafeTicksToInches(.001989436789)
//            .turnTicksToInches(.001989436789)
//            .leftPodY(3.5)
//            .rightPodY(-3.5)
//            .strafePodX(-0.75)
//            .leftEncoder_HardwareMapName("mbl")
//            .rightEncoder_HardwareMapName("mfr")
//            .strafeEncoder_HardwareMapName("mfl")
//            .leftEncoderDirection(Encoder.FORWARD)
//            .rightEncoderDirection(Encoder.FORWARD)
//            .strafeEncoderDirection(Encoder.FORWARD)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
