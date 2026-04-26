package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.hardware.SensorDistanceEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class RobotConstants {
    public static double maxTurretPos = 480.625;
    public static double minTurretPos = -480.625;

    public MotorEx frontRight;
    public MotorEx frontLeft;
    public MotorEx backRight;
    public MotorEx backLeft;

    public ServoEx sIntakeA;
    public ServoEx sIntakeB;

    public SensorDistanceEx R1;
    public SensorDistanceEx R2;
    public SensorDistanceEx R3;

    public MotorEx intake;

    public MotorEx shooterA;
    public MotorEx shooterB;

    public static int maxEPT = 1900;

    // Calibrated hood map: 35deg -> 0.625, 65deg -> 1.0
    public static double maxHoodPos = 0.625;
    public static double minHoodPos = 1;
    public static double maxHoodAngle = 35;
    public static double minHoodAngle = 65;

    public static Pose blueGoalPose = new Pose(2, 141, Math.toRadians(90));
    public static Pose redGoalPose  = new Pose(141, 141, Math.toRadians(90));
    public static Pose farRedGoalPose = new Pose(141, 141);

    public static Pose redDistancePose = new Pose(138, 138, Math.toRadians(90));


    public static final int RED_GOAL_TAG_ID = 24;
    public static final int BLUE_GOAL_TAG_ID = 20;

    public static double engagePos = 0.1;
    public static double disengagePos = 0.5;

    public static final Pose resetPos = new Pose(135, 9, Math.toRadians(0));

    public static String chosenAlliance = "RED";
    public static Pose savedPose = null;

    public static Pose blueRampCP = new Pose(30.68223350253807, 58.12994923857866);
    public static Pose intakeRedRamp = new Pose(136, 57.7, Math.toRadians(26));
    public static Pose intakeRedRampDrifted = new Pose(135, 58.0, Math.toRadians(31));
    public static Pose intakeBlueRamp = intakeRedRamp.mirror();
    public static Pose redRampCP = new Pose(124, 60);
    public static Pose bluePark = new Pose(53.076142131979694, 23.350253807106576, Math.toRadians(180));
    public static Pose redPark = new Pose(53.076142131979694, 23.350253807106576, Math.toRadians(0));



}
