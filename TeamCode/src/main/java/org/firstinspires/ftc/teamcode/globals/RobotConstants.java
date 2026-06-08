package org.firstinspires.ftc.teamcode.globals;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.hardware.SensorDistanceEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class RobotConstants {

    public static Pose blueGoalPose = new Pose(2, 141, Math.toRadians(90));
    public static Pose redGoalPose  = new Pose(141+0.5, 141+0.5, Math.toRadians(90));
    public static Pose redDistancePose = new Pose(138, 138, Math.toRadians(90));

    public static String chosenAlliance = "RED";
    public static Pose savedPose = null;





}
