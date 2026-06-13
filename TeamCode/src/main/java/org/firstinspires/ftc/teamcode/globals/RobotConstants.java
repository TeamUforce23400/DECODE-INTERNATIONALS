package org.firstinspires.ftc.teamcode.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.hardware.SensorDistanceEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class RobotConstants {

    public static Pose blueGoalPose = new Pose(3, 141, Math.toRadians(90));
    public static Pose redGoalPose  = new Pose(147, 141, Math.toRadians(90));
    public static Pose blueDistancePose  = new Pose(-3, 141, Math.toRadians(90));

    public static Pose redDistancePose = new Pose(141, 141, Math.toRadians(90));

    public static String chosenAlliance = "RED";
    public static Pose savedPose = null;





}
