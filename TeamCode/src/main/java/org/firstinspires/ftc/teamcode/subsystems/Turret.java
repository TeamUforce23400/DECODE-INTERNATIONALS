// Turret.java
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.teamcode.globals.Localization;

@Configurable
public class Turret extends SubsystemBase {
    public final ServoEx servoRight;
    public final ServoEx servoLeft;
    public final ServoExGroup turretServos;

    // Turret heading + Servo variables
    // IMPORTANT: BOTH THE SERVOS MUST BE SYNCHRONIZED POSITION WISE.
    public final double minimumValueRad = 0; // TODO: Make sure that at pos 0 of servos, the turret is facing you when the robot is facing away (away from the front of the robot/facing backwards);
    public final double maximumValueRad = 2*Math.PI;
    public final double minPosServos = 0;
    public final double maxPosServos = 1; // TODO: At 2pi (360 degrees from 0 pos of servos), check the position of servos and set it to this variable.

    public Turret(HardwareMap hardwareMap) {
        servoRight = new ServoEx(hardwareMap, "turretRight");
        servoLeft = new ServoEx(hardwareMap, "turretLeft");

        turretServos = new ServoExGroup(servoRight, servoLeft);

        // TODO: Make sure both servo values increase/decrease together, otherwise use the .inverted(true) method.
    }


    @Override
    public void periodic() {
        Pose robotPos = Localization.getPose();
        Pose goalPose = chosenAlliance.equals("RED") ? redGoalPose : blueGoalPose;

        turretServos.set(headingToTurretPos(calculateTargetHeading(robotPos, goalPose)));
    }

    public double calculateTargetHeading(Pose robotPos, Pose goalPose) {
        double absoluteTargetHeading = Math.atan2(goalPose.getY() - robotPos.getY(), goalPose.getX() - robotPos.getX());
        double robotHeading = robotPos.getHeading();
        double targetHeading = (absoluteTargetHeading - robotHeading) + Math.PI;

        if (targetHeading >= 2 * Math.PI) {
            targetHeading -= 2 * Math.PI;
        }
        else if (targetHeading < 0) {
            targetHeading += 2 * Math.PI;
        }

        return targetHeading;
    }

    public double headingToTurretPos(double heading) {
        return ((maxPosServos - minPosServos)*(heading - minimumValueRad)/(maximumValueRad-minimumValueRad)) + minPosServos;
    }

}
