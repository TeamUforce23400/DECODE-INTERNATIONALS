// Turret.java
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.teamcode.globals.Localization;

@Configurable
public class Turret extends SubsystemBase {
    public final ServoEx servoRightFront;
    public final ServoEx servoLeftFront;
    public final ServoEx servoRightBack;
    public final ServoEx servoLeftBack;
    public final ServoExGroup turretServos;

    // Turret heading + Servo variables
    // IMPORTANT: BOTH THE SERVOS MUST BE SYNCHRONIZED POSITION WISE.
    public final double minimumValueRad = 0; // TODO: Make sure that at pos 0 of servos, the turret is facing you when the robot is facing away (away from the front of the robot/facing backwards);
    public final double maximumValueRad = 2*Math.PI;
    public final double minPosServos = 0;
    public final double maxPosServos = 1.0; // TODO: At 2pi (360 degrees from 0 pos of servos), check the position of servos and set it to this variable.

    public Turret(HardwareMap hardwareMap) {
        servoRightFront = new ServoEx(hardwareMap, "trf");
        servoLeftFront = new ServoEx(hardwareMap, "tlf");
        servoRightBack = new ServoEx(hardwareMap, "trr");
        servoLeftBack = new ServoEx(hardwareMap, "tlr");

        turretServos = new ServoExGroup(servoRightFront, servoLeftFront, servoRightBack, servoLeftBack);
        turretServos.setPwm(new PwmControl.PwmRange(500,2500));


        // TODO: Make sure both servo values increase/decrease together, otherwise use the .inverted(true) method.
        servoLeftFront.setInverted(true);
        servoRightFront.setInverted(true);
        servoLeftBack.setInverted(true);
        servoRightBack.setInverted(true);
    }


    @Override
    public void periodic() {
        Pose robotPos = Localization.getPose();
        Pose goalPose = chosenAlliance.equals("RED") ? redGoalPose : blueGoalPose;

        double targetPos = headingToTurretPos(calculateTargetHeading(robotPos, goalPose));

        turretServos.set(targetPos);
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

    public double getServoPosition() {
        return servoLeftBack.getRawPosition();
    }

    public void setPosition(double position) {
        turretServos.set(position);
    }

}