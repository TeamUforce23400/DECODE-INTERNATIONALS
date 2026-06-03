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

    public final double minimumValueRad = 0;
    public final double maximumValueRad = (400.0/180.0) * Math.PI; //Update degrees value (400 currently) incase of turret rotation angle change

    public final double minPosServos = 0.0;
    public final double maxPosServos = 1.0 ;

    public static double maxStepPerLoop = 0.015;

    public static double wrapLow = 0.05;
    public static double wrapHigh = 0.95;
    public static double safeMiddle = 0.5;

    private double currentServoPosition = 0.5;
    private double finalTargetPosition = 0.5;
    private boolean routingThroughMiddle = false;

    public Turret(HardwareMap hardwareMap) {
        servoRightFront = new ServoEx(hardwareMap, "trf");
        servoLeftFront = new ServoEx(hardwareMap, "tlf");
        servoRightBack = new ServoEx(hardwareMap, "trr");
        servoLeftBack = new ServoEx(hardwareMap, "tlr");

        PwmControl.PwmRange turretPwmRange = new PwmControl.PwmRange(500, 2500);

        servoRightFront.setPwm(turretPwmRange);
        servoLeftFront.setPwm(turretPwmRange);
        servoRightBack.setPwm(turretPwmRange);
        servoLeftBack.setPwm(turretPwmRange);

        turretServos = new ServoExGroup(
                servoRightFront,
                servoLeftFront,
                servoRightBack,
                servoLeftBack
        );

        servoLeftFront.setInverted(true);
        servoRightFront.setInverted(true);
        servoLeftBack.setInverted(true);
        servoRightBack.setInverted(true);

        turretServos.set(currentServoPosition);
    }

    @Override
    public void periodic() {
        Pose robotPos = Localization.getPose();
        Pose goalPose = chosenAlliance.equals("RED")
                ? redGoalPose
                : blueGoalPose;

        double targetHeading = calculateTargetHeading(robotPos, goalPose);
        double targetServoPosition = headingToTurretPos(targetHeading);

        targetServoPosition = clamp(targetServoPosition);

        updateSafeTarget(targetServoPosition);

        double activeTarget = routingThroughMiddle
                ? safeMiddle
                : finalTargetPosition;

        currentServoPosition = moveToward(
                currentServoPosition,
                activeTarget,
                maxStepPerLoop
        );

        if (routingThroughMiddle
                && Math.abs(currentServoPosition - safeMiddle) < 0.02) {
            routingThroughMiddle = false;
        }

        turretServos.set(currentServoPosition);
    }

    public double calculateTargetHeading(Pose robotPos, Pose goalPose) {
        double absoluteTargetHeading = Math.atan2(
                goalPose.getY() - robotPos.getY(),
                goalPose.getX() - robotPos.getX()
        );

        double robotHeading = robotPos.getHeading();

        double targetHeading =
                absoluteTargetHeading - robotHeading + Math.PI;

        if (targetHeading >= 2 * Math.PI) {
            targetHeading -= 2 * Math.PI;
        } else if (targetHeading < 0) {
            targetHeading += 2 * Math.PI;
        }

        return targetHeading;
    }

    public double headingToTurretPos(double heading) {
        return ((maxPosServos - minPosServos)
                * (heading - minimumValueRad)
                / (maximumValueRad - minimumValueRad))
                + minPosServos;
    }

    private void updateSafeTarget(double newTarget) {
        boolean currentNearMin = currentServoPosition < wrapLow;
        boolean currentNearMax = currentServoPosition > wrapHigh;

        boolean targetNearMin = newTarget < wrapLow;
        boolean targetNearMax = newTarget > wrapHigh;

        boolean unsafeCross =
                (currentNearMin && targetNearMax)
                        || (currentNearMax && targetNearMin);

        finalTargetPosition = newTarget;

        if (unsafeCross) {
            routingThroughMiddle = true;
        }
    }

    private double moveToward(double current, double target, double maxStep) {
        double error = target - current;

        if (Math.abs(error) <= maxStep) {
            return target;
        }

        return current + Math.signum(error) * maxStep;
    }

    private double clamp(double position) {
        return Math.max(minPosServos, Math.min(maxPosServos, position));
    }

    public double getServoPosition() {
        return currentServoPosition;
    }

    public void setPosition(double position) {
        position = clamp(position);

        updateSafeTarget(position);

        double activeTarget = routingThroughMiddle
                ? safeMiddle
                : finalTargetPosition;

        currentServoPosition = moveToward(
                currentServoPosition,
                activeTarget,
                maxStepPerLoop
        );

        if (routingThroughMiddle
                && Math.abs(currentServoPosition - safeMiddle) < 0.02) {
            routingThroughMiddle = false;
        }

        turretServos.set(currentServoPosition);
    }
}