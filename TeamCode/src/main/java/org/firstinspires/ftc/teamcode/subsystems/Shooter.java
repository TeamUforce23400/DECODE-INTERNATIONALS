package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getVelocity;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxEPT;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxHoodAngle;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxHoodPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minHoodAngle;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minHoodPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redDistancePose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.Iterator;

import fi.iki.elonen.NanoHTTPD;

public class Shooter extends SubsystemBase {

    private TelemetryManager telemetry;
    private boolean autoShoot = false;
    private double pos = 0.5;
//    private static double flywheelOffset = 400;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    private ServoEx hood;
    public static double landAngle = Math.toRadians(-30);
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;
    // 2.5in ring radius; 5in was ring diameter and caused a systematic short-range bias.
    private static double passThroughPointRadius = 0.0635;
    // Extra horizontal correction in inches (positive = shoot farther).
    public static double shooterDistanceBiasInches = 0;
    // Far-shot correction for drag/spin/slip not captured by ideal projectile equations.
    public static double farCompStartInches = 100.0;
    public static double powerConstant = 2.42;
    public static double farRangeCompInches = 10.0;
    // Blend long shots toward a lower/direct trajectory.
    public static double farLowAngleStartInches = 110.0;
    public static double farLowAngleBlendInches = 25.0;
    public static double farLowAngleTargetDeg = 35.0;
    // Hold flywheel speed with feedforward + small trim instead of 0/1 bang-bang power.
    public static double shooterPowerKp = 0.00045;
    private static boolean isAuto;

    private static InterpLUT distSpeed = new InterpLUT();
    private static InterpLUT distAngle = new InterpLUT();

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = new ServoEx(hardwareMap, "hood");
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        sh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sh2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        I = 0.2;
        P = 1.3;
        kS = 0.06;
        kV = 0.00039;
//        distSpeed.add(15.48, 1000);
//        distSpeed.add(35.20, 1125);
//        distSpeed.add(45.55, 1200);
//        distSpeed.add(56.70, 1275);
//        distSpeed.add(72.68, 1350);
//
//        distAngle.add(15.48, 0.65);
//        distAngle.add(35.20, 0.45);
//        distAngle.add(45.55, 0.4);
//        distAngle.add(56.70, 0.4);
//        distAngle.add(72.68, 0.4);
////        distSpeed.add(15.48, 1000);
//        distSpeed.createLUT();
//        distAngle.createLUT();

    }

    @Override
    public void periodic() {
        velocity1 = sh.getVelocity();
        velocity2 = sh2.getVelocity();
        double actualShotSpeed = velocity1;
        if (autoShoot) {
            double shotDistance = getGoalDistance(chosenAlliance);
            double[] coefficients = Shooter.getCoefficientsFromDistance(shotDistance);
            if (shotDistance > 100) {
                Shooter.landAngle = Math.toRadians(-20);
                Shooter.powerConstant = 2.3;
            }
            else {
                Shooter.landAngle = Math.toRadians(-10);
                Shooter.powerConstant = 2.42;
            }
            targetVelocity = coefficients[1];
            if (Math.abs(actualShotSpeed - targetVelocity) > 40 && (shotDistance < 100)) {
                pos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, actualShotSpeed);
            } else {
                pos = coefficients[0];
            }
            setHood(pos);
        }

//        telemetry.addData("TargetVel", targetVelocity);
//        telemetry.addData("Difference", actualShotSpeed - targetVelocity);
//        telemetry.addData("New hood", (Math.abs(actualShotSpeed - targetVelocity) > 40.0));
//        telemetry.addData("CurrentVel1", velocity1);
//        telemetry.addData("CurrentVel2", velocity2);

        double shooterPower = 0.0;
//        if (targetVelocity - actualShotSpeed > 50) {
//            shooterPower = 1;
//        }
//        else if (targetVelocity > 1.0) {
//            shooterPower = Range.clip(
//                    kS + (kV * targetVelocity) + (shooterPowerKp * (targetVelocity - actualShotSpeed)),
//                    0.0,
//                    1.0
//            );
//        }
        if (actualShotSpeed < targetVelocity - 10) {
            shooterPower = 1;
        }
        else {
            shooterPower = 0;
        }
        sh.setPower(shooterPower);
        sh2.setPower(shooterPower);

//        telemetry.addData("ShooterPower", shooterPower);
//        telemetry.update();
    }

    public boolean atSpeed() {
        targetVelocity = getCoefficientsFromDistance(getGoalDistance(chosenAlliance))[1];
        double actualShotSpeed = Math.abs(0.5 * (sh.getVelocity() - sh2.getVelocity()));
        return Math.abs(actualShotSpeed - targetVelocity) < 30;
    }

    public void setAutoShoot(boolean on) {
        autoShoot = on;
    }

    public void setTargetEPT(double ept) {
        targetVelocity = ept;
    }

    public static double[] getFarSideCoefficients(double d) {
        return new double[]{MathUtils.clamp(0.00235*d+0.34585, Math.min(minHoodPos, maxHoodPos), Math.max(minHoodPos, maxHoodPos)), 4*d+1074};
    }


    public void setHood(double pos) {
        hood.set(Range.clip(
                pos,
                Math.min(minHoodPos, maxHoodPos),
                Math.max(minHoodPos, maxHoodPos)
        ));
    }

    public double getHoodPos() {
        return hood.get();
    }

    public int getVelA() { return (int) sh.getVelocity(); }
    public int getVelB() { return (int) sh2.getVelocity(); }


    public static double speedFromDistance(double d) {
//        if (d > 0 && d < 47.8) {
//            return (int) 1.41862*d+1041.18911;
//        }
//        else if (d>=47.8 && d < 70) {
//            return (int) 7.70289*d+740.79915;
//        }
//        else if (d>=70) {
//            return (int) (0.00122932*(Math.pow(d, 3)))-(0.318188*(Math.pow(d, 2)))+(30.10425*d+403.64484);
//        }
        if (d>72.68) {
            return distSpeed.get(72.67);
        }
        else if (d<15.48) {
            return distSpeed.get(15.5);
        }
        else { return distSpeed.get(d); }
    }

    public static double angleFromDistance(double d) {
//        if (isAuto) {
//            if (d > 0 && d < 47.8) {
//                return 0.63;
//            }
//            else if (d>=47.8 && d < 70) {
//                return 0.57;
//            }
//            else if (d>=70) {
//                return 0.4;
//            }
//        }
//        else {
//            if (d > 0 && d < 47.8) {
//                return 0.63;
//            }
//            else if (d>=47.8 && d < 70) {
//                return 0.575;
//            }
//            else if (d>=70) {
//                return 0.4;
//            }
//        }
        if (d>72.68) {
            return distAngle.get(72.67);
        }
        else if (d<15.48) {
            return distAngle.get(15.5);
        }
        else { return distAngle.get(d); }
    }

    public static double getHoodPosFromAngle(double angle) {
        double slope = (minHoodPos - maxHoodPos) / (minHoodAngle - maxHoodAngle);
        return slope * (angle - maxHoodAngle) + maxHoodPos;
    }

    public static double getHoodAngleFromPos(double pos) {
        double slope = (minHoodAngle - maxHoodAngle) / (minHoodPos - maxHoodPos);
        return slope * (pos - maxHoodPos) + maxHoodAngle;
    }

        
    public static double getShooterTicksFromSpeed(double speed) {
        return (28*powerConstant*speed/(2*Math.PI*0.048));
    }

    public static double getShooterSpeedFromTicks(double ticksPerSecond) {
        return (ticksPerSecond * Math.PI * 0.048*2) / (28.0*powerConstant);
    }

    public static double getLowAngleHoodFromDistanceAndSpeed(double distanceInches, double actualTicksPerSecond) {
        double g = 9.81;
        double x = ((distanceInches + shooterDistanceBiasInches) * 0.0254) - passThroughPointRadius;
        if (distanceInches >= farCompStartInches) {
            x += farRangeCompInches * 0.0254;
        }
        double y = 0.5842;
        double baselineHoodPos = getCoefficientsFromDistance(distanceInches)[0];
        double ballSpeed = getShooterSpeedFromTicks(actualTicksPerSecond);

        if (x <= 0 || actualTicksPerSecond <= 0 || ballSpeed <= 0) {
            return baselineHoodPos;
        }

        double discriminant = Math.pow(ballSpeed, 4) - g * (g * x * x + 2 * y * ballSpeed * ballSpeed);
        if (discriminant < 0) {
            return baselineHoodPos;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double denominator = g * x;
        if (Math.abs(denominator) <= 1e-6) {
            return baselineHoodPos;
        }

        double tanThetaLow = (ballSpeed * ballSpeed - sqrtDiscriminant) / denominator;
        double hoodAngle = Math.atan(tanThetaLow);
        if (Double.isNaN(hoodAngle) || Double.isInfinite(hoodAngle)) {
            return baselineHoodPos;
        }

        double minHoodAngleRad = Math.toRadians(Math.min(maxHoodAngle, minHoodAngle));
        double maxHoodAngleRad = Math.toRadians(Math.max(maxHoodAngle, minHoodAngle));
        hoodAngle = MathUtils.clamp(hoodAngle, minHoodAngleRad, maxHoodAngleRad);

        return Range.clip(
                getHoodPosFromAngle(Math.toDegrees(hoodAngle)),
                Math.min(minHoodPos, maxHoodPos),
                Math.max(minHoodPos, maxHoodPos)
        );
    }

    public static double[] getCoefficientsFromDistance(double d) {
        double g = 9.81;
        double x = ((d + shooterDistanceBiasInches) * 0.0254) - passThroughPointRadius;
        if (d >= farCompStartInches) {
            x += farRangeCompInches * 0.0254;
        }
        double y = 0.5842;
        double a = landAngle;

//        if (d >= farCompStartInches) {
//            a = Math.toRadians(-20);
//        }

        double minHoodAngleRad = Math.toRadians(Math.min(maxHoodAngle, minHoodAngle));
        double maxHoodAngleRad = Math.toRadians(Math.max(maxHoodAngle, minHoodAngle));

        if (x <= 0) {
            double hoodPos = Range.clip(
                    getHoodPosFromAngle(Math.toDegrees(maxHoodAngleRad)),
                    Math.min(minHoodPos, maxHoodPos),
                    Math.max(minHoodPos, maxHoodPos)
            );
            return new double[]{hoodPos, 0};
        }

        double hoodAngle = MathUtils.clamp(Math.atan(2 * y / x - Math.tan(a)), minHoodAngleRad, maxHoodAngleRad);
//        if (d >= farLowAngleStartInches) {
//            double blend = Range.clip(
//                    (d - farLowAngleStartInches) / Math.max(1e-6, farLowAngleBlendInches),
//                    0.0,
//                    1.0
//            );
//            double targetFarAngleRad = Math.toRadians(farLowAngleTargetDeg);
//            hoodAngle += (targetFarAngleRad - hoodAngle) * blend;
//        }
        hoodAngle = MathUtils.clamp(
                hoodAngle,
                minHoodAngleRad,
                maxHoodAngleRad
        );

        double denominator = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);
        if (denominator <= 1e-6) {
            hoodAngle = maxHoodAngleRad;
            denominator = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);
        }
        if (denominator <= 1e-6) {
            double hoodPos = Range.clip(
                    getHoodPosFromAngle(Math.toDegrees(hoodAngle)),
                    Math.min(minHoodPos, maxHoodPos),
                    Math.max(minHoodPos, maxHoodPos)
            );
            return new double[]{hoodPos, 0};
        }

        double ballSpeed = Math.sqrt(g * x * x / denominator);
        if (Double.isNaN(ballSpeed) || Double.isInfinite(ballSpeed)) {
            ballSpeed = 0;
        }

        Vector robotToGoal = (chosenAlliance.equals("BLUE")
                ? RobotConstants.blueGoalPose
                : RobotConstants.redGoalPose)
                .minus(getPose())
                .getAsVector();


        Vector robotVelocity = getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoal.getTheta();

        // follower velocity is in in/s; convert to m/s to match ballistic units.
        double robotSpeedMps = robotVelocity.getMagnitude() * 0.0254;
        double parallelComponent = -Math.cos(coordinateTheta) * robotSpeedMps;
        double perpendicularComponent = Math.sin(coordinateTheta) * robotSpeedMps;

        double vz = Math.sin(hoodAngle) * ballSpeed;
        double time = x / (ballSpeed * Math.cos(hoodAngle));
        double vxc = x / time + parallelComponent;
        double vxn = Math.sqrt(vxc * vxc + perpendicularComponent * perpendicularComponent);
        double nx = vxn * time;

        hoodAngle = MathUtils.clamp(Math.atan((vz/vxn)), minHoodAngleRad, maxHoodAngleRad);
        ballSpeed = Math.sqrt(g*nx*nx/(2*Math.pow(Math.cos(hoodAngle), 2) * (nx * Math.tan(hoodAngle) - y)));


//        if (perpendicularComponent > 0) {
//            hoodAngle = getLowAngleHoodFromDistanceAndSpeed(nx, getShooterTicksFromSpeed(ballSpeed));
//        }

        double hoodPos = MathUtils.clamp(
                getHoodPosFromAngle(Math.toDegrees(hoodAngle)),
                Math.min(minHoodPos, maxHoodPos),
                Math.max(minHoodPos, maxHoodPos)
        );

//        if (d > 125) {
//            return getFarSideCoefficients(d);
//        }
//        else {
            return new double[]{hoodPos, Range.clip(getShooterTicksFromSpeed(ballSpeed), 0, 1900)};
//        }
    }
}
