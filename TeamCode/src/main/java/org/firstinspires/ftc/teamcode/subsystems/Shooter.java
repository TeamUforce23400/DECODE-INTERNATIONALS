package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;
import static org.firstinspires.ftc.teamcode.globals.Localization.getVelocity;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Configurable
public class Shooter extends SubsystemBase {

    public final DcMotorEx shooterRight;
    public final DcMotorEx shooterLeft;
    public final ServoEx hood;

    public static boolean isFarside = true;
    public static double farVelocity = 1200;

    public static double landAngleDegrees = -20;

    public double landAngle = Math.toRadians(landAngleDegrees);
    public final double encoderResolution = 28.0; // TODO: This is the shooter motors' ticks resolution (on the goBILDA site). For 6000 rpm, it is 28.0;
    public final double motorToFlywheelRatio = 1.0; // TODO: This is the gear ratio from the shooter motors to the flywheel. flywheelTeeth/motorTeeth
    public final double verticalDifference = 0.5; // TODO: This should be your vertical distance from the point where the balls (middle of shooter) are launched to the height of the goal (top of white part of the April Tag).
    public static double powerConstant = 2.37; // TODO: Tune this until the balls go in from most distances. Ideal range is 2-2.5. (A lower power constant means that the energy loss is less when shooting)
    public double shooterDistanceBias = 0; // In meters; best to keep at 0; you should only use if there is a genuine bias and not a power constants issue.

    // The variable names are kind of unintuitive here, but it should be fine as long as you set the constants properly.
    // IMPORTANT: A lower hood angle means that the hood is less curved (much higher shots - more curved trajectory). A higher hood angle means that the hood is more curved (lower shots - flatter trajectory).
    // IMPORTANT: The hood angle degrees is going to be measured from the horizontal, NOT vertical.
    // You can either do this manually using a protractor and measuring the exact moment the ball is launched (should be approx. center of the ball).
    // OR you can check the CAD and do this (with the CAD measurement method, you need to make sure that the hood is set to the same position as the servo position in real life).

    public double minimumHoodPos = 0.07665; // TODO: This will be your minimum hood servo position, at minimum hood angle degree (for example, servo position 0 at launch angle 80 degrees - lower hood angle, higher shots, higher trajectory).
    public double maximumHoodPos = 0.875; // TODO: This will be your maximum hood servo position, at maximum hood angle degree (for example, servo position 1 at launch angle 20 degrees - higher hood angle, lower shots, flatter trajectory).
    // Change the values inside the brackets in degrees; it will be converted to radians.
    public double minHoodPosRad = Math.toRadians(57.452); // TODO: This should be the launch angle of the ball when the hood servo is at minimum hood position (higher shots, higher trajectory).
    public double maxHoodPosRad = Math.toRadians(27.952); // TODO: This should be the launch angle of the ball when the hood servo is at maximum hood position (more curved path/direct shots, flatter trajectory).

    public PIDFController controllerRight;
    public PIDFController controllerLeft;
    public final double P = 0.03;
    public final double I = 0.2;
    public final double kV = 0.0004;
    public final double kS = 0.065;
    public Shooter(HardwareMap hardwareMap) {
        // TODO: The directions are relative to when the shooter is facing away from you - towards the front of the field/goal.
        shooterRight = hardwareMap.get(DcMotorEx.class, "st");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "sb");
        hood = new ServoEx(hardwareMap, "hood");

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controllerRight = new PIDFController(P,I,0.0, 0);
        controllerLeft = new PIDFController(P,I,0.0, 0);
        controllerRight.setFeedforward(kV, 0.0, kS);
        controllerLeft.setFeedforward(kV, 0.0, kS);
        // TODO: Make sure both the motors tick together (both should have positive encoder ticks together). If not, then use .setDirection() method.
        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        // TODO: Hood servo position must increase as the hood curve increases (angle decreases, flatter trajectory); for example, 0-->1, 80deg --> 20deg.
        hood.setInverted(true);
    }

    @Override
    public void periodic() {
        Pose robotPos = Localization.getPose();
        Pose goalPose = chosenAlliance.equals("RED") ? redGoalPose : blueGoalPose;

        double shotDistance = robotPos.distanceFrom(goalPose) * 0.0254;
        double actualVelocity = shooterRight.getVelocity();

        double[] coefficients = getCoefficientsFromDistance(shotDistance);

        double targetVelocity = getTicksFromBallSpeed(coefficients[0]);
        double hoodPos = getHoodPosFromAngle(coefficients[1]);

        // Velocity compensation
        if (Math.abs(targetVelocity - actualVelocity) > 30) {
            hoodPos = getHoodPosFromAngle(getCompensatedHoodAngle(shotDistance, actualVelocity));
        }

        hood.set(hoodPos);

        targetVelocity = manualFar(targetVelocity);

        double errorRight = targetVelocity - shooterRight.getVelocity();
        double errorLeft = targetVelocity - shooterLeft.getVelocity();
        shooterRight.setPower(controllerRight.calculate(errorRight, targetVelocity, 0.0));
        shooterLeft.setPower(controllerLeft.calculate(errorLeft, targetVelocity, 0.0));
    }

    public double manualFar( double targetVel){
        isFarside=!isFarside;
        if (isFarside){
            targetVel = MathUtils.clamp(
                    targetVel,
                    farVelocity,
                    farVelocity+400
            );
            return targetVel;
        }
        else{
            targetVel = MathUtils.clamp(
                    targetVel,
                    0,
                    targetVel+500
            );
            return targetVel;
        }

    }

    public double getHoodPosFromAngle(double angle) {
        return ((maximumHoodPos - minimumHoodPos)*(angle - minHoodPosRad)/(maxHoodPosRad-minHoodPosRad)) + minimumHoodPos;
    }

    public double getAngleFromHoodPos(double pos) {
        return ((maxHoodPosRad - minHoodPosRad)*(pos-minimumHoodPos)/(maximumHoodPos-minimumHoodPos)) + minHoodPosRad;
    }

    public double getTicksFromBallSpeed(double speed) {
        return (encoderResolution * motorToFlywheelRatio * powerConstant * speed/(2 * Math.PI * 0.036));
    }

    public double getBallSpeedFromTicks(double ticksPerSecond) {
        return (ticksPerSecond * Math.PI * 0.036 * 2) / (encoderResolution * powerConstant * motorToFlywheelRatio);
    }

    public boolean atSpeed() {
        Pose robotPos = Localization.getPose();
        Pose goalPose = chosenAlliance.equals("RED") ? redGoalPose : blueGoalPose;

        double shotDistance = robotPos.distanceFrom(goalPose) * 0.0254;

        double[] coefficients = getCoefficientsFromDistance(shotDistance);
        double targetVelocity = getTicksFromBallSpeed(coefficients[0]);

        double actualRightVelocity = shooterRight.getVelocity();

        return Math.abs(actualRightVelocity - targetVelocity) < 20;

    }





    public double[] getCoefficientsFromDistance(double distance) {
        // IMPORTANT: All units are meters, but pedroPathing methods return inches, so appropriate conversion HAS ALREADY been done.
        double g = 9.81;
        double x = distance + shooterDistanceBias;
        double y = verticalDifference;
        double a = landAngle; // The angle at which you want your balls to land. Higher land angle means the balls take a higher curve, and lower land angle means balls are more direct. A good land angle should work at most distances while the balls should not bounce out.

        double hoodAngle = Math.atan(2 * y / x - Math.tan(a));

        hoodAngle = MathUtils.clamp(
                hoodAngle,
                maxHoodPosRad,
                minHoodPosRad
        );

        double denominator = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);
        double ballSpeed = Math.sqrt(g * x * x / denominator);
        if (Double.isNaN(ballSpeed) || Double.isInfinite(ballSpeed)) {
            ballSpeed = 0;
        }


        // You can comment this next part out, and it WON'T affect the normal calculations (of course, shooting while moving won't work if you comment it out).
        // START: Shooting while moving
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

        hoodAngle = MathUtils.clamp(Math.atan((vz/vxn)), maxHoodPosRad, minHoodPosRad);
        ballSpeed = Math.sqrt(g*nx*nx/(2*Math.pow(Math.cos(hoodAngle), 2) * (nx * Math.tan(hoodAngle) - y)));

        if (Double.isNaN(ballSpeed) || Double.isInfinite(ballSpeed)) {
            ballSpeed = 0;
        }
        if (Double.isNaN(hoodAngle) || Double.isInfinite(hoodAngle)) {
            hoodAngle = minHoodPosRad;
        }
        // END: Shooting while moving

        return new double[] {ballSpeed, hoodAngle};
    }

    public double getCompensatedHoodAngle(double distance, double actualVelocity) {
        double g = 9.81;
        double x = distance + shooterDistanceBias;
        double y = verticalDifference;
        double baselineHoodAngle = getCoefficientsFromDistance(distance)[1];
        double ballSpeed = getBallSpeedFromTicks(actualVelocity);

        if (x <= 0 || actualVelocity <= 0 || ballSpeed <= 0) {
            return baselineHoodAngle;
        }

        double discriminant = Math.pow(ballSpeed, 4) - g * (g * x * x + 2 * y * ballSpeed * ballSpeed);

        if (discriminant < 0) {
            return baselineHoodAngle;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double denominator = g * x;
        if (Math.abs(denominator) <= 1e-6) {
            return baselineHoodAngle;
        }

        double tanThetaLow = (ballSpeed * ballSpeed - sqrtDiscriminant) / denominator;
        double hoodAngle = Math.atan(tanThetaLow);
        if (Double.isNaN(hoodAngle) || Double.isInfinite(hoodAngle)) {
            return baselineHoodAngle;
        }

        hoodAngle = MathUtils.clamp(hoodAngle, maxHoodPosRad, minHoodPosRad);

        return hoodAngle;

    }
}
