// Turret.java
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.farRedGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxTurretPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.minTurretPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

@Configurable
public class Turret extends SubsystemBase {
    public final MotorEx turret;
    private final AprilTagTracking vision;

    public boolean autoAimEnabled = false;
    private boolean positionControlEnabled = false;

    private static final double MIN_TURRET_RAD = Math.toRadians(-135);
    private static final double MAX_TURRET_RAD = Math.toRadians(135);
    private static final double INCHES_TO_METERS = 0.0254;

    public static double turretForwardOffsetInches = -2;
    public static double turretLeftOffsetInches = 0.0;
    public static double turretZeroOffsetDeg = 0.0;
    public static double turretTicksPerDegree = 3.75;
    public static double turretEncoderDirection = 1.0;
    public static double maxVelocityLeadDeg = 40.0;

    public static double kP = 0.009;
    public static double kI = 0.08;
    public static double kD = 0.0004;
    public static double kF = 0.0001;
    public boolean isAutoCode = false;
    // PIDF (tune these)
    public static PIDFController turretPID = new PIDFController(
            kP, kI, kD, kF
    );

    private static final int TICKS_TOLERANCE = 2;
    private double maxPower = 1;
    private double desiredRelHeadingRad = 0.0;
    private double chosenRelHeadingRad = 0.0;
    private boolean targetClipped = false;
    private final AimDebug aimDebug = new AimDebug();

    public static int targetTicks = 168;

    public static class AimDebug {
        public String alliance = "";
        public String goalName = "";
        public double redDistanceIn;
        public double robotX;
        public double robotY;
        public double robotHeadingDeg;
        public double turretForwardOffsetIn;
        public double turretLeftOffsetIn;
        public double turretZeroOffsetDeg;
        public double turretTicksPerDegree;
        public double turretEncoderDirection;
        public double turretOriginX;
        public double turretOriginY;
        public double goalX;
        public double goalY;
        public double goalBearingDeg;
        public double velocityLeadDeg;
        public double targetAbsHeadingDeg;
        public double relToTargetRawDeg;
        public double relToTargetNormDeg;
        public double chosenRelDeg;
        public double currentTurretTicks;
        public double currentTurretRelDeg;
        public double currentTurretAbsDeg;
        public double targetTicksDouble;
        public int targetTicksInt;
        public double targetTicksFromMapping;
        public double expectedPhysicalRelDeg;
        public boolean clipped;
        public double pidPower;
    }

    public Turret(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        turret = new MotorEx(hardwareMap, "turret");
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setInverted(true);

        vision = new AprilTagTracking(hardwareMap);
    }

    public void startVision() {
        vision.start();
    }

    public void straight() {
        setTargetTicks(0);
    }

    public boolean isStraight() {
        return (turret.getCurrentPosition() > -1 && turret.getCurrentPosition() < 1);
    }

    public void setTargetTicks(int ticks) {
        targetTicks = (int) Range.clip(ticks, minTurretPos, maxTurretPos);
        positionControlEnabled = true;
    }

    public void resetTurretEncoder() {
        turret.stopAndResetEncoder();
        targetTicks = 0;
        turretPID.reset();
        positionControlEnabled = false;
    }

    public double getPos() {
        return turret.getCurrentPosition();
    }

    public void setAutoAim(boolean enabled) {
        boolean wasEnabled = autoAimEnabled;
        autoAimEnabled = enabled;
        if (enabled) {
            if (!wasEnabled) {
                turretPID.reset();
            }
            positionControlEnabled = true;
        } else {
            turret.set(0);
            turretPID.reset();
            positionControlEnabled = false;
        }
    }

    public double getTargetTicksFromPos(Pose pos) {
        return updateAimTargetFromPose(pos, 0.0);
    }

    public void setTurretPos(double ticks) {
        setTargetTicks((int) Math.round(ticks));
    }

    @Override
    public void periodic() {
        turretPID.setPIDF(kP, kI, kD, kF);

        if (autoAimEnabled) {
            updateAimTargetFromPose(Localization.getPose(), getVelocityLeadRad());
            positionControlEnabled = true;
        }

        if (!autoAimEnabled && !positionControlEnabled) {
            turret.set(0);
            return;
        }

        // PIDF to targetTicks
        turretPID.setSetPoint(targetTicks);

        double current = turret.getCurrentPosition();
        double power = turretPID.calculate(current);
        power = Range.clip(power, -maxPower, maxPower);
        aimDebug.pidPower = power;

//        double err = targetTicks - current;
//        if (Math.abs(err) > TICKS_TOLERANCE && Math.abs(power) < 0.2) {
//            power = Math.copySign(0.2, power);
//        }
//
//
        turret.set(power);
    }

    private double getVelocityLeadRad() {
        double goalDistanceInches = Localization.getGoalDistance(chosenAlliance);
        double[] shooterCoefficients = Shooter.getCoefficientsFromDistance(goalDistanceInches);

        double hoodPos = shooterCoefficients[0];
        double hoodAngleRad = Math.toRadians(Shooter.getHoodAngleFromPos(hoodPos));
        double measuredShotTicksPerSec = Math.abs(0.5 * (Shooter.velocity1 - Shooter.velocity2));
        double shooterTicksPerSec = measuredShotTicksPerSec > 1.0
                ? measuredShotTicksPerSec
                : Math.max(Shooter.targetVelocity, shooterCoefficients[1]);
        double shooterSpeedMps = Shooter.getShooterSpeedFromTicks(shooterTicksPerSec);

        Pose robotPose = Localization.getPose();
        Pose goalPose = getGoalPose(chosenAlliance, robotPose);
        Pose turretOrigin = getTurretOriginPose(robotPose);

        Vector robotToGoal = goalPose
                .minus(turretOrigin)
                .getAsVector();
        Vector robotVelocity = Localization.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoal.getTheta();
        double robotSpeedMps = robotVelocity.getMagnitude() * INCHES_TO_METERS;
        double parallelComponent = -Math.cos(coordinateTheta) * robotSpeedMps;
        double perpendicularComponent = Math.sin(coordinateTheta) * robotSpeedMps;

        double horizontalLaunchSpeedMps = shooterSpeedMps * Math.cos(hoodAngleRad);
        double lateralDemandSq = perpendicularComponent * perpendicularComponent;
        double horizontalSpeedSq = horizontalLaunchSpeedMps * horizontalLaunchSpeedMps;
        if (horizontalSpeedSq <= lateralDemandSq + 1e-6) {
            return 0.0;
        }

        // The compensated shooter solution already bakes chassis parallel velocity into the
        // required horizontal launch magnitude. Recover the along-goal component from that
        // magnitude instead of adding parallel velocity a second time.
        double launcherParallelComponent = Math.sqrt(horizontalSpeedSq - lateralDemandSq);

        // Sign is inverted so turret leads in the physical/camera frame correctly.
        double leadRad = -Math.atan2(perpendicularComponent, launcherParallelComponent);
        double maxLeadRad = Math.toRadians(maxVelocityLeadDeg);
        return Range.clip(leadRad, -maxLeadRad, maxLeadRad);
    }

    public boolean isAimed() {
        return Math.abs(targetTicks - turret.getCurrentPosition()) <= TICKS_TOLERANCE;
    }

    public double getDesiredRelHeadingDeg() {
        return Math.toDegrees(desiredRelHeadingRad);
    }

    public double getChosenRelHeadingDeg() {
        return Math.toDegrees(chosenRelHeadingRad);
    }

    public boolean isTargetClipped() {
        return targetClipped;
    }

    public AimDebug getAimDebug() {
        return aimDebug;
    }

    private double posToHeading(double posTicks) {
        double pos = Range.clip(posTicks, minTurretPos, maxTurretPos);
        double ticksPerDeg = Math.abs(turretTicksPerDegree) < 1e-6
                ? maxTurretPos / 135.0
                : turretTicksPerDegree;
        double direction = Math.signum(turretEncoderDirection);
        if (direction == 0.0) {
            direction = 1.0;
        }
        return Math.toRadians(turretZeroOffsetDeg + direction * pos / ticksPerDeg);
    }

    private double headingToPos(double headingRad) {
        double h = Range.clip(headingRad, MIN_TURRET_RAD, MAX_TURRET_RAD);
        double ticksPerDeg = Math.abs(turretTicksPerDegree) < 1e-6
                ? maxTurretPos / 135.0
                : turretTicksPerDegree;
        double direction = Math.signum(turretEncoderDirection);
        if (direction == 0.0) {
            direction = 1.0;
        }
        return direction * (Math.toDegrees(h) - turretZeroOffsetDeg) * ticksPerDeg;
    }


    private double chooseTurretRelHeading(double relRad, double currentTurretRelRad) {
        double base = normalizeRadians(relRad);

        double[] candidates = new double[] { base, base + 2.0 * Math.PI, base - 2.0 * Math.PI };

        double bestInRange = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        for (double c : candidates) {
            if (c >= MIN_TURRET_RAD && c <= MAX_TURRET_RAD) {
                double d = Math.abs(c - currentTurretRelRad);
                if (d < bestDist) {
                    bestDist = d;
                    bestInRange = c;
                }
            }
        }
        if (!Double.isNaN(bestInRange)) return bestInRange;

        double bestClamped = 0.0;
        bestDist = Double.POSITIVE_INFINITY;

        for (double c : candidates) {
            double clamped = Range.clip(c, MIN_TURRET_RAD, MAX_TURRET_RAD);
            double d = Math.abs(clamped - currentTurretRelRad);
            if (d < bestDist) {
                bestDist = d;
                bestClamped = clamped;
            }
        }
        return bestClamped;
    }

    private void recordAimDebug(double relToTarget, double chosenRel) {
        desiredRelHeadingRad = normalizeRadians(relToTarget);
        chosenRelHeadingRad = chosenRel;
        targetClipped = Math.abs(normalizeRadians(chosenRel - desiredRelHeadingRad)) > Math.toRadians(1.0);
    }

    private Pose getTurretOriginPose(Pose robotPose) {
        double heading = robotPose.getHeading();
        double x = robotPose.getX()
                + turretForwardOffsetInches * Math.cos(heading)
                + turretLeftOffsetInches * Math.cos(heading + Math.PI / 2.0);
        double y = robotPose.getY()
                + turretForwardOffsetInches * Math.sin(heading)
                + turretLeftOffsetInches * Math.sin(heading + Math.PI / 2.0);
        return new Pose(x, y, heading);
    }

    private Pose getGoalPose(String alliance, Pose robotPose) {
        if ("BLUE".equals(alliance)) {
            return blueGoalPose;
        }
        return getRedDistance(robotPose) > 100.0 ? farRedGoalPose : redGoalPose;
    }

    private String getGoalName(String alliance, Pose robotPose) {
        if ("BLUE".equals(alliance)) {
            return "blue";
        }
        return getRedDistance(robotPose) > 100.0 ? "farRed" : "red";
    }

    private double getGoalBearing(Pose turretOrigin, String alliance, Pose robotPose) {
        return getGoalPose(alliance, robotPose).minus(turretOrigin).getAsVector().getTheta();
    }

    private int updateAimTargetFromPose(Pose robotPose, double velocityLeadRad) {
        double robotHeading = robotPose.getHeading();
        double robotHeadingPred = normalizeRadians(robotHeading);
        double currentTicks = getPos();
        double turretRelHeading = posToHeading(currentTicks);
        Pose turretOrigin = getTurretOriginPose(robotPose);
        Pose goalPose = getGoalPose(chosenAlliance, robotPose);
        double goalBearing = goalPose.minus(turretOrigin).getAsVector().getTheta();
        double targetAbsHeading = normalizeRadians(goalBearing + velocityLeadRad);
        double relToTarget = targetAbsHeading - robotHeadingPred;
        double chosenRel = chooseTurretRelHeading(relToTarget, turretRelHeading);
        double targetTicksRaw = headingToPos(chosenRel);
        int targetTicksNew = (int) Math.round(Range.clip(targetTicksRaw, minTurretPos, maxTurretPos));

        recordAimDebug(relToTarget, chosenRel);
        updateAimDebug(
                robotPose,
                turretOrigin,
                goalPose,
                goalBearing,
                velocityLeadRad,
                targetAbsHeading,
                relToTarget,
                chosenRel,
                currentTicks,
                turretRelHeading,
                targetTicksRaw,
                targetTicksNew
        );

        targetTicks = targetTicksNew;
        return targetTicks;
    }

    private void updateAimDebug(Pose robotPose, Pose turretOrigin, Pose goalPose, double goalBearing,
                                double velocityLeadRad, double targetAbsHeading, double relToTarget,
                                double chosenRel, double currentTicks, double currentTurretRel,
                                double targetTicksRaw, int targetTicksNew) {
        aimDebug.alliance = chosenAlliance;
        aimDebug.goalName = getGoalName(chosenAlliance, robotPose);
        aimDebug.redDistanceIn = getRedDistance(robotPose);
        aimDebug.robotX = robotPose.getX();
        aimDebug.robotY = robotPose.getY();
        aimDebug.robotHeadingDeg = Math.toDegrees(normalizeRadians(robotPose.getHeading()));
        aimDebug.turretForwardOffsetIn = turretForwardOffsetInches;
        aimDebug.turretLeftOffsetIn = turretLeftOffsetInches;
        aimDebug.turretZeroOffsetDeg = turretZeroOffsetDeg;
        aimDebug.turretTicksPerDegree = turretTicksPerDegree;
        aimDebug.turretEncoderDirection = turretEncoderDirection;
        aimDebug.turretOriginX = turretOrigin.getX();
        aimDebug.turretOriginY = turretOrigin.getY();
        aimDebug.goalX = goalPose.getX();
        aimDebug.goalY = goalPose.getY();
        aimDebug.goalBearingDeg = Math.toDegrees(normalizeRadians(goalBearing));
        aimDebug.velocityLeadDeg = Math.toDegrees(velocityLeadRad);
        aimDebug.targetAbsHeadingDeg = Math.toDegrees(targetAbsHeading);
        aimDebug.relToTargetRawDeg = Math.toDegrees(relToTarget);
        aimDebug.relToTargetNormDeg = Math.toDegrees(normalizeRadians(relToTarget));
        aimDebug.chosenRelDeg = Math.toDegrees(chosenRel);
        aimDebug.currentTurretTicks = currentTicks;
        aimDebug.currentTurretRelDeg = Math.toDegrees(currentTurretRel);
        aimDebug.currentTurretAbsDeg = Math.toDegrees(normalizeRadians(robotPose.getHeading() + currentTurretRel));
        aimDebug.targetTicksDouble = targetTicksRaw;
        aimDebug.targetTicksInt = targetTicksNew;
        aimDebug.targetTicksFromMapping = headingToPos(chosenRel);
        aimDebug.expectedPhysicalRelDeg = turretZeroOffsetDeg + Math.signum(turretEncoderDirection == 0.0 ? 1.0 : turretEncoderDirection)
                * targetTicksNew / (Math.abs(turretTicksPerDegree) < 1e-6 ? maxTurretPos / 135.0 : turretTicksPerDegree);
        aimDebug.clipped = targetClipped;
    }
}
