package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.disengagePos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.engagePos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class Intake extends SubsystemBase {
    private static final double AUTO_STOP_CURRENT_AMPS = 5.0;
    private static final double DEFAULT_INTAKE_POWER = 1.0;
    private final Timer timer = new Timer();
    private static final double SLOW_TRANSFER_POWER = 0.7;
    private static final double STOPPER_SHOOT_POS = 0.35;
    private static final double STOPPER_HOLD_POS = 0.48;

    private final AnalogInput s1;
    private final AnalogInput s2;
    private final AnalogInput s3;

    public final DcMotorEx intakeMotor;
    private final ServoEx stopper;

    private final ServoEx pto;

    private final TelemetryManager telemetry;

    private double current;
    private boolean autoEnabled = false;
    private boolean triggered = false;
    private boolean shooting = false;
    private boolean currentStopEnabled = false;

    private boolean all3 = false;

    public Intake(HardwareMap hardwareMap, TelemetryManager telemetryManager) {

        s1 = hardwareMap.get(AnalogInput.class, "s1");
        s2 = hardwareMap.get(AnalogInput.class, "s2");
        s3 = hardwareMap.get(AnalogInput.class, "s3");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        stopper = new ServoEx(hardwareMap, "stopper");
        pto = new ServoEx(hardwareMap, "pto");
        pto.setInverted(true);

        telemetry = telemetryManager;
    }

    @Override
    public void periodic() {
        current = intakeMotor.getCurrent(CurrentUnit.AMPS);

        if ((Math.abs(stopper.get() - 0.48) < 0.01) && isBallDetected03() && intakeMotor.isMotorEnabled()) {
            disengagePTO();
        }

        if (areAllBallsDetected()) {
            engagePTO();
        }


        if (shouldStopForCurrent()) {
            intakeOff();
            triggered = true;
            return;
        }

        if (shouldAutoRun()) {
                intakeMotor.setPower(1);
        }
    }

    private boolean shouldStopForCurrent() {
        return currentStopEnabled && !shooting && current > AUTO_STOP_CURRENT_AMPS && areAllBallsDetected();
    }

    private boolean shouldAutoRun() {
        return autoEnabled && !triggered && current <= AUTO_STOP_CURRENT_AMPS;
    }

    public void setStopper(double pos) {
        stopper.set(pos);
    }
    public void setPTO(double pos) {
        pto.set(pos);
    }

    public void engagePTO() {
        pto.set(engagePos);
    }

    public void disengagePTO() {
        pto.set(disengagePos);
    }

    public void onSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

    public double getStopper() {
        return stopper.get();
    }

    public void intakeOff() {
        intakeMotor.setPower(0.0);
    }

    public void intake1On() {
        intakeMotor.setPower(DEFAULT_INTAKE_POWER);
    }

    public void intakeOpposite() {
        intakeMotor.setPower(-1.0);
    }


    public boolean isBallDetected01() {
        return ((s1.getVoltage()*32.50930976)-2.695384202) < 3.5;
    }

    public boolean isBallDetected02() {
        return ((s2.getVoltage()*32.50930976)-2.695384202) < 3.5;
    }

    public boolean isBallDetected03() {
        return (((s3.getVoltage()*32.50930976)-2.695384202) < 3.5);
    }

    public boolean areAllBallsDetected() {
        return isBallDetected01() && isBallDetected02() && isBallDetected03();
    }

    public boolean noBalls() {
        return !isBallDetected01() && !isBallDetected02() && !isBallDetected03();
    }

    public boolean canIntake() {
        return !areAllBallsDetected();
    }

    public void testSensors() {
        boolean d1 = isBallDetected01();
        boolean d2 = isBallDetected02();
        boolean d3 = isBallDetected03();

//        telemetry.addData("Mode", "Ranger 15° FOV (Analog)");
//        telemetry.addData("Detect higher V?", DETECT_IS_HIGHER_V);
//
//        telemetry.addData("S1 V", v1);
//        telemetry.addData("S1 Det", d1 ? 1 : 0);
//
//        telemetry.addData("S2 V", v2);
//        telemetry.addData("S2 Det", d2 ? 1 : 0);
//
//        telemetry.addData("S3 V", v3);
//        telemetry.addData("S3 Det", d3 ? 1 : 0);

        telemetry.addData("S1", d1);
        telemetry.addData("S2", d2);
        telemetry.addData("S3", d3);

        telemetry.update();
    }

    public boolean isIntakeOn() {
        return intakeMotor.getPower() > 0.4;
    }
    public boolean isIntakeOff() {
        return intakeMotor.getPower() < 0.1;
    }


    public void intakeReset() {
        all3 = false;
    }
    public void autoIntake() {
        if (!all3) {

            if ((areAllBallsDetected() && getCurrent() > 6)) {
                intakeOff();
                all3 = true;
                return;
            }

            intake1On();
        }
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean hasTriggeredStop() {
        return triggered;
    }

    public void setAutoEnabled(boolean enabled) {
        autoEnabled = enabled;
    }

    public void setCurrentStopEnabled(boolean enabled) {
        currentStopEnabled = enabled;
    }

    public void startTransfer() {
        setStopper(STOPPER_SHOOT_POS);
        engagePTO();
        intakeMotor.setPower(0.95);
        shooting = true;
        triggered = false;
    }

    public void stopTransfer() {
        setStopper(STOPPER_HOLD_POS);
        intakeMotor.setPower(0);
        shooting = false;
        currentStopEnabled = true;
    }

    public void startSlowTransfer() {
        startTransfer();
        currentStopEnabled = false;
        onSpeed(SLOW_TRANSFER_POWER);
    }

    public void stopSlowTransfer() {
        stopTransfer();
        intakeOff();
    }
}
