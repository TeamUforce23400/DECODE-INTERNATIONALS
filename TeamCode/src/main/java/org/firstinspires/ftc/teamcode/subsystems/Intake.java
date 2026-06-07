package org.firstinspires.ftc.teamcode.subsystems;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class Intake extends SubsystemBase {

    public final DcMotorEx intake;
    public final DcMotorEx transfer;
    private final ServoEx stopperRight;
    private final TelemetryManager telemetry;
//    private final ServoEx stopperLeft;
    private final ServoExGroup stopper;

    public final double openPos = 0.1; // TODO: This will be your stopper open position.
    public static double closePos = 0.3; // TODO: This will be your stopper close position.

    private final AnalogInput s1;
    private final AnalogInput s2;

    public boolean isIntakeOn = false;
    public boolean firstIn = false;
    public static boolean all3 = false;

    public Intake(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        intake = hardwareMap.get(DcMotorEx.class, "inr");
        transfer = hardwareMap.get(DcMotorEx.class, "inl");

        stopperRight = new ServoEx(hardwareMap, "sr");
//        stopperLeft = new ServoEx(hardwareMap, "sl");
        stopper = new ServoExGroup(stopperRight);
        stopper.setInverted(false);

        s1 = hardwareMap.get(AnalogInput.class, "s1");
        s2 = hardwareMap.get(AnalogInput.class, "s2");

        telemetry = telemetryManager;


        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        testSensors();
    }

    public boolean isBallDetected01() {
        return ((s1.getVoltage()*32.50930976)-2.695384202) < 3.5;
    }

    public boolean isBallDetected02() {
        return ((s2.getVoltage()*32.50930976)-2.695384202) < 3.5;
    }

    public boolean areAllBallsDetected() {
        return isBallDetected01() && isBallDetected02();
    }

    public boolean noBalls() {
        return !isBallDetected01() && !isBallDetected02();
    }

    public void testSensors() {
        boolean d1 = isBallDetected01();
        boolean d2 = isBallDetected02();

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

        telemetry.update();
    }

    public void powerIntake(double power) {
        intake.setPower(power);
    }

    public void powerTransfer(double power) {
        transfer.setPower(power);
    }

    public void powerFullIntake(double power) {
        transfer.setPower(power);
        intake.setPower(power);
    }

    public void openStopper(boolean open) {
        if (open) {
            stopper.set(openPos);
        } else {
            stopper.set(closePos);
        }
    }

    public void autoIntake() {
        if (!firstIn) {
            if (transfer.getCurrent(CurrentUnit.AMPS) > 5.8) {
                firstIn = true;
                powerTransfer(0.0);
            }
        }
        else if (!all3) {
            if (intake.getCurrent(CurrentUnit.AMPS) > 6) {
                all3 = true;
            }
        }
    }

    public void setStopper(double position) {
        stopper.set(position);
    }

    public double getIntakePower() {
        return intake.getPower();
    }

    public double getTransferPower() {
        return transfer.getPower();
    }


    public double getStopperPosition() {
        return stopper.getRawPosition();
    }

}
