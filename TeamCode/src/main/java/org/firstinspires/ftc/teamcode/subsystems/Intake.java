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

    public final DcMotorEx intakeRight;
    public final DcMotorEx intakeLeft;
    private final ServoEx stopperRight;
    private final ServoEx stopperLeft;
    private final ServoExGroup stopper;

    public final double openPos = 0; // TODO: This will be your stopper open position.
    public final double closePos = 1; // TODO: This will be your stopper close position.

    public Intake(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        intakeRight = hardwareMap.get(DcMotorEx.class, "inr");
        intakeLeft = hardwareMap.get(DcMotorEx.class, "inl");

        stopperRight = new ServoEx(hardwareMap, "servoRight");
        stopperLeft = new ServoEx(hardwareMap, "servoLeft");
        stopper = new ServoExGroup(stopperLeft,stopperRight);

        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void periodic() {
    }

    public void powerIntake(double power) {
        intakeRight.setPower(power);
    }

    public void powerTransfer(double power) {
        intakeLeft.setPower(power);
    }

    public void powerFullIntake(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void openStopper(boolean open) {
        if (open) {
            stopper.set(openPos);
        } else {
            stopper.set(closePos);
        }
    }

}
