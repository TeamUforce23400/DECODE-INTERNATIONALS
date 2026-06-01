package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

@TeleOp(name = "Motor Servo Direction Debug", group = "Debug")
public class TeleopDebugMotors extends LinearOpMode {

    DcMotor motor;
    DcMotor motor1;
    Servo servo;
    Servo servoRightFront;
    Servo servoLeftFront;
    Servo servoRightBack;
    Servo servoLeftBack;
    ServoExGroup turretServos;

    double motorPower = 1.0;
    double servoPos = 0.5;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "inr");
        motor1 = hardwareMap.get(DcMotor.class, "inl");
        servoRightFront = hardwareMap.get(Servo.class, "trf");
        servoLeftFront = hardwareMap.get(Servo.class, "tlf");
        servoRightBack = hardwareMap.get(Servo.class, "trr");
        servoLeftBack = hardwareMap.get(Servo.class, "tlr");


        servoRightFront.setDirection(Servo.Direction.REVERSE);
        servoRightBack.setDirection(Servo.Direction.REVERSE);
        servoLeftBack.setDirection(Servo.Direction.FORWARD);
        servoLeftFront.setDirection(Servo.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoRightFront.setPosition(servoPos);
        servoLeftFront.setPosition(servoPos);
        servoRightBack.setPosition(servoPos);
        servoLeftBack.setPosition(servoPos);


        waitForStart();

        while (opModeIsActive()) {

            // Motor direction test
            if (gamepad1.a) {
                motor.setPower(motorPower);
                motor1.setPower(-motorPower);
            } else if (gamepad1.b) {
                motor.setPower(-motorPower);
                motor1.setPower(+motorPower);
            } else {
                motor.setPower(0);
                motor1.setPower(0);
            }

            // Servo position adjustment
            if (gamepad1.x) {
                servoPos -= 0.005;
                sleep(40);
            }

            if (gamepad1.y) {
                servoPos += 0.005;
                sleep(40);
            }

            if (gamepad1.dpad_left) {
                servoPos = 0.0;
            }

            if (gamepad1.dpad_up) {
                servoPos = 0.5;
            }

            if (gamepad1.dpad_right) {
                servoPos = 1.0;
            }

            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            servoRightFront.setPosition(servoPos);
            servoLeftFront.setPosition(servoPos);
            servoRightBack.setPosition(servoPos);
            servoLeftBack.setPosition(servoPos);


            // Motor direction toggle
            if (gamepad1.left_bumper) {
                motor.setDirection(DcMotor.Direction.FORWARD);
            }

            if (gamepad1.right_bumper) {
                motor.setDirection(DcMotor.Direction.REVERSE);
            }

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Motor Encoder", motor.getCurrentPosition());
            telemetry.addData("Motor Direction", motor.getDirection());
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();
        }
    }
}