package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@TeleOp(name = "TelemetryTest")
public class TelemetryTest extends DbzOpMode {

    protected Servo rightpushServo, leftpushServo;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private AnalogInput turretEncoder;

    @Override
    public void opInit() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        outtake1Motor = hardwareMap.get(DcMotorEx.class, "outtake1Motor");
        outtake2Motor = hardwareMap.get(DcMotorEx.class, "outtake2Motor");

        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");

        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake1Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void opLoop() {
        telemetry.addData("Turret encoder voltage", turretEncoder.getVoltage());
        telemetry.addData("Turret ticks", turret.getCurrentPosition());
        telemetry.addData("Intake ticks", intakeMotor.getCurrentPosition());
        telemetry.addData("Outtake 1 ticks", outtake1Motor.getCurrentPosition());
        telemetry.addData("Outtake 2 ticks", outtake2Motor.getCurrentPosition());

        telemetry.addData("Left push servo: ", leftpushServo.getPosition());
        telemetry.addData("Right push servo: ", rightpushServo.getPosition());

        telemetry.update();
    }


    @Override
    public void opLoopHook() {
    }

    @Override
    public void opTeardown() {
    }
}