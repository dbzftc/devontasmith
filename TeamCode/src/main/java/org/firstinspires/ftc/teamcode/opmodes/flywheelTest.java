package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "flywheelTest", group = "Tuning")
public class flywheelTest extends DbzOpMode {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0; // feedforward coefficient (ticks/sec -> power)
    public static double targetVelocity = 3000; // ticks/sec

    private PIDController controller;
    private DcMotorEx motor1, motor2;
    private VoltageSensor batteryVoltageSensor;

    @Override
    protected void opInit() {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtake1Motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2Motor");

        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // One motor reversed if your flywheels are mirrored
        motor1.setDirection(DcMotorEx.Direction.REVERSE);

        controller = new PIDController(kP, kI, kD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    protected void opLoopHook() {


    }


    @Override
    protected void opLoop() {
        controller.setPID(kP, kI, kD);

        double currentVelocity = motor2.getVelocity();
        double pid = controller.calculate(currentVelocity, targetVelocity);

        // Battery compensation for feedforward
        double batteryVoltage = batteryVoltageSensor.getVoltage();
        double feedforward = (kF * targetVelocity) * (12.0 / batteryVoltage);

        double power = pid + feedforward;
        power = Math.max(-1, Math.min(1, power));

        motor1.setPower(power);
        motor2.setPower(power);

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", targetVelocity - currentVelocity);
        telemetry.addData("Power", power);
        telemetry.addData("Battery Voltage", batteryVoltage);
        telemetry.update();
    }

    @Override
    protected void opTeardown() {}
}
