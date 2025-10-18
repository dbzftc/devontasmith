package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "flywheelTest", group = "Tuning")
public class flywheelTest extends DbzOpMode {
    public static double p = 0.0008; // Start small for velocity PID
    public static double i = 0;
    public static double d = 0.00001;
    public static double f = 0.1; // feedforward to overcome friction
    public static double targetVelocity = 1000; // ticks per second

    private PIDController vel_pid;

    private DcMotorEx motor1, motor2;

    public static String motorLeftName = "outtake1Motor";
    public static String motorRightName = "outtake2Motor";

    @Override
    protected void opInit() {
        motor1 = hardwareMap.get(DcMotorEx.class, motorLeftName);
        motor2 = hardwareMap.get(DcMotorEx.class, motorRightName);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Use RUN_USING_ENCODER to get velocity from the motor
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        vel_pid = new PIDController(p, i, d);
    }

    @Override
    protected void opLoop() {

        // Update PID coefficients live
        vel_pid.setPID(p, i, d);

        // Read current velocity of one motor (could also average both)
        double currentVelocity = motor2.getVelocity(); // ticks/sec

        // Calculate PID output
        double pid_output = vel_pid.calculate(currentVelocity, targetVelocity);

        // Total power = PID output + feedforward
        double power = pid_output + f;

        // Apply to motors (reverse if needed for opposite spin)
        motor1.setPower(-power);
        motor2.setPower(power);

        // Telemetry
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Motor1 Velocity", motor1.getVelocity());
        telemetry.addData("Motor2 Velocity", motor2.getVelocity());
        telemetry.addData("Power", power);
        telemetry.update();
    }

    @Override
    protected void opTeardown() {
    }

    @Override
    protected void opLoopHook() {}
}
