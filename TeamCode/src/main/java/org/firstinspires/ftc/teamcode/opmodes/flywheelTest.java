package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "flywheelTest", group = "Tuning")
public class flywheelTest extends DbzOpMode {
    public static double kP = 0.0001;
    public static double kI = 0.0;
    public static double kD = 0.00001;
    public static double kF = 0.00042; // feedforward coefficient (ticks/sec -> power)
    public static double targetVelocity = -2300; // ticks/sec

    private PIDController controller;
    private DcMotorEx motor1, motor2;
    private VoltageSensor batteryVoltageSensor;
    public static double holdPos = 0.3;
    public static double holdPos2 = 0.15;
    private Servo shoot1Servo;
    private Servo shoot2Servo;
    public static double shootPos = 0.5;

    @Override
    protected void opInit() {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtake1Motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2Motor");
//        shoot1Servo = hardwareMap.get(Servo.class, "shoot1Servo");
//        shoot2Servo = hardwareMap.get(Servo.class, "shoot2Servo");

        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // One motor reversed if your flywheels are mirrored
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor1.setDirection(DcMotorEx.Direction.REVERSE);



        controller = new PIDController(kP, kI, kD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    protected void opLoopHook() {


    }


    @Override
    protected void opLoop() {


        robot.intakeMotor.setPower(-1);
        controller.setPID(kP, kI, kD);

        controller.setIntegrationBounds(-0.3,0.3);
        double currentVelocity = motor2.getVelocity();
        double pid = controller.calculate(currentVelocity, targetVelocity);

        // Battery compensation for feedforward
        double batteryVoltage = batteryVoltageSensor.getVoltage();
        double voltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        double feedforward = (kF * targetVelocity) * (12.0 / batteryVoltage);

        double power = pid + feedforward;
        power = Math.max(-1, Math.min(1, power));

        motor1.setPower(power);
        motor2.setPower(power);
            if(dbzGamepad1.dpad_up){
                robot.holdServo.setPosition(holdPos);


            }
            else if(dbzGamepad1.dpad_down){
                robot.holdServo.setPosition(holdPos2);


            }



        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", targetVelocity - currentVelocity);
        telemetry.addData("Power", power);
        telemetry.addData("Battery Voltage", batteryVoltage);
        telemetry.update();
//        shoot1Servo.setPosition(shootPos);
//        shoot2Servo.setPosition(shootPos);
    }

    @Override
    protected void opTeardown() {}
}
