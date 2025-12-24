package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;
import static org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap.Motor.leftpushServo;
import static org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap.Motor.rightpushServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "TurretPIDTest")
public class TurretPIDTest extends DbzOpMode {
    private double power = 0.8;
    private ElapsedTime intaketimer = new ElapsedTime();
    private boolean leftTriggerLast = false;
    private boolean rightTriggerLast = false;
    public static double targetX = 135;
    public static double targetY = 61;

    protected Servo rightpushServo, leftpushServo;
    private PIDController controller;
    private DcMotorEx motor1, motor2;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private VoltageSensor batteryVoltageSensor;
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 2;
    public static double vkP = 0.03;
    public static double vkI = 0.0;
    public static double vkD = 0.001;
    public static double vkF = 2;
    public static double targetVelocity = -2300; // ticks/sec

    private boolean leftBumperLast = false;

    private boolean shootLast = false;
    private boolean shooting = false;
    private boolean Shooting = false;

    public static double turretZeroDeg = 336.0;

    private AnalogInput turretEncoder;

    private boolean aimingActive = false;
    private boolean lastLeftBumper = false;


    public static Follower follower;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    @Override
    public void opInit() {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtake1Motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2Motor");
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        intakeMotor = robot.intakeMotor;
        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;
        outtake1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2Motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake2Motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // One motor reversed if your flywheels are mirrored
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor1.setDirection(DcMotorEx.Direction.REVERSE);



        controller = new PIDController(kP, kI, kD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(1.0);

        PanelsConfigurables.INSTANCE.refreshClass(this);

        // ---- moved from init_loop() ----
        follower.update();
        if (follower.getCurrentPath() != null) {
            drawOnlyCurrent();
        }

        // ---- moved from start() ----
        follower.startTeleopDrive();
        controller.reset();
    }

    @Override
    public void opLoop() {

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        shoot();

        activeIntake();
        follower.update();
        aim();

        if (follower.getCurrentPath() != null) {
            draw();
        }

        telemetryM.update(telemetry);
//        robot.intakeMotor.setPower(-1);
        controller.setPID(vkP, vkI, vkD);

        controller.setIntegrationBounds(-0.3,0.3);
        double currentVelocity = motor2.getVelocity();
        double pid = controller.calculate(currentVelocity, targetVelocity);

        // Battery compensation for feedforward
        double batteryVoltage = batteryVoltageSensor.getVoltage();
        double voltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        double feedforward = (vkF * targetVelocity) * (12.0 / batteryVoltage);

        double power = pid + feedforward;
        power = Math.max(-1, Math.min(1, power));

        outtake1Motor.setPower(-power);
        outtake2Motor.setPower(power);



        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", targetVelocity - currentVelocity);
        telemetry.addData("Power", power);
        telemetry.addData("Battery Voltage", batteryVoltage);
        telemetry.update();
    }

    private void shoot() {
        boolean shootPressed = dbzGamepad1.right_bumper;
        if (shootPressed && !shootLast && !shooting) {
            leftpushServo.setPosition(0.9);
            rightpushServo.setPosition(0.86);

            intaketimer.reset();
            shooting = true;
        }

        if (shooting && intaketimer.milliseconds() > 1000) {
            leftpushServo.setPosition(0.25);   // OPEN again (idle)
            rightpushServo.setPosition(0.21);

            shooting = false;
        }

        shootLast = shootPressed;
//
//        boolean leftBumperPressed = dbzGamepad1.left_bumper;
//
//        if (leftBumperPressed && !leftBumperLast) {
//            if (!Shooting) {
//                outtake1Motor.setPower(-power);
//                outtake2Motor.setPower(power);
//                Shooting = true;
//            } else {
//                outtake1Motor.setPower(0);
//                outtake2Motor.setPower(0);
//                Shooting = false;
//            }
//        }
//
//        leftBumperLast = leftBumperPressed;
    }

    private void activeIntake() {

        boolean leftTriggerHeld  = dbzGamepad1.left_trigger > 0.1;
        boolean rightTriggerHeld = dbzGamepad1.right_trigger > 0.1;

        if (rightTriggerHeld) {
            intakeMotor.setPower(1);
            leftpushServo.setPosition(0.25);
            rightpushServo.setPosition(0.21);
        }

        if (leftTriggerHeld) {
            intakeMotor.setPower(-1);
            leftpushServo.setPosition(0.25);
            rightpushServo.setPosition(0.21);
        }

        if (!leftTriggerHeld && leftTriggerLast) {
            intakeMotor.setPower(0);
            leftpushServo.setPosition(0.30);
            rightpushServo.setPosition(0.26);
        }

        if (!rightTriggerHeld && rightTriggerLast) {
            intakeMotor.setPower(0);
            leftpushServo.setPosition(0.30);
            rightpushServo.setPosition(0.26);
        }

        leftTriggerLast  = leftTriggerHeld;
        rightTriggerLast = rightTriggerHeld;
    }


    private void aim() {

        boolean leftBumper = gamepad1.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            aimingActive = !aimingActive;
            controller.reset();
        }
        lastLeftBumper = leftBumper;

        if (!aimingActive) {
            turret.setPower(0);
            return;
        }

        controller.setPID(kP, kI, kD);
        controller.setIntegrationBounds(-0.25, 0.25);

        double currentAngle = getTurretAngleDeg();
        double targetAngle = getDesiredTurretAngleDeg();

        double error = angleWrap(targetAngle - currentAngle);
        double pidOutput = controller.calculate(0, error);

        double currentVelocity = turret.getVelocity();
        double maxVelocity = turret.getMotorType().getMaxRPM() *
                turret.getMotorType().getTicksPerRev() / 60.0;

        double normalizedVelocity = currentVelocity / maxVelocity;
        double ffOutput = kF * normalizedVelocity;

        double output = pidOutput + ffOutput;
        output = Math.max(-1.0, Math.min(1.0, output));

        turret.setPower(output);

        telemetryM.debug("Turret Angle (deg)", currentAngle);
        telemetryM.debug("Target Angle (deg)", targetAngle);
        telemetryM.debug("Angle Error (deg)", error);
        telemetryM.debug("PID Output", pidOutput);
        telemetryM.debug("Velocity FF", ffOutput);
        telemetryM.debug("Final Motor Power", output);
    }

    private double getTurretAngleDeg() {
        double voltage = turretEncoder.getVoltage();
        double angle = (voltage / turretEncoder.getMaxVoltage()) * 360.0;
        angle -= turretZeroDeg;
        return angleWrap(angle);
    }

    private double getDesiredTurretAngleDeg() {
        Pose pose = follower.getPose();
        if (pose == null) return getTurretAngleDeg();

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        double fieldAngle = Math.atan2(dy, dx);
        double turretAngle = Math.toDegrees(fieldAngle - pose.getHeading());

        return angleWrap(turretAngle);
    }

    private double angleWrap(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    @Override
    public void opLoopHook() {
        // required by DbzOpMode
    }

    @Override
    public void opTeardown() {
        // required by DbzOpMode
    }
}
