package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;


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
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "TurretPIDTest")
public class TurretPIDTest extends DbzOpMode {
    private ElapsedTime intaketimer = new ElapsedTime();
    private boolean leftTriggerLast = false;
    private boolean rightTriggerLast = false;
    public static double targetX = 17.5;
    public static double targetY = 0;

    public static double targetVelocity = -200; // ticks/sec
    protected Servo rightpushServo, leftpushServo, hoodServo;
    private PIDController controller;
    private DcMotorEx motor1, motor2;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private VoltageSensor batteryVoltageSensor;
    public static double kP = 0.025;
    public static double kI = 0.0;
    public static double kD = 0.0000001;
    public static double kF = 0.5;
    public static double vkP = 2.742;
    public static double vkI = 0.0;
    public static double vkD = 0.0011;
    public static double vkF = 1.125;
    public static double hoodServoPos = 0.7;
    public static double TV = -1400;
    public static double threshold = 70;
    public static double offsetDistance = 110.0; // distance threshold to apply turret offset
    public static double offsetAngle = 0;

    private boolean shootLast = false;
    private boolean shooting = false;
    private boolean autoHoodActive = false;
    private boolean lastXButton = false;

    public static double turretZeroDeg = 336.0;
    public static double intakePos = 0;
    private AnalogInput turretEncoder;

    private boolean aimingActive = false;
    boolean intakeForwardOn = false;
    boolean intakeReverseOn = false;

    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;
    private PIDController turretBigPID; //Doesn't do anything anymore because we set the effective range of smallPID to 700
    private PIDController turretSmallPID;

    public static double turretPidSwitchDeg = 20;

    public static double turretBigKp = 0.01;
    public static double turretBigKi = 0.0;
    public static double turretBigKd = 0.01;

    public static double turretSmallKp = 0.0075;
    public static double turretSmallKi = 0.0;
    public static double turretSmallKd = 0.075;
    public static double powMult = 1;
    public static double multiplier = 0.75;


    private enum TurretState {
        NORMAL,
        CENTERING
    }

    private TurretState turretState = TurretState.NORMAL;

    public static Follower follower;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    @Override
    public void opInit() {

        motor1 = hardwareMap.get(DcMotorEx.class, "outtake1Motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2Motor");
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(0.7);
        intakeMotor = robot.intakeMotor;
        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor1.setDirection(DcMotorEx.Direction.REVERSE);

        turretBigPID = new PIDController(turretBigKp, turretBigKi, turretBigKd);
        turretSmallPID = new PIDController(turretSmallKp, turretSmallKi, turretSmallKd);

        turretBigPID.setTolerance(1.0);
        turretSmallPID.setTolerance(0.5);


        controller = new PIDController(kP, kI, kD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0.0, 0.0, 0.0));


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(1.0);

        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.update();
        if (follower.getCurrentPath() != null) {
            drawOnlyCurrent();
        }

        follower.startTeleopDrive();
        controller.reset();
    }

    @Override
    public void opLoop() {

        boolean xButton = gamepad1.a;
        if (xButton & !lastXButton) {
            autoHoodActive = !autoHoodActive;
        }
        lastXButton = xButton;
        Pose ppose = follower.getPose();

        if (autoHoodActive) {
            if (ppose != null) {
                double dx = targetX - ppose.getX();
                double dy = targetY - ppose.getY();
                double distance = Math.sqrt(dx*dx + dy*dy);

                hoodServo.setPosition(-0.000000217243 * Math.pow(distance, 3) + 0.0000386489 * Math.pow(distance, 2) + 0.00297592 * distance + 0.413722 - 0.08 );
                targetVelocity = (-0.0111536 * distance * distance - 4.00719 * distance -1097.37524);
            } else {
                hoodServo.setPosition(0.7);
                targetVelocity = 0;
            }
        } else {
            hoodServo.setPosition(hoodServoPos);
            targetVelocity = TV;
        }
        Pose pppose = follower.getPose();
        if(pppose!=null){
            double dx = targetX - ppose.getX();
            double dy = targetY - ppose.getY();
            double distance = Math.sqrt(dx*dx + dy*dy);
            telemetry.addData("Distance to target", distance);
        } else {
            telemetry.addData("Distance to target", "Pose is null");
        }

        intakeMotor.setPower(intakePos);
//        hoodServo.setPosition(hoodServoPos);
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
//        controller.setPID(vkP, vkI, vkD);
//
//        controller.setIntegrationBounds(-0.3,0.3);
//        double VcurrentVelocity = motor2.getVelocity();
//        double pid = controller.calculate(VcurrentVelocity, targetVelocity);
//
//        // Battery compensation for feedforward
//        double batteryVoltage = batteryVoltageSensor.getVoltage();
//        double feedforward = (vkF * targetVelocity) * (12.0 / batteryVoltage);
//
//        double power = pid + feedforward;
//        power = Math.max(-1, Math.min(1, power));
//
//        motor1.setPower(power);
//        motor2.setPower(power);
        double currentVelocity = motor2.getVelocity();


        double maxVelocity = motor2.getMotorType().getMaxRPM() *
                motor2.getMotorType().getTicksPerRev() / 60.0;

        double velocityError = (targetVelocity - currentVelocity) / maxVelocity;
        double pidOutput = vkP * velocityError;

        double ff = vkF * (targetVelocity / maxVelocity);
        double batteryVoltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        ff *= 12.0 / batteryVoltage;
        double power = pidOutput + ff;

        power = Math.max(-1.0, Math.min(1.0, power));

        outtake1Motor.setPower(power);  // reverse if necessary
        outtake2Motor.setPower(power);

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error (ticks/sec)", targetVelocity - currentVelocity);
        telemetry.addData("PID Output", pidOutput);
        telemetry.addData("Feedforward", ff);
        telemetry.addData("Motor Power", power);
        telemetry.addData("Battery Voltage", batteryVoltage);
//        telemetry.addData("X", ppose.getX());
//        telemetry.addData("Y", ppose.getY());
        telemetry.update();
    }



    private void shoot() {

        boolean rightTriggerHeld = dbzGamepad1.right_trigger > 0.1;
        if (rightTriggerHeld && !shootLast && !shooting) {
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

        shootLast = rightTriggerLast;



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
        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        if (shooting) return;

        if (leftBumper && !lastLeftBumper) {
            intakeForwardOn = !intakeForwardOn;
            intakeReverseOn = false;
        }
        if (rightBumper && !lastRightBumper) {
            intakeReverseOn = !intakeReverseOn;
            intakeForwardOn = false;
        }
        if (intakeForwardOn) {
            intakeMotor.setPower(1);
            leftpushServo.setPosition(0.18);
            rightpushServo.setPosition(0.14);
        }
        else if (intakeReverseOn) {
            intakeMotor.setPower(-1);
            leftpushServo.setPosition(0.18);
            rightpushServo.setPosition(0.14);
        }
        else {
            intakeMotor.setPower(0);
            leftpushServo.setPosition(0.30);
            rightpushServo.setPosition(0.26);
        }
        lastLeftBumper  = leftBumper;
        lastRightBumper = rightBumper;
    }



    private void aim() {
        boolean leftTriggerPressed  = dbzGamepad1.left_trigger > 0.1;
        if (leftTriggerPressed && !leftTriggerLast) {
            aimingActive = !aimingActive;
            turretBigPID.reset();
            turretSmallPID.reset();
        }
        leftTriggerLast = leftTriggerPressed;

        // --- Choose target angle ---
//        double targetAngleDeg;
//        if (aimingActive) {
//            targetAngleDeg = getDesiredTurretAngleDeg();
//        } else {
//            targetAngleDeg = 0.0; // forward (3.08V)
//        }

        double targetAngleDeg;
        double desired = angleWrap(getDesiredTurretAngleDeg());

        if (!aimingActive) {
            turretState = TurretState.NORMAL;
            targetAngleDeg = 0.0;
        } else {
            double desiredClamped = overshoot();

            switch (turretState) {
                case NORMAL:
                    if (Math.abs(desiredClamped - getTurretAngleDeg()) <= threshold) {
                        targetAngleDeg = desiredClamped;
                    } else {
                        turretState = TurretState.CENTERING;
                        targetAngleDeg = 0.0;
                    }
                    break;

                case CENTERING:
                    targetAngleDeg = 0.0;
                    if (Math.abs(getTurretAngleDeg()) < 5.0) {
                        turretState = TurretState.NORMAL;
                    }
                    break;

                default:
                    targetAngleDeg = 0.0;
                    turretState = TurretState.NORMAL;
                    break;
            }
        }

        double currentAngleDeg = getTurretAngleDeg();

        double errorDeg = angleWrap(targetAngleDeg - currentAngleDeg);

        double output;
        if (Math.abs(errorDeg) > turretPidSwitchDeg) {
            turretBigPID.setPID(turretBigKp, turretBigKi, turretBigKd);
            output = turretBigPID.calculate(0, errorDeg);
        } else {
            turretSmallPID.setPID(turretSmallKp, turretSmallKi, turretSmallKd);
            output = turretSmallPID.calculate(0, errorDeg);
        }

        output = Math.max(-1.0, Math.min(1.0, output));

        turret.setPower(output);

        telemetryM.addData("Turret Angle (deg)", currentAngleDeg);
        telemetryM.addData("Target Angle (deg)", targetAngleDeg);
        telemetryM.addData("Error (deg)", errorDeg);
        telemetryM.addData("Turret Power", output);
    }

    private double getTurretAngleDeg() {
        double voltage = turretEncoder.getVoltage();
        double angle = (voltage / turretEncoder.getMaxVoltage()) * 360.0;
        angle -= turretZeroDeg;
        return angleWrap(angle);
    }

    private double getDesiredTurretAngleDeg() {
        Pose pose = follower.getPose();
        if (pose == null) {
            // Follower hasn't initialized yet, just return current turret angle
            return getTurretAngleDeg();
        }

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        double fieldAngle = Math.atan2(dy, dx);
        double turretAngle = Math.toDegrees(fieldAngle - pose.getHeading());

        double distance = Math.sqrt(dx*dx + dy*dy);
        if (distance >= offsetDistance) {
            turretAngle += offsetAngle; // apply right shift
        }

        return turretAngle;
    }

    private double overshoot() {
        double desired = angleWrap(getDesiredTurretAngleDeg());
        if (desired > threshold) {
            return threshold;
        }
        if (desired < -threshold) {
            return -threshold;
        }
        return desired;
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
