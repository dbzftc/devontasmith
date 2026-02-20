package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;
import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "V2RED")
public class V2RED extends DbzOpMode {

    private enum TurretState { NORMAL, CENTERING }

    private ElapsedTime intaketimer = new ElapsedTime();
    private ElapsedTime threeBallTimer = new ElapsedTime();
    private ElapsedTime pushDelayTimer = new ElapsedTime();

    public static double threeBallHoldTime = 0.15;
    public static double holdToPushDelay  = 0.25;

    private boolean atTTarget = false;
    private boolean atWTarget = false;

    private boolean threeBallsLocked = false;
    private boolean holdOpened = false;

    public static double dthresh = 4.0;

    public static double servooffset = 0.023;

    public static double Push0 = 0.06;
    public static double Push1 = 0.4;
    public static double Push2 = 0.6;
    public static double Push3 = 0.66;

    public static double lock = 0.15;

    public static double hoodServoPos = 0.33;

    public static double holdOpenPos = 0.2;
    public static double holdClosePos = 0.1;

    public static double hoodDipDuringShot = 0.015;

    public static double rpmA = 0.0125, rpmB = 4.153, rpmC = 1045.2;
    public static double hoodA = -0.00003, hoodB = 0.0097, hoodC = 0.118;
    public static double timeA = 0.0, timeB = 0.0, timeC = 0.0;

    public static double goalx = 144;
    public static double goaly = 144;

    public static double vkP = 4.8;
    public static double vkF = 1.32;

    public static double TV = 0;

    public static double threshold = 120;
    public static double threshold2 = 180;

    public static double turretZeroDeg = 329;

    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.002;

    public static double turretDeadbandDeg = 0.0;
    public static double turretMaxPower = 1.0;

    public static double turretKs = 0.001;
    public static double turretFFDeadbandDeg = 0.0;

    public static double turretoffset = 3.0;

    private Pose lastPose = new Pose();
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double lastTime = 0.0;

    protected Servo rightpushServo, leftpushServo, hoodServo, holdServo;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;

    private VoltageSensor batteryVoltageSensor;
    private AnalogInput turretEncoder;

    private DistanceSensor sensor1, sensor2;
//    protected Servo light, light2;

    private double lastLightPos = -1;
    private double lastLight2Pos = -1;

    private boolean shootLast = false;
    private boolean shooting = false;

    private boolean autoHoodActive = true;
    private boolean lastAButton = false;

    private boolean aimingActive = false;
    private boolean leftTriggerLast = false;

    private boolean intakeForwardOn = false;
    private boolean intakeReverseOn = false;

    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    private PIDController turretPID;

    private boolean moveshoot = false;
    private boolean lastshoot = false;

    private double turretHeadingOffsetDeg = 0.0;

    private boolean lastr1 = false;
    private boolean lastl1 = false;

    private TurretState turretState = TurretState.NORMAL;

    public static Follower follower;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    private Limelight3A limelight;
    private boolean lastSnapButton = false;

    private double targetVelocity = 0.0;

    @Override
    public void opInit() {

        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo  = hardwareMap.get(Servo.class, "leftpushServo");
        hoodServo      = hardwareMap.get(Servo.class, "hoodServo");
        holdServo      = hardwareMap.get(Servo.class, "holdServo");

        sensor1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        sensor2 = hardwareMap.get(DistanceSensor.class, "sensor2");
//
//        light  = hardwareMap.get(Servo.class, "light");
//        light2 = hardwareMap.get(Servo.class, "light2");

        hoodServo.setPosition(hoodServoPos);
        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);

        intakeMotor   = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        outtake1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2Motor.setDirection(DcMotorEx.Direction.FORWARD);

        turretPID = new PIDController(turretKp, turretKi, turretKd);
        turretPID.setTolerance(1.0);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);

        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(org.firstinspires.ftc.teamcode.opmodes.PoseCache.lastPose);

        lastPose = follower.getPose();
        velocityTimer.reset();
        lastTime = 0.0;

        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.update();
        follower.update();

        if (follower.getCurrentPath() != null) {
            drawOnlyCurrent();
        }
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();

        follower.startTeleopDrive();
    }

    @Override
    public void opLoop() {

        boolean shoottoggle = gamepad1.left_stick_button;
        if (shoottoggle && !lastshoot) {
            moveshoot = !moveshoot;
        }
        lastshoot = shoottoggle;

        boolean aButton = gamepad1.a;
        if (aButton && !lastAButton) {
            autoHoodActive = !autoHoodActive;
        }
        lastAButton = aButton;

        boolean rightStickPress = gamepad2.right_stick_button;
        boolean leftStickPress  = gamepad2.left_stick_button;

        if (rightStickPress && !lastr1) {
            turretHeadingOffsetDeg -= turretoffset;
        }
        if (leftStickPress && !lastl1) {
            turretHeadingOffsetDeg += turretoffset;
        }

        lastr1 = rightStickPress;
        lastl1 = leftStickPress;

        boolean snap = gamepad1.b;
        if (snap && !lastSnapButton) {
            trySnapPoseFromLimelight();
        }
        lastSnapButton = snap;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        follower.update();

        applyHoodAndVelocityRegressions();

//        updateLights();
        checkThreeBallsAndLock();
        shootFastOnly();
        activeIntake();

        if (dbzGamepad1.x) {
            follower.setPose(new Pose(8, 8.5, Math.toRadians(180)));
            turretHeadingOffsetDeg = 0.0;
        }
        if (dbzGamepad1.y) {
            follower.setPose(new Pose(136, 8.5, Math.toRadians(0)));
            turretHeadingOffsetDeg = 0.0;
        }

        aim();
        runFlywheelVelocityControl();

        if (follower.getCurrentPath() != null) {
            draw();
        }

        telemetryM.update(telemetry);
        telemetry.update();
    }

    private void applyHoodAndVelocityRegressions() {

        Pose poseNow = follower.getPose();

        if (autoHoodActive && poseNow != null) {

            Pose vGoal = updateGoalV2(poseNow);

            double vDist = Math.hypot(vGoal.getX() - poseNow.getX(), vGoal.getY() - poseNow.getY());

            double hoodPos = (hoodA * vDist * vDist) + (hoodB * vDist) + hoodC;
            if (shooting) hoodPos -= hoodDipDuringShot;

            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
            hoodServo.setPosition(hoodPos);

            double targetRPM = (rpmA * vDist * vDist) + (rpmB * vDist) + rpmC;
            targetRPM = Math.max(0.0, targetRPM);

            double ticksPerRev = outtake2Motor.getMotorType().getTicksPerRev();
            targetVelocity = (targetRPM * ticksPerRev) / 60.0;

        } else if (!autoHoodActive) {

            hoodServo.setPosition(hoodServoPos);
            targetVelocity = TV;

        } else {

            hoodServo.setPosition(hoodServoPos);
            targetVelocity = 0.0;
        }
    }

    private Pose updateGoalV2(Pose robotPose) {

        com.pedropathing.math.Vector vel = follower.getVelocity();
        double vx = (vel != null) ? vel.getXComponent() : 0.0;
        double vy = (vel != null) ? vel.getYComponent() : 0.0;

        double dist = Math.hypot(goalx - robotPose.getX(), goaly - robotPose.getY());
        double shotTime = (timeA * dist * dist) + (timeB * dist) + timeC;

        double vGoalX = goalx - (vx * shotTime);
        double vGoalY = goaly - (vy * shotTime);

        return new Pose(vGoalX, vGoalY, 0);
    }

    private void checkThreeBallsAndLock() {

        double dist1 = sensor1.getDistance(DistanceUnit.CM);
        double dist2 = sensor2.getDistance(DistanceUnit.CM);

        boolean detected1 = dist1 < dthresh;
        boolean detected2 = dist2 < dthresh;

        boolean ballDetected = detected1 && detected2;

        if (!ballDetected) {
            threeBallTimer.reset();
            pushDelayTimer.reset();
            threeBallsLocked = false;
            holdOpened = false;
            return;
        }

        if (threeBallTimer.seconds() < threeBallHoldTime) {
            return;
        }

        if (!holdOpened) {
            holdServo.setPosition(holdOpenPos);
            pushDelayTimer.reset();
            holdOpened = true;
            return;
        }

        if (!threeBallsLocked && pushDelayTimer.seconds() > holdToPushDelay) {
            leftpushServo.setPosition(lock);
            rightpushServo.setPosition(lock - servooffset);
            holdServo.setPosition(holdOpenPos);
            threeBallsLocked = true;
        }

        if (threeBallsLocked) {
            holdServo.setPosition(holdOpenPos);
        }
    }

    private void resetAfterShooting() {
        threeBallsLocked = false;
        holdOpened = false;

        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);

        intakeForwardOn = true;
        intakeReverseOn = false;
        intakeMotor.setPower(-1);
    }
//
//    private void updateLights() {
//
//        if (light == null || light2 == null) return;
//
//        double newPos = threeBallsLocked ? 0.722 : 0.0;
//        newPos = Math.round(newPos * 1000.0) / 1000.0;
//
//        if (Math.abs(lastLightPos - newPos) > 0.001) {
//            light.setPosition(newPos);
//            lastLightPos = newPos;
//        }
//
//        if (Math.abs(lastLight2Pos - newPos) > 0.001) {
//            light2.setPosition(newPos);
//            lastLight2Pos = newPos;
//        }
//    }

    private void shootFastOnly() {

        boolean rightTriggerHeld = dbzGamepad1.right_trigger > 0.1;

        if (rightTriggerHeld && !shootLast && !shooting) {

            threeBallsLocked = false;
            holdOpened = false;

            holdServo.setPosition(holdOpenPos);

            leftpushServo.setPosition(Push3);
            rightpushServo.setPosition(Push3 - servooffset);

            intaketimer.reset();
            shooting = true;
        }

        if (shooting && intaketimer.milliseconds() > 700) {

            leftpushServo.setPosition(Push0);
            rightpushServo.setPosition(Push0 - servooffset);

            holdServo.setPosition(holdClosePos);

            shooting = false;
            resetAfterShooting();
        }

        shootLast = rightTriggerHeld;
    }

    private void activeIntake() {

        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        if (shooting) {
            intakeMotor.setPower(0);
            lastRightBumper = rb;
            lastLeftBumper = lb;
            return;
        }

        if (threeBallsLocked) {
            intakeMotor.setPower(0);
            lastRightBumper = rb;
            lastLeftBumper = lb;
            return;
        }

        if (rb && !lastRightBumper) {
            intakeForwardOn = !intakeForwardOn;
            intakeReverseOn = false;
        }

        if (lb && !lastLeftBumper) {
            intakeReverseOn = !intakeReverseOn;
            intakeForwardOn = false;
        }

        if (intakeForwardOn) {
            intakeMotor.setPower(-1);
        } else if (intakeReverseOn) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        if (!threeBallsLocked && !shooting) {
            leftpushServo.setPosition(Push0);
            rightpushServo.setPosition(Push0 - servooffset);
        }

        lastRightBumper = rb;
        lastLeftBumper = lb;
    }

    private void aim() {

        boolean leftTriggerPressed = dbzGamepad1.left_trigger > 0.1;
        if (leftTriggerPressed && !leftTriggerLast) {
            aimingActive = !aimingActive;
            turretPID.reset();
        }
        leftTriggerLast = leftTriggerPressed;

        double targetAngleDeg;

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
        atTTarget = Math.abs(errorDeg) < 2.0;

        if (Math.abs(errorDeg) <= turretDeadbandDeg) {
            turret.setPower(0);
            return;
        }

        turretPID.setPID(turretKp, turretKi, turretKd);
        double pidOut = turretPID.calculate(currentAngleDeg, targetAngleDeg);

        double ff = 0.0;
        if (Math.abs(errorDeg) > turretFFDeadbandDeg) {
            ff = Math.copySign(turretKs, errorDeg);
        }

        double output = pidOut + ff;

        if (output > turretMaxPower) output = turretMaxPower;
        if (output < -turretMaxPower) output = -turretMaxPower;

        turret.setPower(output);
    }

    private void runFlywheelVelocityControl() {

        if (targetVelocity <= 1.0) {
            outtake1Motor.setPower(0);
            outtake2Motor.setPower(0);
            atWTarget = false;
            return;
        }

        double currentVelocity = outtake2Motor.getVelocity();
        double maxVelocity = outtake2Motor.getMotorType().getMaxRPM()
                * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;

        double normalizedError = (targetVelocity - currentVelocity) / maxVelocity;
        double pTerm = vkP * normalizedError;

        double feedforward = vkF * (targetVelocity / maxVelocity);
        double batteryVoltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        feedforward *= 12.0 / batteryVoltage;

        double power = Math.max(-1.0, Math.min(1.0, pTerm + feedforward));

        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);

        atWTarget = Math.abs(targetVelocity - currentVelocity) < 40.0;
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

        Pose vGoal = updateGoalV2(pose);

        double fieldAngle = Math.atan2(vGoal.getY() - pose.getY(), vGoal.getX() - pose.getX());
        double turretAngleDeg = Math.toDegrees(fieldAngle - pose.getHeading()) + turretHeadingOffsetDeg;

        return angleWrap(turretAngleDeg);
    }

    private double overshoot() {
        double desired = angleWrap(getDesiredTurretAngleDeg());
        if (desired > threshold2) return threshold2;
        if (desired < -threshold) return -threshold;
        return desired;
    }

    private double angleWrap(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    private void trySnapPoseFromLimelight() {

        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        Pose3D llpose = result.getBotpose();
        if (llpose == null) return;

        double absX = llpose.getPosition().y * 39.37 + 72.0;
        double absY = 72.0 - llpose.getPosition().x * 39.37;

        double yawRad = llpose.getOrientation().getYaw(AngleUnit.RADIANS);
        double turretRad = Math.toRadians(getTurretAngleDeg());
        double chassisHeading = AngleUnit.normalizeRadians(yawRad - turretRad);

        follower.setPose(new Pose(absX, absY, chassisHeading));
    }

    @Override
    public void opLoopHook() {}

    @Override
    public void opTeardown() {}
}
