package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@Autonomous(name = "REDAUTOV2")
public class REDAUTOV2 extends DbzOpMode {

    public static double servooffset = 0.05;
    public static double Push0 = 0.81;
    public static double Push3 = 0.26;
    public static double lock = 0.75;
    public static double holdOpenPos = 0.8;
    public static double holdClosePos = 0.8;
    public static double dthresh = 0.24;

    public static double TV = 1550;
    public static double hoodServoPos = 0.5;
    public static double vkP = 17, vkF = 1.2, vkD = 0.0, vkDMax = 0.25;

    public static double velA = -0.0157003, velB = 11.6092, velC = 727.08688;
    public static double hoodA = -0.0000876693, hoodB = 0.0228448, hoodC = -0.779915;
    public static double timeA = 0.00002, timeB = 0.004, timeC = 0.25;

    public static double goalx = 144, goaly = 144;

    public static double hoodDipDuringShot = 0.015;
    public static double dipDelaySec = 0.5;
    public static double dipDurationSec = 0.15;

    public static double turretZeroDeg = 230;
    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.0007;
    public static double turretDeadbandDeg = 0.0;
    public static double turretMaxPower = 1.0;
    public static double turretKs = 0.0;
    public static double turretFFDeadbandDeg = 0.0;
    public static double threshold = 180;
    public static double threshold2 = 180;
    public static double turretHeadingOffsetDeg = 0.0;

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;
        public PathChain Path8, Path9, Path10, Path11, Path12, Path13;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(32.298, 134.112), new Pose(45.714, 83.627)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(282))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(45.714, 83.627), new Pose(51.180, 59.307), new Pose(22.385, 59.807)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(22.385, 59.807), new Pose(51.180, 59.307), new Pose(45.714, 83.627)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(45.714, 83.627), new Pose(35.205, 47.668), new Pose(7.727, 61.398)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(7.727, 61.398), new Pose(35.205, 47.668), new Pose(45.714, 83.627)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(45.714, 83.627), new Pose(35.205, 47.668), new Pose(7.727, 61.398)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(7.727, 61.398), new Pose(35.205, 47.668), new Pose(45.714, 83.627)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(45.714, 83.627), new Pose(35.205, 47.668), new Pose(7.727, 61.398)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(7.727, 61.398), new Pose(35.205, 47.668), new Pose(45.714, 83.627)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.714, 83.627), new Pose(22.981, 83.671)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(22.981, 83.671), new Pose(45.714, 83.627)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(275))
                    .setReversed()
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(45.714, 83.627), new Pose(50.870, 34.736), new Pose(22.248, 34.776)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(22.248, 34.776), new Pose(59.360, 99.615)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }

    protected Servo rightpushServo, leftpushServo, hoodServo, holdServo;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private VoltageSensor batteryVoltageSensor;
    private AnalogInput turretEncoder, distancez;

    private Follower follower;
    private Paths paths;

    private static final Pose START_POSE = new Pose(32.298, 134.112, Math.toRadians(180));

    private enum AutonState {
        FOLLOW_PATH1, SHOOT_1,
        FOLLOW_PATH2,
        FOLLOW_PATH3, SHOOT_3,
        FOLLOW_PATH4, INTAKE_WAIT_1,
        FOLLOW_PATH5, SHOOT_5,
        FOLLOW_PATH6, INTAKE_WAIT_2,
        FOLLOW_PATH7, SHOOT_7,
        FOLLOW_PATH8, INTAKE_WAIT_3,
        FOLLOW_PATH9, SHOOT_9,
        FOLLOW_PATH10,
        FOLLOW_PATH11, SHOOT_11,
        FOLLOW_PATH12,
        FOLLOW_PATH13, SHOOT_13,
        DONE
    }
    private AutonState autonState = AutonState.FOLLOW_PATH1;

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime intakeRevTimer = new ElapsedTime();
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime dipTimer = new ElapsedTime();

    private enum IntakePhase { WAITING, REVERSING }
    private IntakePhase intakePhase = IntakePhase.WAITING;

    private enum TurretState { NORMAL, CENTERING }
    private TurretState turretState = TurretState.NORMAL;
    private PIDController turretPID;

    private double targetVelocity = 0;
    private double baseHoodPos = hoodServoPos;
    private double lastVelErrorNorm = 0;
    private double lastVelTimeSec = 0;

    private boolean shooting = false;
    private boolean dipActive = false;
    private boolean dipDone = false;

    @Override
    public void opInit() {
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");

        distancez = hardwareMap.get(AnalogInput.class, "distance");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;
        outtake1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2Motor.setDirection(DcMotorEx.Direction.FORWARD);

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turretPID = new PIDController(turretKp, turretKi, turretKd);
        turretPID.setTolerance(1.0);

        hoodServo.setPosition(hoodServoPos);
        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);

        telemetry = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);

        velocityTimer.reset();
        lastVelTimeSec = 0;

        follower.followPath(paths.Path1, true);
        stateTimer.reset();
    }

    @Override
    public void opLoop() {
        follower.update();

        applyHoodAndVelocityRegressions();
        runFlywheelVelocityControl();
        aim();
        dipshot();

        switch (autonState) {

            case FOLLOW_PATH1:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                if (stateTimer.seconds() >= 0.5) {
                    endShoot();
                    follower.followPath(paths.Path2, true);
                    autonState = AutonState.FOLLOW_PATH2;
                }
                break;

            case FOLLOW_PATH2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    autonState = AutonState.FOLLOW_PATH3;
                }
                break;

            case FOLLOW_PATH3:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.SHOOT_3;
                }
                break;

            case SHOOT_3:
                if (stateTimer.seconds() >= 0.5) {
                    endShoot();
                    intakeMotor.setPower(1);
                    follower.followPath(paths.Path4, true);
                    autonState = AutonState.FOLLOW_PATH4;
                }
                break;

            case FOLLOW_PATH4:
                intakeMotor.setPower(1);
                checkSensorLock();
                if (!follower.isBusy()) {
                    intakePhase = IntakePhase.WAITING;
                    stateTimer.reset();
                    autonState = AutonState.INTAKE_WAIT_1;
                }
                break;

            case INTAKE_WAIT_1:
                if (runIntakeWait()) {
                    follower.followPath(paths.Path5, true);
                    autonState = AutonState.FOLLOW_PATH5;
                }
                break;

            case FOLLOW_PATH5:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.SHOOT_5;
                }
                break;

            case SHOOT_5:
                if (stateTimer.seconds() >= 0.5) {
                    endShoot();
                    intakeMotor.setPower(1);
                    follower.followPath(paths.Path6, true);
                    autonState = AutonState.FOLLOW_PATH6;
                }
                break;

            case FOLLOW_PATH6:
                intakeMotor.setPower(1);
                checkSensorLock();
                if (!follower.isBusy()) {
                    intakePhase = IntakePhase.WAITING;
                    stateTimer.reset();
                    autonState = AutonState.INTAKE_WAIT_2;
                }
                break;

            case INTAKE_WAIT_2:
                if (runIntakeWait()) {
                    follower.followPath(paths.Path7, true);
                    autonState = AutonState.FOLLOW_PATH7;
                }
                break;

            case FOLLOW_PATH7:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.SHOOT_7;
                }
                break;

            case SHOOT_7:
                if (stateTimer.seconds() >= 0.5) {
                    endShoot();
                    intakeMotor.setPower(1);
                    follower.followPath(paths.Path8, true);
                    autonState = AutonState.FOLLOW_PATH8;
                }
                break;

            case FOLLOW_PATH8:
                intakeMotor.setPower(1);
                checkSensorLock();
                if (!follower.isBusy()) {
                    intakePhase = IntakePhase.WAITING;
                    stateTimer.reset();
                    autonState = AutonState.INTAKE_WAIT_3;
                }
                break;

            case INTAKE_WAIT_3:
                if (runIntakeWait()) {
                    follower.followPath(paths.Path9, true);
                    autonState = AutonState.FOLLOW_PATH9;
                }
                break;

            case FOLLOW_PATH9:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.SHOOT_9;
                }
                break;

            case SHOOT_9:
                if (stateTimer.seconds() >= 0.5) {
                    endShoot();
                    follower.followPath(paths.Path10, true);
                    autonState = AutonState.FOLLOW_PATH10;
                }
                break;

            case FOLLOW_PATH10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, true);
                    autonState = AutonState.FOLLOW_PATH11;
                }
                break;

            case FOLLOW_PATH11:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.SHOOT_11;
                }
                break;

            case SHOOT_11:
                if (stateTimer.seconds() >= 0.5) {
                    endShoot();
                    follower.followPath(paths.Path12, true);
                    autonState = AutonState.FOLLOW_PATH12;
                }
                break;

            case FOLLOW_PATH12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path13, true);
                    autonState = AutonState.FOLLOW_PATH13;
                }
                break;

            case FOLLOW_PATH13:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.SHOOT_13;
                }
                break;

            case SHOOT_13:
                if (stateTimer.seconds() >= 0.5) {
                    endShoot();
                    autonState = AutonState.DONE;
                }
                break;

            case DONE:
                intakeMotor.setPower(0);
                outtake1Motor.setPower(0);
                outtake2Motor.setPower(0);
                turret.setPower(0);
                break;
        }

        telemetry.addData("Auton State", autonState);
        telemetry.addData("Sensor V", String.format("%.3f", distancez.getVoltage()));
        telemetry.addData("Flywheel V", String.format("%.0f", outtake2Motor.getVelocity()));
        telemetry.addData("Target V", String.format("%.0f", targetVelocity));
        telemetry.addData("Turret Deg", String.format("%.1f", getTurretAngleDeg()));
        telemetry.update();
    }

    private void startShoot() {
        holdServo.setPosition(holdOpenPos);
        leftpushServo.setPosition(Push3);
        rightpushServo.setPosition(Push3 - servooffset);
        shooting = true;
        dipActive = false;
        dipDone = false;
    }

    private void endShoot() {
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);
        holdServo.setPosition(holdClosePos);
        shooting = false;
    }

    private boolean runIntakeWait() {
        boolean sensorFired = distancez.getVoltage() < dthresh;

        switch (intakePhase) {
            case WAITING:
                intakeMotor.setPower(1);
                if (sensorFired) {
                    triggerSensorLock();
                    intakePhase = IntakePhase.REVERSING;
                    return false;
                }
                if (stateTimer.seconds() >= 1.0) {
                    intakeMotor.setPower(0);
                    return true;
                }
                return false;

            case REVERSING:
                if (intakeRevTimer.seconds() >= 1.0) {
                    intakeMotor.setPower(0);
                    leftpushServo.setPosition(Push0);
                    rightpushServo.setPosition(Push0 - servooffset);
                    return true;
                }
                return false;
        }
        return false;
    }

    private void triggerSensorLock() {
        leftpushServo.setPosition(lock);
        rightpushServo.setPosition(lock - servooffset);
        holdServo.setPosition(holdClosePos);
        intakeMotor.setPower(-1);
        intakeRevTimer.reset();
    }

    private void checkSensorLock() {
        if (distancez.getVoltage() < dthresh) {
            triggerSensorLock();
        }
    }

    private void applyHoodAndVelocityRegressions() {
        Pose poseNow = follower.getPose();
        if (poseNow == null) {
            baseHoodPos = hoodServoPos;
            targetVelocity = TV;
            return;
        }
        Pose vGoal = updateGoalV2(poseNow);
        double dist = Math.hypot(vGoal.getX() - poseNow.getX(), vGoal.getY() - poseNow.getY());

        double hoodPos = (hoodA * dist * dist) + (hoodB * dist) + hoodC;
        baseHoodPos = Math.max(0.0, Math.min(1.0, hoodPos));

        double vel = (velA * dist * dist) + (velB * dist) + velC;
        double maxVel = outtake2Motor.getMotorType().getMaxRPM()
                * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;
        targetVelocity = Math.max(-maxVel, Math.min(maxVel, vel));
    }

    private void dipshot() {
        if (shooting && !dipActive && !dipDone) {
            dipActive = true;
            dipTimer.reset();
        }
        if (!shooting) {
            dipActive = false;
            dipDone = false;
            hoodServo.setPosition(baseHoodPos);
            return;
        }
        if (dipActive) {
            double elapsed = dipTimer.seconds();
            if (elapsed < dipDelaySec) {
                hoodServo.setPosition(baseHoodPos);
            } else if (elapsed < dipDelaySec + dipDurationSec) {
                hoodServo.setPosition(Math.max(0.0, baseHoodPos - hoodDipDuringShot));
            } else {
                hoodServo.setPosition(baseHoodPos);
                dipActive = false;
                dipDone = true;
            }
        }
    }

    private Pose updateGoalV2(Pose robotPose) {
        com.pedropathing.math.Vector vel = follower.getVelocity();
        double vx = (vel != null) ? vel.getXComponent() : 0.0;
        double vy = (vel != null) ? vel.getYComponent() : 0.0;
        double dist = Math.hypot(goalx - robotPose.getX(), goaly - robotPose.getY());
        double shotTime = (timeA * dist * dist) + (timeB * dist) + timeC;
        return new Pose(goalx - (vx * shotTime), goaly - (vy * shotTime), 0);
    }

    private void aim() {
        double desired = overshoot();
        double targetAngleDeg;

        switch (turretState) {
            case NORMAL:
                if (Math.abs(desired - getTurretAngleDeg()) <= threshold) {
                    targetAngleDeg = desired;
                } else {
                    turretState = TurretState.CENTERING;
                    targetAngleDeg = 0.0;
                }
                break;
            case CENTERING:
                targetAngleDeg = 0.0;
                if (Math.abs(getTurretAngleDeg()) < 5.0) turretState = TurretState.NORMAL;
                break;
            default:
                targetAngleDeg = 0.0;
                break;
        }

        double current = getTurretAngleDeg();
        double errorDeg = angleWrap(targetAngleDeg - current);

        if (Math.abs(errorDeg) <= turretDeadbandDeg) {
            turret.setPower(0);
            return;
        }

        turretPID.setPID(turretKp, turretKi, turretKd);
        double pidOut = turretPID.calculate(current, targetAngleDeg);
        double ff = (Math.abs(errorDeg) > turretFFDeadbandDeg)
                ? Math.copySign(turretKs, errorDeg) : 0.0;
        double output = Math.max(-turretMaxPower, Math.min(turretMaxPower, pidOut + ff));
        turret.setPower(output);
    }

    private void runFlywheelVelocityControl() {
        if (Math.abs(targetVelocity) <= 1.0) {
            outtake1Motor.setPower(0);
            outtake2Motor.setPower(0);
            lastVelErrorNorm = 0.0;
            lastVelTimeSec = velocityTimer.seconds();
            return;
        }

        double currentVelocity = outtake2Motor.getVelocity();
        double maxVelocity = outtake2Motor.getMotorType().getMaxRPM()
                * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;

        double nowSec = velocityTimer.seconds();
        double dt = Math.max(1e-3, Math.min(0.1, nowSec - lastVelTimeSec));

        double normalizedError = (targetVelocity - currentVelocity) / maxVelocity;
        double pTerm = vkP * normalizedError;
        double dErr = (normalizedError - lastVelErrorNorm) / dt;
        double dTerm = Math.max(-vkDMax, Math.min(vkDMax, vkD * dErr));

        double feedforward = vkF * (targetVelocity / maxVelocity);
        feedforward *= 12.0 / Math.max(10.5, batteryVoltageSensor.getVoltage());

        double power = Math.max(-1.0, Math.min(1.0, pTerm + dTerm + feedforward));
        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);

        lastVelErrorNorm = normalizedError;
        lastVelTimeSec = nowSec;
    }

    private double getTurretAngleDeg() {
        double voltage = turretEncoder.getVoltage();
        double angle = (voltage / turretEncoder.getMaxVoltage()) * 360.0;
        return angleWrap(angle - turretZeroDeg);
    }

    private double getDesiredTurretAngleDeg() {
        Pose pose = follower.getPose();
        if (pose == null) return getTurretAngleDeg();
        Pose vGoal = updateGoalV2(pose);
        double fieldAngle = Math.atan2(vGoal.getY() - pose.getY(), vGoal.getX() - pose.getX());
        return angleWrap(Math.toDegrees(fieldAngle - pose.getHeading()) + turretHeadingOffsetDeg);
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

    @Override public void opLoopHook() {}

    @Override
    public void opTeardown() {

    }
}