package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;
import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "V2RED")
public class V2RED extends DbzOpMode {

    private enum TurretState {NORMAL, CENTERING}

    private ElapsedTime intaketimer = new ElapsedTime();
    private ElapsedTime threeBallTimer = new ElapsedTime();
    private ElapsedTime pushDelayTimer = new ElapsedTime();
    private ElapsedTime detectionTimer = new ElapsedTime();
    private boolean wasDetected = false;

    public static double targetX = 144;
    public static double targetY = 144;
    public static double threeBallHoldTime = 0.15;
    public static double holdToPushDelay = 0.25;

    private boolean atTTarget = false;
    private boolean atWTarget = false;

    private double lastLightPos = -1;
    public static double shot1 = 300;
    public static double shot2 = 600;
    public static double shotreturn = 1000;

    private boolean threeBallsLocked = false;
    private boolean holdOpened = false;

    public static double servooffset = 0.035;

    public static double Push0 = 0.85;
    public static double timeeiieiu = 0.25;
    public static double Push1 = 0.67;
    public static double Push2 = 0.47;
    public static double Push3 = 0.22;

    public static double lock = 0.71;

    public static double holdOpenPos = 0.8;
    public static double holdClosePos = 0.467;

    public static double dthresh = 0.147;
    public static double dthresh1 = 0.18;
    public static double dthresh2  = 0.15;

    public static double hoodDipDuringShot = 0.015;
    public static double dipDelaySec = 0.5;
    public static double dipDurationSec = 0.15;

    public static double velA = -0.0157003, velB = 11.6092, velC = 727.08688;

    public static double hoodA = -0.0000876693, hoodB = 0.0228448, hoodC = -0.779915;
    public static double timeA = 0.00002;
    public static double timeB = 0.004;
    public static double timeC = 0.25;

    public static double goalx = 144;
    public static double goaly = 144;

    public static double vkDMax = 0.25;

    public static double TV = 0;
    public static double hoodServoPos = 0.5;
    public static double vkP = 5;
    public static double vkF = 1.2;
    public static double vkD = 0.0;

    public static double threshold = 220;
    public static double threshold2 = 180;

    public static double turretZeroDeg = 181;

    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.001;
    public static double hang = 0.5;
    public static double hangdown = 0.5;

    public static double turretDeadbandDeg = 0.0;
    public static double turretMaxPower = 1.0;

    public static double turretKs = 0.00;
    public static double turretFFDeadbandDeg = 0.0;

    public static double turretoffset = 3.0;
    private AnalogInput distancez, distance1, distance2;

    private ElapsedTime velocityTimer = new ElapsedTime();
    private double lastVelErrorNorm = 0.0;
    private double lastVelTimeSec = 0.0;

    protected Servo rightpushServo, leftpushServo, hoodServo, holdServo, hang1, hang2;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;

    private VoltageSensor batteryVoltageSensor;
    private AnalogInput turretEncoder;

    protected DistanceSensor sensor1, sensor2, sensor3;

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

    private double turretTargetDegTelem = 0.0;
    private double turretCurrentDegTelem = 0.0;

    private double flyTargetVelTelem = 0.0;
    private double flyCurrentVelTelem = 0.0;

    private double baseHoodPos = hoodServoPos;
    protected Servo light;

    private boolean dipActive = false;
    private boolean dipDone = false;
    private ElapsedTime dipTimer = new ElapsedTime();

    private double maxVel = 1900;
    private boolean useRpmMaxVel = false;
    private boolean lastDpadUpG1 = false;
    private boolean lastDpadDownG1 = false;

    private boolean lastDpadUpG2 = false;
    private boolean lastDpadDownG2 = false;

    @Override
    public void opInit() {
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");
        hang1 = hardwareMap.get(Servo.class, "hang1");
        hang2 = hardwareMap.get(Servo.class, "hang2");

        light = hardwareMap.get(Servo.class, "light");

        sensor1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        sensor2 = hardwareMap.get(DistanceSensor.class, "sensor2");
        distancez = hardwareMap.get(AnalogInput.class, "distancez");
        distance1 = hardwareMap.get(AnalogInput.class, "distance1");
        distance2 = hardwareMap.get(AnalogInput.class, "distance2");

        baseHoodPos = hoodServoPos;
        hoodServo.setPosition(baseHoodPos);

        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);

        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;

        telemetry = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );
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

        velocityTimer.reset();
        lastVelTimeSec = velocityTimer.seconds();
        lastVelErrorNorm = 0.0;

        maxVel = 1900;
        useRpmMaxVel = false;

        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.update();
        follower.update();

        if (follower.getCurrentPath() != null) {
            drawOnlyCurrent();
        }

        follower.startTeleopDrive();
    }

    @Override
    public void opLoop() {
        updateLights();

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

        boolean dpadUpG1 = gamepad1.dpad_up;
        boolean dpadDownG1 = gamepad1.dpad_down;

        if (dpadUpG1 && !lastDpadUpG1) {
            useRpmMaxVel = true;
            maxVel = outtake2Motor.getMotorType().getMaxRPM()
                    * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;
        }
        if (dpadDownG1 && !lastDpadDownG1) {
            useRpmMaxVel = false;
            maxVel = 1900;
        }

        lastDpadUpG1 = dpadUpG1;
        lastDpadDownG1 = dpadDownG1;

        boolean dpadUpG2 = gamepad2.dpad_up;
        boolean dpadDownG2 = gamepad2.dpad_down;

        if (dpadUpG2 && !lastDpadUpG2) {
            holdServo.setPosition(holdOpenPos);
        }
        if (dpadDownG2 && !lastDpadDownG2) {
            holdServo.setPosition(holdClosePos);
        }

        lastDpadUpG2 = dpadUpG2;
        lastDpadDownG2 = dpadDownG2;

        boolean rightStickPress = gamepad2.right_stick_button;
        boolean leftStickPress = gamepad2.left_stick_button;

        if (rightStickPress && !lastr1) turretHeadingOffsetDeg -= turretoffset;
        if (leftStickPress && !lastl1) turretHeadingOffsetDeg += turretoffset;

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
        dipshot();

        checkThreeBallsAndLock();
        shootFastOnly();
        activeIntake();

        Pose ppose = follower.getPose();
        if (ppose != null) {
            double dx = targetX - ppose.getX();
            double dy = targetY - ppose.getY();
            double distance = Math.sqrt(dx * dx + dy * dy);
            telemetryM.addData("Distance", distance);
        } else {
            telemetryM.addData("Pose", "NULL - FOLLOWER NOT READY");
        }

        if (dbzGamepad1.x) {
            follower.setPose(new Pose(9.76378, 8.661, Math.toRadians(180)));
            turretHeadingOffsetDeg = 0.0;
        }
        if (dbzGamepad1.y) {
            follower.setPose(new Pose(144 - 9.76378, 8.661, Math.toRadians(0)));
            turretHeadingOffsetDeg = 0.0;
        }

        aim();
        runFlywheelVelocityControl();

        if (follower.getCurrentPath() != null) {
            draw();
        }

        sendGraphTelemetry();

        telemetryM.update(telemetry);
        telemetry.update();
    }

    private enum BallState {IDLE, REVERSING, LOCKED}

    private BallState ballState = BallState.IDLE;
    private ElapsedTime ballReverseTimer = new ElapsedTime();

    private void updateLights() {
        if (light == null) return;

        double newPos;

        if (threeBallsLocked) {
            newPos = 0.722;
        } else {
            newPos = 0.0;
        }

        if (newPos != 0.0 && newPos != 0.722) {
            newPos = 0.0;
        }

        newPos = Math.round(newPos * 1000.0) / 1000.0;

        if (Math.abs(lastLightPos - newPos) > 0.001) {
            light.setPosition(newPos);
            lastLightPos = newPos;
        }
    }

    private void checkThreeBallsAndLock() {
        double dist = distancez.getVoltage();
        double dist1 = distance1.getVoltage();
        double dist2 = distance2.getVoltage();

        boolean detected = dist < dthresh;
        boolean detected1 = dist1 < dthresh1;
        boolean detected2 = dist2 < dthresh2;

        telemetryM.addData("Distance Voltage", String.format("%.3f", dist));
        telemetryM.addData("Distance Voltage1", String.format("%.3f", dist1));
        telemetryM.addData("Distance Voltage2", String.format("%.3f", dist2));
        telemetryM.addData("Ball Detected", detected ? "YES" : "NO");
        telemetryM.addData("Ball Detected", detected1 ? "YES" : "NO");
        telemetryM.addData("Ball Detected", detected2 ? "YES" : "NO");
        telemetryM.addData("Ball State", ballState.name());

        switch (ballState) {

            case IDLE:
                if (detected && detected1 && detected2 && !shooting) {
                    if (!wasDetected) {
                        detectionTimer.reset();
                        ballReverseTimer.reset();
                        wasDetected = true;
                    }
                    gamepad1.rumble(1000);
                    holdServo.setPosition(holdClosePos);
                    leftpushServo.setPosition(lock);
                    rightpushServo.setPosition(lock - servooffset);
                    intakeMotor.setPower(-1);
                    ballReverseTimer.reset();
                    threeBallsLocked = true;
                    ballState = BallState.REVERSING;
                    wasDetected = false;
                } else {
                    wasDetected = false;
                    threeBallsLocked = false;
                }
                break;

            case REVERSING:
                if (!shooting && ballReverseTimer.seconds() < 1.0) {
                    holdServo.setPosition(holdClosePos);
                    leftpushServo.setPosition(lock);
                    rightpushServo.setPosition(lock - servooffset);
                    intakeMotor.setPower(-1);
                }

                if (ballReverseTimer.seconds() >= 1.0) {
                    intakeMotor.setPower(1);
                    ballState = BallState.LOCKED;
                }
                break;

            case LOCKED:
                if (!shooting) {
                    leftpushServo.setPosition(lock);
                    rightpushServo.setPosition(lock - servooffset);
                    intakeMotor.setPower(1);
                    wasDetected = false;
                }
                break;
        }
    }

    private void applyHoodAndVelocityRegressions() {
        Pose poseNow = follower.getPose();

        if (autoHoodActive && poseNow != null) {
            Pose vGoal = updateGoalV2(poseNow);
            double vDist = Math.hypot(vGoal.getX() - poseNow.getX(), vGoal.getY() - poseNow.getY());

            double hoodPos = (hoodA * vDist * vDist) + (hoodB * vDist) + hoodC;
            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
            baseHoodPos = hoodPos;

            double vel = (velA * vDist * vDist) + (velB * vDist) + velC;
            vel = Math.max(-maxVel, Math.min(maxVel, vel));
            targetVelocity = vel;

        } else if (!autoHoodActive) {
            baseHoodPos = hoodServoPos;
            targetVelocity = TV;
        } else {
            baseHoodPos = hoodServoPos;
            targetVelocity = 0.0;
        }
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

        double speed = Math.hypot(vx, vy);
        if (speed < 1.5) {
            vx = 0;
            vy = 0;
        }

        double dist = Math.hypot(goalx - robotPose.getX(), goaly - robotPose.getY());
        double shotTime = (timeA * dist * dist) + (timeB * dist) + timeC;

        double vGoalX = goalx - (vx * shotTime);
        double vGoalY = goaly - (vy * shotTime);

        return new Pose(vGoalX, vGoalY, 0);
    }

    private void shootFastOnly() {
        boolean rightTriggerHeld = dbzGamepad1.right_trigger > 0.1;
        boolean fastshoot = moveshoot;
        if (!fastshoot) {
            if (rightTriggerHeld && !shootLast && !shooting) {
                holdServo.setPosition(holdOpenPos);
                leftpushServo.setPosition(Push1);
                rightpushServo.setPosition(Push1 - servooffset);
                intaketimer.reset();
                shooting = true;
                threeBallsLocked = false;
                holdOpened = false;
                intaketimer.reset();
                shooting = true;
            }
            if (shooting && intaketimer.milliseconds() > shotreturn) {
                leftpushServo.setPosition(Push0);
                rightpushServo.setPosition(Push0 - servooffset);
                holdServo.setPosition(holdClosePos);
                shooting = false;
                resetAfterShooting();
            } else if (shooting && intaketimer.milliseconds() > shot2) {
                leftpushServo.setPosition(Push3);
                rightpushServo.setPosition(Push3 - servooffset);
            } else if (shooting && intaketimer.milliseconds() > shot1) {
                leftpushServo.setPosition(Push2);
                rightpushServo.setPosition(Push2 - servooffset);
            }
        } else {
            if (rightTriggerHeld && !shootLast && !shooting) {
                threeBallsLocked = false;
                holdOpened = false;

                holdServo.setPosition(holdOpenPos);

                leftpushServo.setPosition(Push3);
                rightpushServo.setPosition(Push3 - servooffset);

                intaketimer.reset();

                shooting = true;
                dipActive = false;
                dipDone = false;
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
        telemetry.addData("Fastshooting", fastshoot);
    }

    private void resetAfterShooting() {
        threeBallsLocked = false;
        holdOpened = false;
        ballState = BallState.IDLE;

        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);

        intakeForwardOn = true;
        intakeReverseOn = false;
        intakeMotor.setPower(1);
    }

    private void activeIntake() {
        if (ballState == BallState.REVERSING || ballState == BallState.LOCKED) {
            lastRightBumper = gamepad1.right_bumper;
            lastLeftBumper = gamepad1.left_bumper;
            return;
        }

        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        if (shooting) {
            intakeMotor.setPower(0);
            lastRightBumper = rb;
            lastLeftBumper = lb;
            return;
        }

        if (threeBallsLocked) {
            intakeMotor.setPower(-1);
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
            intakeMotor.setPower(1);
        } else if (intakeReverseOn) {
            intakeMotor.setPower(-1);
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

        turretTargetDegTelem = targetAngleDeg;
        turretCurrentDegTelem = currentAngleDeg;

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
        flyTargetVelTelem = targetVelocity;
        flyCurrentVelTelem = outtake2Motor.getVelocity();

        if (Math.abs(targetVelocity) <= 1.0) {
            outtake1Motor.setPower(0);
            outtake2Motor.setPower(0);
            atWTarget = false;
            lastVelErrorNorm = 0.0;
            lastVelTimeSec = velocityTimer.seconds();
            return;
        }

        double currentVelocity = flyCurrentVelTelem;

        double maxVelocity = outtake2Motor.getMotorType().getMaxRPM()
                * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;

        double nowSec = velocityTimer.seconds();
        double dt = nowSec - lastVelTimeSec;
        if (dt <= 0.0) dt = 1e-3;
        if (dt > 0.1) dt = 0.1;

        double normalizedError = (targetVelocity - currentVelocity) / maxVelocity;
        double pTerm = vkP * normalizedError;

        double dErr = (normalizedError - lastVelErrorNorm) / dt;
        double dTerm = vkD * dErr;
        if (dTerm > vkDMax) dTerm = vkDMax;
        if (dTerm < -vkDMax) dTerm = -vkDMax;

        double feedforward = vkF * (targetVelocity / maxVelocity);
        double batteryVoltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        feedforward *= 12.0 / batteryVoltage;

        double power = pTerm + dTerm + feedforward;
        power = Math.max(-1.0, Math.min(1.0, power));

        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);

        atWTarget = Math.abs(targetVelocity - currentVelocity) < 40.0;

        lastVelErrorNorm = normalizedError;
        lastVelTimeSec = nowSec;
    }

    private double getTurretAngleDeg() {
        double voltage = turretEncoder.getVoltage();
        double angle = (voltage / turretEncoder.getMaxVoltage()) * 360.0;
        angle -= turretZeroDeg;
        return angleWrapAsym(angle, threshold);
    }

    private double getDesiredTurretAngleDeg() {
        Pose pose = follower.getPose();
        if (pose == null) return getTurretAngleDeg();

        Pose vGoal = updateGoalV2(pose);

        double fieldAngle = Math.atan2(vGoal.getY() - pose.getY(), vGoal.getX() - pose.getX());
        double turretAngleDeg = Math.toDegrees(fieldAngle - pose.getHeading()) + turretHeadingOffsetDeg;

        return angleWrapAsym(turretAngleDeg, threshold);
    }

    private double overshoot() {
        double desired = angleWrapAsym(getDesiredTurretAngleDeg(), threshold);
        if (desired > threshold2) return threshold2;
        if (desired < -threshold) return -threshold;
        return desired;
    }

    private double angleWrap(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    private double angleWrapAsym(double angle, double negLimit) {
        return ((angle + negLimit) % 360 + 360) % 360 - negLimit;
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

    private void sendGraphTelemetry() {
        telemetry.addData("Turret/TargetDeg", turretTargetDegTelem);
        telemetry.addData("Turret/CurrentDeg", turretCurrentDegTelem);
        telemetry.addData("Turret/ErrorDeg", angleWrap(turretTargetDegTelem - turretCurrentDegTelem));
        telemetry.addData("Turret/AtTarget", atTTarget);

        telemetry.addData("Flywheel/TargetVel", flyTargetVelTelem);
        telemetry.addData("Flywheel/CurrentVel", flyCurrentVelTelem);
        telemetry.addData("Flywheel/VelError", flyTargetVelTelem - flyCurrentVelTelem);
        telemetry.addData("Flywheel/AtTarget", atWTarget);

        telemetry.addData("MaxVel Mode", useRpmMaxVel ? "RPM" : "1900");
        telemetry.addData("MaxVel", maxVel);
    }

    @Override
    public void opLoopHook() {}

    @Override
    public void opTeardown() {}
}