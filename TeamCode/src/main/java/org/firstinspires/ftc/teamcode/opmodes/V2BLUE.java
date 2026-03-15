package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;
import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
import org.firstinspires.ftc.teamcode.auton.ConstantsTele;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "V2BLUE")
public class V2BLUE extends DbzOpMode {

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
    public static double intakecurrentthresh = 8.5;

    private boolean threeBallsLocked = false;
    private boolean holdOpened = false;

    public static double servooffset = 0.035;

    public static double Push0 = 0.85;
    public static double timeeiieiu = 0.25;
    public static double Push1 = 0.67;
    public static double Push2 = 0.47;
    public static double Push3 = 0.22;

    public static double lock = 0.71;

    public static double sensorStickyWindow = 0.15;

    public static double holdOpenPos = 0.8;
    public static double holdClosePos = 0.467;

    public static double dthresh = 0.157;
    public static double dthresh1 = 0.173;
    public static double dthresh2  = 0.155;

    public static double hoodDipDuringShot = 0.0;
    public static double dipDelaySec = 0.5;
    public static double dipDurationSec = 0.15;

    public static double velA = -0.0157003, velB = 11.6092, velC = 727.08688;

    public static double hoodA = -0.0000876693, hoodB = 0.0228448, hoodC = -0.779915;
    public static double timeA = 0.00002;
    public static double timeB = 0.004;
    public static double timeC = 0.25;

    public static double goalx = 0;
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

    public static double turretKs = 0.01;
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

    private boolean aimingActive = true;
    private boolean leftTriggerLast = false;

    private boolean intakeForwardOn = false;
    private boolean intakeReverseOn = false;

    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    private PIDController turretPID;

    private boolean moveshoot = true;
    private boolean lastshoot = false;

    private double turretHeadingOffsetDeg = 0.0;
    private boolean lastr1 = false;
    private boolean lastl1 = false;
    private ElapsedTime sensorTimer0 = new ElapsedTime();
    private ElapsedTime sensorTimer1 = new ElapsedTime();
    private ElapsedTime sensorTimer2 = new ElapsedTime();

    private ElapsedTime currentTimer = new ElapsedTime();
    private boolean latch0 = false, latch1 = false, latch2 = false;

    private TurretState turretState = TurretState.NORMAL;

    public static Follower follower;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    private Limelight3A limelight;
    private boolean lastSnapButton = false;

    private double targetVelocity = 0.0;

    private boolean lastRightBumperG2 = false;
    private boolean lastLeftBumperG2 = false;
    private boolean g2IntakeForwardOn = false;
    private boolean g2IntakeReverseOn = false;

    private double turretTargetDegTelem = 0.0;
    private double turretCurrentDegTelem = 0.0;

    private double flyTargetVelTelem = 0.0;
    private double flyCurrentVelTelem = 0.0;

    private double baseHoodPos = hoodServoPos;
    protected Servo light;

    private boolean dipActive = false;
    private boolean dipDone = false;
    private ElapsedTime dipTimer = new ElapsedTime();

    // --- 3-ball hold-then-push delay ---
    private ElapsedTime holdDelayTimer = new ElapsedTime();
    private boolean waitingForHold = false;

    private double maxVel = 1900;
    private boolean useRpmMaxVel = false;
    private boolean lastDpadUpG1 = false;
    private boolean lastDpadDownG1 = false;

    private boolean lastDpadUpG2 = false;
    private boolean lastDpadDownG2 = false;

    private double desiredHoldPos = holdOpenPos;
    private boolean manualHoldOverride = false;

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

        desiredHoldPos = holdClosePos;
        holdServo.setPosition(desiredHoldPos);
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

        follower = ConstantsTele.createFollower(hardwareMap);
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
            manualHoldOverride = true;
            desiredHoldPos = holdOpenPos;
        }
        if (dpadDownG2 && !lastDpadDownG2) {
            manualHoldOverride = true;
            desiredHoldPos = holdClosePos;
        }

        lastDpadUpG2 = dpadUpG2;
        lastDpadDownG2 = dpadDownG2;

        boolean rightStickPress = gamepad2.right_bumper;
        boolean leftStickPress = gamepad2.right_bumper;

        if (rightStickPress && !lastr1) turretHeadingOffsetDeg -= turretoffset;
        if (leftStickPress && !lastl1) turretHeadingOffsetDeg += turretoffset;

        lastr1 = rightStickPress;
        lastl1 = leftStickPress;

        boolean snap = gamepad1.b;
        if (snap && !lastSnapButton) {
            trySnapPoseFromLimelight();
        }
        lastSnapButton = snap;

        double mult = gamepad1.left_trigger > 0.1 ? 0.3 : 1;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * mult,
                -gamepad1.left_stick_x * mult,
                -gamepad1.right_stick_x * mult,
                true
        );

        follower.update();

        applyHoodAndVelocityRegressions();
        dipshot();

        runBallDetection();
        shootFastOnly();
        activeIntake();
        updateHoldServoOutput();

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
            follower.setPose(new Pose(15, 111, Math.toRadians(90)));
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

        if(intakeReverseOn || g2IntakeReverseOn){
            newPos = 0.277;
        }

        newPos = Math.round(newPos * 1000.0) / 1000.0;

        if (Math.abs(lastLightPos - newPos) > 0.001) {
            light.setPosition(newPos);
            lastLightPos = newPos;
        }
    }

    private void runBallDetection() {
        double dist  = distancez.getVoltage();
        double dist1 = distance1.getVoltage();
        double dist2 = distance2.getVoltage();

        if (dist  < dthresh)  { latch0 = true; sensorTimer0.reset(); }
        if (dist1 < dthresh1) { latch1 = true; sensorTimer1.reset(); }
        if (dist2 < dthresh2) { latch2 = true; sensorTimer2.reset(); }

        if (sensorTimer0.seconds() > sensorStickyWindow) latch0 = false;
        if (sensorTimer1.seconds() > sensorStickyWindow) latch1 = false;
        if (sensorTimer2.seconds() > sensorStickyWindow) latch2 = false;

        boolean detected = latch0 && latch1 && latch2;

        switch (ballState) {
            case IDLE:
                if (detected && !shooting) {
                    if (!wasDetected) {
                        detectionTimer.reset();
                        wasDetected = true;
                    }
                    if (detectionTimer.seconds() >= 0.2) {
                        latch0 = false; latch1 = false; latch2 = false;
                        manualHoldOverride = false;
                        desiredHoldPos = holdClosePos;
                        leftpushServo.setPosition(lock);
                        rightpushServo.setPosition(lock - servooffset);
                        intakeMotor.setPower(-1);
                        ballReverseTimer.reset();
                        ballState = BallState.REVERSING;
                        wasDetected = false;
                        threeBallsLocked = true;
                    }
                } else if (!detected) {
                    wasDetected = false;
                    threeBallsLocked = false;
                }
                break;

            case REVERSING:
                manualHoldOverride = false;
                desiredHoldPos = holdClosePos;
                if (!shooting && ballReverseTimer.seconds() < 3.0) {
                    leftpushServo.setPosition(lock);
                    rightpushServo.setPosition(lock - servooffset);
                    intakeMotor.setPower(-1);
                }
                if (ballReverseTimer.seconds() >= 0.5) {
                    desiredHoldPos = holdOpenPos;
                }
                if (ballReverseTimer.seconds() >= 3.0) {
                    intakeMotor.setPower(-1);
                    leftpushServo.setPosition(Push0);
                    rightpushServo.setPosition(Push0 - servooffset);
                    ballState = BallState.LOCKED;
                }
                break;

            case LOCKED:
                leftpushServo.setPosition(lock);
                rightpushServo.setPosition(lock - servooffset);
                if (shooting) {
                    intakeMotor.setPower(1);
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
                // Capture 3-ball state and whether hold is already open BEFORE clearing anything
                boolean hadThreeBalls = threeBallsLocked;
                boolean holdAlreadyOpen = holdServo.getPosition() >= holdOpenPos - 0.01;

                manualHoldOverride = false;
                desiredHoldPos = holdOpenPos;
                intaketimer.reset();
                shooting = true;
                threeBallsLocked = false;
                holdOpened = false;
                dipActive = false;
                dipDone = false;

                if (hadThreeBalls && !holdAlreadyOpen) {
                    // Hold was closed — open it first, then wait 200 ms before pushing
                    waitingForHold = true;
                    holdDelayTimer.reset();
                } else {
                    // No 3-ball lock OR hold was already open — push immediately
                    waitingForHold = false;
                    leftpushServo.setPosition(Push1);
                    rightpushServo.setPosition(Push1 - servooffset);
                    intaketimer.reset(); // timing starts from the actual push
                }
            }

            if (shooting && waitingForHold) {
                // Hold servo is moving to open; push only after 200 ms
                if (holdDelayTimer.milliseconds() >= 200) {
                    waitingForHold = false;
                    leftpushServo.setPosition(Push1);
                    rightpushServo.setPosition(Push1 - servooffset);
                    intaketimer.reset(); // shot sequence timing starts now
                }
                // Don't advance push stages while still waiting
            } else if (shooting) {
                if (intaketimer.milliseconds() > shotreturn) {
                    leftpushServo.setPosition(Push0);
                    rightpushServo.setPosition(Push0 - servooffset);
                    desiredHoldPos = holdClosePos;
                    shooting = false;
                    resetAfterShooting();
                } else if (intaketimer.milliseconds() > shot2) {
                    leftpushServo.setPosition(Push3);
                    rightpushServo.setPosition(Push3 - servooffset);
                } else if (intaketimer.milliseconds() > shot1) {
                    leftpushServo.setPosition(Push2);
                    rightpushServo.setPosition(Push2 - servooffset);
                }
            }

        } else {
            // moveshoot path
            if (rightTriggerHeld && !shootLast && !shooting) {
                boolean hadThreeBalls = threeBallsLocked;
                boolean holdAlreadyOpen = holdServo.getPosition() >= holdOpenPos - 0.01;

                threeBallsLocked = false;
                holdOpened = false;
                manualHoldOverride = false;
                desiredHoldPos = holdOpenPos;
                intaketimer.reset();
                shooting = true;
                dipActive = false;
                dipDone = false;

                if (hadThreeBalls && !holdAlreadyOpen) {
                    waitingForHold = true;
                    holdDelayTimer.reset();
                } else {
                    waitingForHold = false;
                    leftpushServo.setPosition(Push3);
                    rightpushServo.setPosition(Push3 - servooffset);
                    intaketimer.reset();
                }
            }

            if (shooting && waitingForHold) {
                if (holdDelayTimer.milliseconds() >= 200) {
                    waitingForHold = false;
                    leftpushServo.setPosition(Push3);
                    rightpushServo.setPosition(Push3 - servooffset);
                    intaketimer.reset();
                }
            } else if (shooting && intaketimer.milliseconds() > 700) {
                leftpushServo.setPosition(Push0);
                rightpushServo.setPosition(Push0 - servooffset);
                desiredHoldPos = holdClosePos;
                shooting = false;
                resetAfterShooting();
            }

            if (!shooting) {
                intaketimer.reset();
            }

            shootLast = rightTriggerHeld;
        }

        shootLast = rightTriggerHeld;
        telemetry.addData("Fastshooting", fastshoot);
        telemetry.addData("WaitingForHold", waitingForHold);
    }

    private void resetAfterShooting() {

        threeBallsLocked = false;
        holdOpened = false;
        ballState = BallState.IDLE;
        manualHoldOverride = false;
        desiredHoldPos = holdClosePos;
        waitingForHold = false;
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);

        g2IntakeForwardOn = false;
        g2IntakeReverseOn = false;

        intakeForwardOn = true;
        intakeReverseOn = false;


        intakeMotor.setPower(1);
    }

    private void activeIntake() {

        boolean rb2 = gamepad2.right_bumper;
        boolean lb2 = gamepad2.left_bumper;

        if (rb2 && !lastRightBumperG2) {
            g2IntakeForwardOn = !g2IntakeForwardOn;
            g2IntakeReverseOn = false;
        }
        if (lb2 && !lastLeftBumperG2) {
            g2IntakeReverseOn = !g2IntakeReverseOn;
            g2IntakeForwardOn = false;
        }
        lastRightBumperG2 = rb2;
        lastLeftBumperG2 = lb2;

        if (g2IntakeForwardOn) {
            intakeMotor.setPower(1);
            lastRightBumper = gamepad1.right_bumper;
            lastLeftBumper = gamepad1.left_bumper;
            return;
        } else if (g2IntakeReverseOn) {
            intakeMotor.setPower(-1);
            lastRightBumper = gamepad1.right_bumper;
            lastLeftBumper = gamepad1.left_bumper;
            return;
        }

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

    private void updateHoldServoOutput() {
        if (!manualHoldOverride) {
            if (shooting) {
                desiredHoldPos = holdOpenPos;
            } else if (ballState == BallState.REVERSING) {
                if (ballReverseTimer.seconds() >= 0.5) {
                    desiredHoldPos = holdOpenPos;
                } else {
                    desiredHoldPos = holdClosePos;
                }
            } else if (ballState == BallState.LOCKED) {
                desiredHoldPos = holdOpenPos;
            }
        }
        holdServo.setPosition(desiredHoldPos);
    }

    private void aim() {
        boolean leftTriggerPressed = gamepad1.dpad_right;
        if (leftTriggerPressed && !leftTriggerLast) {
            aimingActive = !aimingActive;
            turretPID.reset();
        }
        leftTriggerLast = gamepad1.dpad_right;

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
        telemetry.addData("Intake Current", String.format("%.2f", intakeMotor.getCurrent(CurrentUnit.AMPS)));

        telemetry.addData("Flywheel/TargetVel", flyTargetVelTelem);
        telemetry.addData("Flywheel/CurrentVel", flyCurrentVelTelem);
        telemetry.addData("Flywheel/VelError", flyTargetVelTelem - flyCurrentVelTelem);
        telemetry.addData("Flywheel/AtTarget", atWTarget);

        telemetry.addData("MaxVel Mode", useRpmMaxVel ? "RPM" : "1900");
        telemetry.addData("MaxVel", maxVel);
        telemetry.addData("Hold/Desired", desiredHoldPos);
        telemetry.addData("Hold/ManualOverride", manualHoldOverride);
        telemetry.addData("Hold/BallState", ballState);
    }

    @Override
    public void opLoopHook() {}

    @Override
    public void opTeardown() {}
}