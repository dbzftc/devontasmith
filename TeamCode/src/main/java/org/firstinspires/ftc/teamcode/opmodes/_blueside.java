package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;

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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;


@Config
@TeleOp(name = "_blueside")
public class _blueside extends DbzOpMode {
    private ElapsedTime intaketimer = new ElapsedTime();
    private ElapsedTime threeBallTimer = new ElapsedTime();
    private ElapsedTime pushDelayTimer = new ElapsedTime();

    public static double threeBallHoldTime = 0.01;
    public static double holdToPushDelay  = 0.25;
    private boolean atTTarget = false;
    private boolean atWTarget = false;

    private boolean threeBallsLocked = false;
    private boolean holdOpened = false;
    public static final double LIGHT_GREEN  = 0.5;
    public static final double LIGHT_PURPLE = 0.722;
    public static final double LIGHT_OFF    = 0.0;

    private double lastLightPos = -1;
    private double lastLight2Pos = -1;

    public static double targetX = 0; // Matching Blue Side
    public static double targetY = 144;

    public static double targetVelocity = -500;
    public static double vkP = 4.8;
    public static double vkF = 1.26;

    public static double servooffset = 0.023;
    public static double Push0 = 0.06;
    public static double Push1 = 0.4;
    public static double Push2 = 0.6;
    public static double Push3 = 0.66;
    public static double lock = 0.15;
    public static double shot1 = 250;
    public static double shot2 = 500;
    public static double shotreturn = 750;
    public static double hoffsettime1 = shot1-100;
    public static double hoffsettime2 = shot2-100;
    public static double choffset1 = 0.0;
    public static double choffset2 = 0.0;
    public static double fhoffset1 = 0.00;
    public static double fhoffset2 = 0.0;

    public static double dtoffset = 2.0;

    public static double shotLeadTime = 0.62;

    public static double hoodServoPos = 0.33;

    public static double holdOpenPos = 0.2;
    public static double holdClosePos = 0.05;

    public static double TV = 0;

    public static double threshold = 90;
    public static double threshold2 = 160;


    public static double turretZeroDeg =295;

    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.002;

    public static double turretDeadbandDeg = 0.0;
    public static double turretMaxPower = 1;

    public static double turretKs = 0.0;
    public static double turretFFDeadbandDeg = 0.0;

    public static double turretPivotForwardIn = 0.0;
    public static double turretPivotLeftIn = 0.0;

    public static double startX = 8;
    public static double startY = 8.5;
    public static double startHeadingDeg = 0.0;

    private Pose lastPose = new Pose();
    private double lastTime = 0;
    private Pose currentVelocity = new Pose();
    private ElapsedTime velocityTimer = new ElapsedTime();

    protected Servo rightpushServo, leftpushServo, hoodServo, holdServo;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private DcMotorEx motor1, motor2;

    private VoltageSensor batteryVoltageSensor;
    private AnalogInput turretEncoder;

    private boolean shootLast = false;
    private boolean shooting = false;

    private boolean autoHoodActive = true;
    private boolean lastAButton = false;

    private boolean aimingActive = false;
    private boolean leftTriggerLast = false;

    boolean intakeForwardOn = false;
    boolean intakeReverseOn = false;

    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    private PIDController turretPID;

    private boolean moveshoot = false;
    private boolean lastshoot = false;

    private double turretHeadingOffsetDeg = 0.0;
    public static double turretoffset = 3.0;

    private boolean lastr1 = false;
    private boolean lastl1 = false;


    private enum TurretState {
        NORMAL,
        CENTERING
    }

    private TurretState turretState = TurretState.NORMAL;

    public static Follower follower;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;
    protected DistanceSensor sensor1, sensor2;
    protected Servo light, light2;

    public static double dthresh = 4.0;


    @Override
    public void opInit() {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtake1Motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2Motor");

        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");

        sensor1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        sensor2 = hardwareMap.get(DistanceSensor.class, "sensor2");

        light = hardwareMap.get(Servo.class, "light");
        light2 = hardwareMap.get(Servo.class, "light2");

        hoodServo.setPosition(hoodServoPos);
        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0-servooffset);

        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor1.setDirection(DcMotorEx.Direction.REVERSE);

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
        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.update();
        if (follower.getCurrentPath() != null) {
            drawOnlyCurrent();
        }

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

        Pose ppose = follower.getPose();


        if(autoHoodActive) {
            if (ppose != null) {
                double dx = targetX - ppose.getX();
                double dy = targetY - ppose.getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (distance >= 125) {

//                    double originalPos = 0.000316354 * Math.pow(distance, 2)
//                            - 0.0843748 * distance
//                            + 6.1213;
//                    targetVelocity = (0.0502566 * distance * distance
//                            - 25.86373 * distance
//                            + 886.53277);
                    double originalPos = 0.0000054648 * Math.pow(distance, 3)
                            - 0.00256972 * distance * distance
                            + 0.406551 * distance
                            - 21.06598;
                    targetVelocity = (-0.0421989 * distance * distance
                            + 4.87989 * distance
                            - 1583.24308);

                    if (!shooting) {
                        hoodServo.setPosition(originalPos);
                    } else {
                        if (intaketimer.milliseconds() > hoffsettime2) {
                            hoodServo.setPosition(originalPos - fhoffset2);
                        } else if (intaketimer.milliseconds() > hoffsettime1) {
                            hoodServo.setPosition(originalPos - fhoffset1);
                        } else {
                            hoodServo.setPosition(originalPos);
                        }
                    }
                } else if (distance < 125) {
                    double originalPos = 0.00000326247 * Math.pow(distance, 3)
                            - 0.000953594 * Math.pow(distance, 2)
                            + 0.0932128 * distance
                            - 2.53108;
                    targetVelocity = (0.0381071 * distance * distance
                            - 11.86256 * distance
                            - 717.83856);
                    if (!shooting) {
                        hoodServo.setPosition(originalPos);
                    } else {
                        if (intaketimer.milliseconds() > hoffsettime2) {
                            hoodServo.setPosition(originalPos - choffset2);
                        } else if (intaketimer.milliseconds() > hoffsettime1) {
                            hoodServo.setPosition(originalPos - choffset1);
                        } else {
                            hoodServo.setPosition(originalPos);
                        }
                    }
                }
            } else {
                hoodServo.setPosition(hoodServoPos);
                targetVelocity = 0;
            }
        } else {
            hoodServo.setPosition(hoodServoPos);
            targetVelocity = TV;
        }
//
//        if(autoHoodActive) {
//            if (ppose != null) {
//                double dx = targetX - ppose.getX();
//                double dy = targetY - ppose.getY();
//                double distance = Math.sqrt(dx * dx + dy * dy);
//                if (distance >= 125) {
//                    double originalPos = 0.0000054648 * Math.pow(distance, 3)
//                            - 0.00256972 * distance * distance
//                            + 0.406551 * distance
//                            - 21.06598;
//                    targetVelocity = (-0.0421989 * distance * distance
//                            + 4.87989 * distance
//                            - 1583.24308);
//                    if (!shooting) {
//                        hoodServo.setPosition(originalPos);
//                    } else {
//                        if (intaketimer.milliseconds() > hoffsettime2) {
//                            hoodServo.setPosition(originalPos - fhoffset2);
//                        } else if (intaketimer.milliseconds() > hoffsettime1) {
//                            hoodServo.setPosition(originalPos - fhoffset1);
//                        } else {
//                            hoodServo.setPosition(originalPos);
//                        }
//                    }
//                } else if (distance < 125) {
//                    double originalPos = 0.00000124862 * Math.pow(distance, 3)
//                            - 0.000339071 * Math.pow(distance, 2)
//                            + 0.0314788 * distance
//                            - 0.606489;
//                    targetVelocity = (-0.00252214*Math.pow(distance, 3)
//                            +0.578787*distance*distance
//                            -47.86867*distance
//                            +37.52654);
//                    if (!shooting) {
//                        hoodServo.setPosition(originalPos);
//                    } else {
//                        if (intaketimer.milliseconds() > hoffsettime2) {
//                            hoodServo.setPosition(originalPos - choffset2);
//                        } else if (intaketimer.milliseconds() > hoffsettime1) {
//                            hoodServo.setPosition(originalPos - choffset1);
//                        } else {
//                            hoodServo.setPosition(originalPos);
//                        }
//                    }
//                }
//            } else {
//                hoodServo.setPosition(hoodServoPos);
//                targetVelocity = 0;
//            }
//        } else {
//            hoodServo.setPosition(hoodServoPos);
//            targetVelocity = TV;
//        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        updateLights();
        checkThreeBallsAndLock();
        updateVelocity();
        shoot();
        activeIntake();

        if (dbzGamepad1.x) {
            follower.setPose(new Pose(8, 8.5, Math.toRadians(180)));
            turretHeadingOffsetDeg = 0.0;
        }
        if (dbzGamepad1.y){
            follower.setPose(new Pose(136, 8.5, Math.toRadians(0)));
            turretHeadingOffsetDeg = 0.0;
        }

        follower.update();
        aim();
        runFlywheelVelocityControl();

        if (follower.getCurrentPath() != null) {
            draw();
        }

        addDebugTelemetry();

        telemetryM.update(telemetry);
        telemetry.update();
    }

    private void checkThreeBallsAndLock() {
        double dist1 = sensor1.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
        double dist2 = sensor2.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
        boolean detected1 = dist1 < dthresh;
        boolean detected2 = dist2 < dthresh;
        boolean ballDetected = detected1 || detected2;

        telemetryM.addData("=== PROXIMITY SENSORS ===", "");
        telemetryM.addData("Sensor 1 (cm)", String.format("%.2f", dist1));
        telemetryM.addData("Sensor 1 Detected", detected1 ? "YES" : "NO");
        telemetryM.addData("Sensor 2 (cm)", String.format("%.2f", dist2));
        telemetryM.addData("Sensor 2 Detected", detected2 ? "YES" : "NO");

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
            holdServo.setPosition(holdClosePos);

            threeBallsLocked = true;
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

    private void updateLights() {
        if (light == null || light2 == null) return;

        double newPos;

        if (threeBallsLocked) {
            newPos = 0.722;
        }
        else {
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

        if (Math.abs(lastLight2Pos - newPos) > 0.001) {
            light2.setPosition(newPos);
            lastLight2Pos = newPos;
        }
    }

    private void updateVelocity() {
        Pose currentPose = follower.getPose();
        double currentTime = velocityTimer.seconds();

        if (currentPose == null || currentTime - lastTime < 0.005) return;

        double dt = currentTime - lastTime;

        double dx = currentPose.getX() - lastPose.getX();
        double dy = currentPose.getY() - lastPose.getY();

        double vx = dx / dt;
        double vy = dy / dt;

        currentVelocity = new Pose(vx, vy, 0);

        lastPose = currentPose;
        lastTime = currentTime;
    }

    private void addDebugTelemetry() {
        Pose pose = follower.getPose();

        telemetryM.addData("shooting: ", shooting);
        telemetryM.addData("=== TARGET ===", "");
        telemetryM.addData("Target X", String.format("%.2f", targetX));
        telemetryM.addData("Target Y", String.format("%.2f", targetY));

        telemetryM.addData("=== CURRENT POSITION ===", "");
        if (pose != null) {
            telemetryM.addData("Current X", String.format("%.2f", pose.getX()));
            telemetryM.addData("Current Y", String.format("%.2f", pose.getY()));
            telemetryM.addData("Heading (rad)", String.format("%.3f", pose.getHeading()));
            telemetryM.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(pose.getHeading())));

            double dx = targetX - pose.getX();
            double dy = targetY - pose.getY();
            double distance = Math.sqrt(dx * dx + dy * dy);
            telemetryM.addData("Distance to Target", String.format("%.2f", distance));
        } else {
            telemetryM.addData("Pose", "NULL - FOLLOWER NOT READY");
        }


        telemetryM.addData("=== TURRET ENCODER ===", "");
        telemetryM.addData("Encoder Voltage", String.format("%.3f", turretEncoder.getVoltage()));
        telemetryM.addData("Max Voltage", String.format("%.3f", turretEncoder.getMaxVoltage()));
        telemetryM.addData("Turret Zero Deg", turretZeroDeg);


        telemetryM.addData("=== TURRET STATE ===", "");
        telemetryM.addData("Aiming Active", aimingActive ? "YES (Left Trigger)" : "NO (Press Left Trigger)");
        telemetryM.addData("Turret State", turretState.toString());
        telemetryM.addData("Current Angle (deg)", String.format("%.2f", getTurretAngleDeg()));
        telemetryM.addData("Desired Angle (deg)", String.format("%.2f", getDesiredTurretAngleDeg()));
        telemetryM.addData("Clamped Angle (deg)", String.format("%.2f", overshoot()));

        telemetryM.addData("=== TURRET PID ===", "");
        telemetryM.addData("Kp", turretKp);
        telemetryM.addData("Ki", turretKi);
        telemetryM.addData("Kd", turretKd);
        telemetryM.addData("Ks (FF)", turretKs);
        telemetryM.addData("Deadband (deg)", turretDeadbandDeg);
        telemetryM.addData("Max Power", turretMaxPower);
        telemetryM.addData("Threshold (deg)", threshold);

        telemetryM.addData("=== TURRET POWER ===", "");
        telemetryM.addData("Turret Motor Power", String.format("%.3f", turret.getPower()));

        telemetryM.addData("=== CONTROLS ===", "");
        telemetryM.addData("Left Trigger", String.format("%.2f", dbzGamepad1.left_trigger));
        telemetryM.addData("Right Trigger", String.format("%.2f", dbzGamepad1.right_trigger));

        telemetryM.addData("=== TURRET PIVOT ===", "");
        telemetryM.addData("Pivot Forward (in)", turretPivotForwardIn);
        telemetryM.addData("Pivot Left (in)", turretPivotLeftIn);

        if (pose != null) {
            double heading = pose.getHeading();
            double offX = turretPivotForwardIn * Math.cos(heading) - turretPivotLeftIn * Math.sin(heading);
            double offY = turretPivotForwardIn * Math.sin(heading) + turretPivotLeftIn * Math.cos(heading);
            double pivotX = pose.getX() + offX;
            double pivotY = pose.getY() + offY;

            telemetryM.addData("Pivot X", String.format("%.2f", pivotX));
            telemetryM.addData("Pivot Y", String.format("%.2f", pivotY));
        }
    }

    private void shoot() {
        boolean rightTriggerHeld = dbzGamepad1.right_trigger > 0.1;
        boolean fastshoot = moveshoot;
        if (!fastshoot) {
            if (rightTriggerHeld && !shootLast && !shooting) {
                holdServo.setPosition(holdOpenPos);
                leftpushServo.setPosition(Push1);
                rightpushServo.setPosition(Push1 - servooffset);
                intaketimer.reset(); shooting = true;
            }
            if (shooting && intaketimer.milliseconds() > shotreturn) {
                leftpushServo.setPosition(Push0);
                rightpushServo.setPosition(Push0 - servooffset);
                holdServo.setPosition(holdClosePos);
                shooting = false; resetAfterShooting();
            } else if (shooting && intaketimer.milliseconds() > shot2) {
                leftpushServo.setPosition(Push3);
                rightpushServo.setPosition(Push3 - servooffset);
            } else if (shooting && intaketimer.milliseconds() > shot1) {
                leftpushServo.setPosition(Push2);
                rightpushServo.setPosition(Push2 - servooffset);
            }
        } else {
            if (rightTriggerHeld && !shootLast && !shooting) {
                holdServo.setPosition(holdOpenPos);
                leftpushServo.setPosition(Push3);
                rightpushServo.setPosition(Push3 - servooffset);
                intaketimer.reset(); shooting = true;
            } if (shooting && intaketimer.milliseconds() > 700) {
                leftpushServo.setPosition(Push0);
                rightpushServo.setPosition(Push0 - servooffset);
                holdServo.setPosition(holdClosePos);
                shooting = false; resetAfterShooting();
            }
        }
        shootLast = rightTriggerHeld;
        telemetryM.addData("slow shooting: ", !moveshoot);
        telemetryM.addData("fast shooting: ", moveshoot);
    }


    private void activeIntake() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        // 1) Shooting always wins: intake OFF
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
        atTTarget = Math.abs(errorDeg) < 2;

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

        telemetryM.addData("Turret Status", "MOVING");
        telemetryM.addData("Target Angle (deg)", String.format("%.2f", targetAngleDeg));
        telemetryM.addData("Error (deg)", String.format("%.2f", errorDeg));
        telemetryM.addData("PID Output", String.format("%.4f", pidOut));
        telemetryM.addData("FF Output", String.format("%.4f", ff));
        telemetryM.addData("Total Output", String.format("%.4f", output));
    }

    private void runFlywheelVelocityControl() {
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

        atWTarget = Math.abs(targetVelocity - currentVelocity) < 40;

        telemetry.addData("Flywheel Target Velocity", targetVelocity);
        telemetry.addData("Flywheel Actual Velocity", currentVelocity);

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

        double vx = currentVelocity.getX();
        double vy = currentVelocity.getY();

        double futureX = pose.getX() + (moveshoot ? vx * shotLeadTime : 0);
        double futureY = pose.getY() + (moveshoot ? vy * shotLeadTime : 0);

        double fieldAngle = Math.atan2(targetY - futureY, targetX - futureX);
        double turretAngleDeg = Math.toDegrees(fieldAngle - pose.getHeading());

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        if (distance >= 125) {
            turretAngleDeg += turretHeadingOffsetDeg  - dtoffset;
        } else {
            turretAngleDeg += turretHeadingOffsetDeg;
        }

        return angleWrap(turretAngleDeg);
    }


    private double overshoot() {
        double desired = angleWrap(getDesiredTurretAngleDeg());
        if (desired > threshold2) {
            return threshold2;
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
    public void opLoopHook() {}

    @Override
    public void opTeardown() {}
}