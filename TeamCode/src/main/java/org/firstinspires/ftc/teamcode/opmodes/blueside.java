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
@TeleOp(name = "blueside")
public class blueside extends DbzOpMode {
    private ElapsedTime intaketimer = new ElapsedTime();
    private ElapsedTime lighttimer = new ElapsedTime();

    public static double targetX = 0;
    public static double targetY = 144;

    public static double targetVelocity = -500;
    public static double vkP = 4.8;
    public static double vkF = 1.25;


    public static double shotLeadTime = 0.8;

    public static double hoodServoPos = 0.33;

    public static double holdOpenPos = 0.2;
    public static double holdClosePos = 0.1;

    public static double TV = -1400;

    public static double threshold = 130;
    public static double offsetDistance = 110.0;
    public static double offsetAngle = 0;

    public static double turretZeroDeg =295;

    public static double leftPushShoot = 0.66;
    public static double rightPushShoot = 0.69;

    public static double leftPushIdle = 0.06;
    public static double rightPushIdle = 0.09;

    public static double turretKp = 0.017;
    public static double turretKi = 0.0;
    public static double turretKd = 0.001;

    public static double turretDeadbandDeg = 0.0;
    public static double turretMaxPower = 0.3;


    public static double turretKs = 0.0; // SET TO 0 AS REQUESTED
    public static double turretFFDeadbandDeg = 0.0;

    public static double turretPivotForwardIn = 0.0;
    public static double turretPivotLeftIn = 0.0;

    public static double startX = 8;
    public static double startY = 8.5;
    public static double startHeadingDeg = 0.0;
    // Add these with your other variables
    private Pose lastPose = new Pose();
    private double lastTime = 0;
    private Pose currentVelocity = new Pose();
    private ElapsedTime velocityTimer = new ElapsedTime();

    protected Servo rightpushServo, leftpushServo, hoodServo, holdServo;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor, frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx motor1, motor2;

    private VoltageSensor batteryVoltageSensor;
    private AnalogInput turretEncoder;

    private boolean shootLast = false;
    private boolean shooting = false;

    private boolean Shooting = false;
    private boolean ShootLast = false;
    public static double hoodoffsettime = 100;
    public static double hoodoffset = 0.05;
    private boolean hoodOffsetActive = false;


    private boolean autoHoodActive = false;
    private boolean lastAButton = false;

    private boolean aimingActive = false;
    private boolean leftTriggerLast = false;

    boolean intakeForwardOn = false;
    boolean intakeReverseOn = false;

    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    private PIDController turretPID;

    private boolean lastUp2 = false;
    private boolean lastDown2 = false;
    private boolean lastLeft2 = false;
    private boolean lastRight2 = false;


    private enum TurretState {
        NORMAL,
        CENTERING
    }

    private TurretState turretState = TurretState.NORMAL;

    public static Follower follower;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;
    protected DistanceSensor sensor1, sensor2;
    protected Servo light;

    public static double dthresh = 2.0;

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

        // If 'light' is also throwing errors, initialize it here too:
        light = hardwareMap.get(Servo.class, "light");



        hoodServo.setPosition(hoodServoPos);
        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(leftPushIdle);
        rightpushServo.setPosition(rightPushIdle);

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

        // TURRET MOTOR INITIALIZATION - FIXED
        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD); // Adjust if needed

        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        //follower = Constants.createFollower(hardwareMap);


        follower.setStartingPose(org.firstinspires.ftc.teamcode.opmodes.PoseCache.lastPose);

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

        boolean aButton = gamepad1.a;
        if (aButton && !lastAButton) {
            autoHoodActive = !autoHoodActive;
        }
        lastAButton = aButton;

        Pose ppose = follower.getPose();


        double dist1 = sensor1.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
        double dist2 = sensor2.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);


        boolean detected1 = dist1 < dthresh;
        boolean detected2 = dist2 < dthresh;

        telemetryM.addData("=== PROXIMITY SENSORS ===", "");
        telemetryM.addData("Sensor 1 (cm)", String.format("%.2f", dist1));
        telemetryM.addData("Sensor 1 Detected", detected1 ? "YES" : "NO");
        telemetryM.addData("Sensor 2 (cm)", String.format("%.2f", dist2));
        telemetryM.addData("Sensor 2 Detected", detected2 ? "YES" : "NO");

        if (detected1 || detected2) {
            light.setPosition(0.5);
            lighttimer.reset();// Example: Set to a specific color (e.g., Green or Yellow)
        } else {
            light.setPosition(0.0); // Off or Default color
        }

        if(lighttimer.seconds()>3){
            lighttimer.reset();
        }
//
//        boolean RightTriggerHeld = dbzGamepad1.right_trigger > 0.1;
//
//        if (RightTriggerHeld && !ShootLast && !Shooting) {
//            intaketimer.reset();
//            Shooting = true;
//        }
//
//        if (Shooting && intaketimer.milliseconds() > hoodoffsettime) {
//            hoodOffsetActive = true;
//            Shooting = false;
//        }
//
//        if (!RightTriggerHeld && ShootLast) {
//            hoodOffsetActive = false;
//        }
//
//        ShootLast = RightTriggerHeld;


        if (autoHoodActive && !Shooting) {
            if (ppose != null) {
                double dx = targetX - ppose.getX();
                double dy = targetY - ppose.getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (distance >= 115) {
//                    // Original Far Range Math
//                    double originalPos = -0.000000217243 * Math.pow(distance, 3)
//                            + 0.0000386489 * Math.pow(distance, 2)
//                            + 0.00297592 * distance
//                            + 0.413722;
//                    // Shifting the 0.7 start down to 0.3 (0.7 - 0.4 = 0.3)
//                    hoodServo.setPosition(originalPos-0.4);
//                    targetVelocity = (-0.0111536 * distance * distance
//                            - 4.00719 * distance
//                            - 1097.37524);
                    double originalPos = 0.000316354 * Math.pow(distance, 2)
                            -0.0843748 * distance
                            +6.1213;
                    // Shifting the 0.7 start down to 0.3 (0.7 - 0.4 = 0.3)
                    hoodServo.setPosition(originalPos);
                    targetVelocity = (0.0502566 * distance * distance
                            -25.86373 * distance
                            +886.53277);
                } else if (distance < 115) {
                    // Original close Range Math
                    double originalPos = 0.00000326247 * Math.pow(distance, 3)
                            -0.000953594 * Math.pow(distance, 2)
                            + 0.0932128 * distance
                            -2.53108;
                    // Shifting the 0.7 start down to 0.3 (0.7 - 0.4 = 0.3)
                    hoodServo.setPosition(originalPos);
                    targetVelocity = (0.0381071 * distance * distance
                            -11.86256 * distance
                            -717.83856);
                }
                //the two lines below are for regression
//                targetVelocity = TV;
//                hoodServo.setPosition(hoodServoPos);
            } else {
                hoodServo.setPosition(hoodServoPos);
                targetVelocity = 0;
            }
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        updateVelocity();
        shoot();
        activeIntake();

        if (dbzGamepad1.x) {
            follower.setPose(new Pose(8, 8.5, Math.toRadians(180)));
        }
        if (dbzGamepad1.y){
            follower.setPose(new Pose(132, 8.5, Math.toRadians(0)));
        }

        if (gamepad2.dpad_up && !lastUp2) {
            targetY += 1.0;
        }
        lastUp2 = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDown2) {
            targetY -= 1.0;
        }
        lastDown2 = gamepad2.dpad_down;


        if (gamepad2.dpad_right && !lastRight2) {
            targetX += 1.0;
        }
        lastRight2 = gamepad2.dpad_right;

        if (gamepad2.dpad_left && !lastLeft2) {
            targetX -= 1.0;
        }
        lastLeft2 = gamepad2.dpad_left;



        follower.update();
        aim();

        runFlywheelVelocityControl();

        if (follower.getCurrentPath() != null) {
            draw();
        }

        // COMPREHENSIVE TELEMETRY
        addDebugTelemetry();

        telemetryM.update(telemetry);
        telemetry.update();
    }

    private void updateVelocity() {
        Pose currentPose = follower.getPose();
        double currentTime = velocityTimer.seconds();

        // Avoid dividing by zero if the loop runs too fast
        if (currentPose == null || currentTime - lastTime < 0.005) return;

        double dt = currentTime - lastTime;

        // Calculate change in position
        double dx = currentPose.getX() - lastPose.getX();
        double dy = currentPose.getY() - lastPose.getY();

        // velocity = distance / time
        double vx = dx / dt;
        double vy = dy / dt;

        // Store it in a Pose object (we don't care about heading velocity for this)
        currentVelocity = new Pose(vx, vy, 0);

        // Update last values for the next loop
        lastPose = currentPose;
        lastTime = currentTime;
    }

    private void addDebugTelemetry() {
        Pose pose = follower.getPose();


        // TARGET INFORMATION
        telemetryM.addData("=== TARGET ===", "");
        telemetryM.addData("Target X", String.format("%.2f", targetX));
        telemetryM.addData("Target Y", String.format("%.2f", targetY));

        // CURRENT POSITION
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

        // TURRET ENCODER RAW DATA
        telemetryM.addData("=== TURRET ENCODER ===", "");
        telemetryM.addData("Encoder Voltage", String.format("%.3f", turretEncoder.getVoltage()));
        telemetryM.addData("Max Voltage", String.format("%.3f", turretEncoder.getMaxVoltage()));
        telemetryM.addData("Turret Zero Deg", turretZeroDeg);

        // TURRET STATE
        telemetryM.addData("=== TURRET STATE ===", "");
        telemetryM.addData("Aiming Active", aimingActive ? "YES (Left Trigger)" : "NO (Press Left Trigger)");
        telemetryM.addData("Turret State", turretState.toString());
        telemetryM.addData("Current Angle (deg)", String.format("%.2f", getTurretAngleDeg()));
        telemetryM.addData("Desired Angle (deg)", String.format("%.2f", getDesiredTurretAngleDeg()));
        telemetryM.addData("Clamped Angle (deg)", String.format("%.2f", overshoot()));

        // TURRET PID
        telemetryM.addData("=== TURRET PID ===", "");
        telemetryM.addData("Kp", turretKp);
        telemetryM.addData("Ki", turretKi);
        telemetryM.addData("Kd", turretKd);
        telemetryM.addData("Ks (FF)", turretKs);
        telemetryM.addData("Deadband (deg)", turretDeadbandDeg);
        telemetryM.addData("Max Power", turretMaxPower);
        telemetryM.addData("Threshold (deg)", threshold);

        // TURRET POWER
        telemetryM.addData("=== TURRET POWER ===", "");
        telemetryM.addData("Turret Motor Power", String.format("%.3f", turret.getPower()));

        // GAMEPAD STATUS
        telemetryM.addData("=== CONTROLS ===", "");
        telemetryM.addData("Left Trigger", String.format("%.2f", dbzGamepad1.left_trigger));
        telemetryM.addData("Right Trigger", String.format("%.2f", dbzGamepad1.right_trigger));

        // PIVOT OFFSET
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
        if (rightTriggerHeld && !shootLast && !shooting) {
            holdServo.setPosition(holdOpenPos);
            leftpushServo.setPosition(leftPushShoot);
            rightpushServo.setPosition(rightPushShoot);

            intaketimer.reset();
            shooting = true;
        }


        if (shooting && intaketimer.milliseconds() > 1000) {
            leftpushServo.setPosition(leftPushIdle);
            rightpushServo.setPosition(rightPushIdle);
            holdServo.setPosition(holdClosePos);


            shooting = false;
        }

        shootLast = rightTriggerHeld;
    }

    private void activeIntake() {
        boolean leftBumper = gamepad1.left_bumper;
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
            leftpushServo.setPosition(leftPushIdle);
            rightpushServo.setPosition(rightPushIdle);
        } else if (intakeReverseOn) {
            intakeMotor.setPower(-1);
            leftpushServo.setPosition(leftPushIdle);
            rightpushServo.setPosition(rightPushIdle);
        } else {
            intakeMotor.setPower(0);

            leftpushServo.setPosition(leftPushIdle);
            rightpushServo.setPosition(rightPushIdle);
        }

        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
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

        if (Math.abs(errorDeg) <= turretDeadbandDeg) {
            turret.setPower(0);
            telemetryM.addData("Turret Status", "AT TARGET");
            telemetryM.addData("Target Angle (deg)", String.format("%.2f", targetAngleDeg));
            telemetryM.addData("Error (deg)", String.format("%.2f", errorDeg));
            telemetryM.addData("PID Output", 0.0);
            telemetryM.addData("FF Output", 0.0);
            telemetryM.addData("Total Output", 0.0);
            return;
        }

        // FIXED PID CALCULATION
        turretPID.setPID(turretKp, turretKi, turretKd);
        double pidOut = turretPID.calculate(currentAngleDeg, targetAngleDeg);

        // Feedforward with Ks = 0
        double ff = 0.0;
        if (Math.abs(errorDeg) > turretFFDeadbandDeg) {
            ff = Math.copySign(turretKs, errorDeg);
        }

        double output = pidOut + ff;

        // Clamp output
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

        // --- ADD TELEMETRY FOR GRAPHING ---
        // Dashboard will graph any numerical value.
        // Sending them like this makes them easy to find.
        telemetry.addData("Flywheel Target V", targetVelocity);
        telemetry.addData("Flywheel Actual V", currentVelocity);
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

        // FIX: Use the calculated velocity
        double vx = currentVelocity.getX();
        double vy = currentVelocity.getY();

        // Calculate future position
        double futureX = pose.getX() + vx * shotLeadTime;
        double futureY = pose.getY() + vy * shotLeadTime;

        double fieldAngle = Math.atan2(targetY - futureY, targetX - futureX);
        double turretAngleDeg = Math.toDegrees(fieldAngle - pose.getHeading());

        return angleWrap(turretAngleDeg);
    }


    private double overshoot() {
        double desired = angleWrap(getDesiredTurretAngleDeg());
        if (desired > threshold) return threshold;
        if (desired < -threshold) return -threshold;
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