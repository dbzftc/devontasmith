package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "V2BLUE")
public class V2BLUE extends DbzOpMode {

    public static double rpmA = 0.0125, rpmB = 4.153, rpmC = 1045.2;
    public static double hoodA = -0.00003, hoodB = 0.0097, hoodC = 0.118;
    public static double timeA = 0.0, timeB = 0.0, timeC = 0.0;
    public static double goalx = 0, goaly = 144;

    public static double vkP = 4.8;
    public static double vkF = 1.32;
    private VoltageSensor batteryVoltageSensor;

    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean shootingSequence = false;
    public static double shot1Time = 250, shot2Time = 500, shotReturnTime = 750;
    public static double Push0 = 0.06, Push1 = 0.4, Push2 = 0.6, Push3 = 0.66;

    private boolean fastPushMode = false;
    private boolean lastStickButton = false;

    private Follower follower;
    private Limelight3A limelight;
    private PIDController turretPID;

    private DcMotorEx turretMotor, flywheel, intakeMotor;
    private Servo hoodServo, leftpushServo, rightpushServo, light, light2;
    private DistanceSensor sensor1, sensor2;
    private AnalogInput turretEncoder;

    private double targetRPM = 0;
    private boolean threeBallsLocked = false, previouslyLocked = false;
    public static double dthresh = 4, turretZeroDeg = 329, servooffset = 0.023;
    public static double kalman = 0.15, trust = 0.05;
    private boolean lastLB = false, lastRB = false;

    @Override
    protected void opInit() {
        follower = org.firstinspires.ftc.teamcode.auton.Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        turretPID = new PIDController(0.02, 0, 0.002);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        hoodServo = hardwareMap.get(Servo.class, "hood");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");

        light = hardwareMap.get(Servo.class, "light");
        light2 = hardwareMap.get(Servo.class, "light2");

        sensor1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        sensor2 = hardwareMap.get(DistanceSensor.class, "sensor2");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void opLoop() {
        updateLocalization();
        checkThreeBalls();

        if (gamepad1.left_stick_button && !lastStickButton) {
            fastPushMode = !fastPushMode;
        }
        lastStickButton = gamepad1.left_stick_button;

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        if (gamepad1.a) {
            Pose vGoal = updateGoal();
            turret(vGoal);
            runLauncher(vGoal);
        } else {
            double currentTurretDegrees = getTurretDegrees();
            double idlePower = turretPID.calculate(currentTurretDegrees, 0);
            turretMotor.setPower(Range.clip(idlePower, -0.3, 0.3));

            targetRPM = 0;
            hoodServo.setPosition(0.33);
        }

        runFlywheelVelocityControl(targetRPM);

        if (gamepad1.right_trigger > 0.1 && !shootingSequence && targetRPM > 0) {
            shootingSequence = true;
            shootTimer.reset();
        }

        if (shootingSequence) {
            handleShootingSequence();
        }

        handleIntake();

        if (gamepad1.y) {
            follower.setPose(new Pose(8, 8, Math.toRadians(180)));
        }

        updateStatusLights();
    }

    private void runFlywheelVelocityControl(double target) {
        if (target <= 0) {
            flywheel.setPower(0);
            return;
        }

        double currentVel = flywheel.getVelocity();
        double maxVel = 2800;

        double error = (target - currentVel) / maxVel;
        double feedforward = vkF * (target / maxVel) * (12.0 / batteryVoltageSensor.getVoltage());
        double power = (vkP * error) + feedforward;

        flywheel.setPower(Range.clip(power, -1, 1));
    }

    private void handleShootingSequence() {
        if (!fastPushMode) {
            if (shootTimer.milliseconds() < shot1Time) {
                leftpushServo.setPosition(Push1);
            } else if (shootTimer.milliseconds() < shot2Time) {
                leftpushServo.setPosition(Push2);
            } else if (shootTimer.milliseconds() < shotReturnTime) {
                leftpushServo.setPosition(Push3);
            } else {
                leftpushServo.setPosition(Push0);
                shootingSequence = false;
            }
        } else {
            if (shootTimer.milliseconds() < 700) {
                leftpushServo.setPosition(Push3);
            } else {
                leftpushServo.setPosition(Push0);
                shootingSequence = false;
            }
        }
        rightpushServo.setPosition(leftpushServo.getPosition() - servooffset);
    }

    private Pose updateGoal() {
        Pose robotPose = follower.getPose();
        Vector robotVelocity = follower.getVelocity();

        double actualDist = Math.hypot(goalx - robotPose.getX(), goaly - robotPose.getY());
        double shotTime = (timeA * Math.pow(actualDist, 2)) + (timeB * actualDist) + timeC;

        double vGoalX = goalx - (robotVelocity.getXComponent() * shotTime);
        double vGoalY = goaly - (robotVelocity.getYComponent() * shotTime);

        return new Pose(vGoalX, vGoalY, 0);
    }

    private void turret(Pose vGoal) {
        double currentTurretDegrees = getTurretDegrees();
        Pose robotPose = follower.getPose();

        double targetDegrees = Math.toDegrees(Math.atan2(vGoal.getY() - robotPose.getY(), vGoal.getX() - robotPose.getX()) - robotPose.getHeading());
        double error = angleWrap(targetDegrees - currentTurretDegrees);

        turretMotor.setPower(Range.clip(turretPID.calculate(0, error), -1.0, 1.0));
    }

    private void runLauncher(Pose vGoal) {
        Pose robotPose = follower.getPose();
        double vDist = Math.hypot(vGoal.getX() - robotPose.getX(), vGoal.getY() - robotPose.getY());

        targetRPM = (rpmA * Math.pow(vDist, 2)) + (rpmB * vDist) + rpmC;
        hoodServo.setPosition(Range.clip((hoodA * Math.pow(vDist, 2)) + (hoodB * vDist) + hoodC, 0, 1));
    }

    private void handleIntake() {
        if (gamepad1.right_bumper && !lastRB) {
            intakeMotor.setPower(intakeMotor.getPower() > 0.1 ? 0 : 1.0);
        }
        if (gamepad1.left_bumper && !lastLB) {
            intakeMotor.setPower(intakeMotor.getPower() < -0.1 ? 0 : -1.0);
        }

        if (threeBallsLocked && intakeMotor.getPower() > 0) {
            intakeMotor.setPower(0);
        }

        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
    }

    private void checkThreeBalls() {
        threeBallsLocked = (sensor1.getDistance(DistanceUnit.CM) < dthresh || sensor2.getDistance(DistanceUnit.CM) < dthresh);

        if (threeBallsLocked && !previouslyLocked) {
            gamepad1.rumble(0.5, 0.5, 250);
        }
        previouslyLocked = threeBallsLocked;
    }

    public void updateLocalization() {
        follower.update();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D llpose = result.getBotpose();

            double turretRad = Math.toRadians(getTurretDegrees());
            double chassisHeading = AngleUnit.normalizeRadians(llpose.getOrientation().getYaw(AngleUnit.RADIANS) - turretRad);

            double absX = llpose.getPosition().y * 39.37 + 72;
            double absY = 72 - llpose.getPosition().x * 39.37;

            Pose current = follower.getPose();
            Vector vel = follower.getVelocity();
            double speed = Math.hypot(vel.getXComponent(), vel.getYComponent());

            double kGain = kalman / (1.0 + (speed * trust));

            follower.setPose(new Pose(
                    current.getX() + kGain * (absX - current.getX()),
                    current.getY() + kGain * (absY - current.getY()),
                    chassisHeading
            ));
        }
    }

    private void updateStatusLights() {
        double currentTurretDegrees = getTurretDegrees();
        Pose p = follower.getPose();

        double error = angleWrap(Math.toDegrees(Math.atan2(goaly - p.getY(), goalx - p.getX()) - p.getHeading()) - currentTurretDegrees);

        if (Math.abs(error) < 2.0 && targetRPM > 0 && flywheel.getVelocity() > (targetRPM * 0.95)) {
            light.setPosition(0.5);
            light2.setPosition(0.5);
        } else if (threeBallsLocked) {
            light.setPosition(0.722);
            light2.setPosition(0.722);
        } else {
            light.setPosition(0.0);
            light2.setPosition(0.0);
        }
    }

    private double getTurretDegrees() {
        double degrees = (turretEncoder.getVoltage() / turretEncoder.getMaxVoltage()) * 360.0 - turretZeroDeg;
        return angleWrap(degrees);
    }

    private double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    @Override protected void opLoopHook() {}
    @Override protected void opTeardown() {}
}