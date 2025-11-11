package org.firstinspires.ftc.teamcode.opmodes;


import static org.firstinspires.ftc.teamcode.opmodes.flywheelTest.kI;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;


import java.util.List;


@TeleOp(name = "smittyOpMode")
public class smittyOpMode extends DbzOpMode {
    private final double powMult = 1.0;


    protected DcMotorEx frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx intakeMotor, outtake1Motor, outtake2motor;

    protected Limelight3A limelight;
    protected Servo holdServo, outtake1, outtake2;


    private double flywheelPower = 0.90;
    private double flywheelPower2 = 0.65;
    private double flywheelPowerOff=0;

    public static double kP = 0.00014;
    public static double kI = 0.0;
    public static double kD = 0.000012;
    public static double kF = 0.00043;  // feedforward coefficient (ticks/sec -> power)
    public static double targetVelocity = -2100; // ticks/sec

    private PIDController controller;
    private DcMotorEx motor1, motor2;
    private VoltageSensor batteryVoltageSensor;
    public static double holdPos = 0.3;
    public static double holdPos2 = 0.12;
    public static double shootPos = 0.0;
    private Servo shoot1Servo;
    private Servo shoot2Servo;





    private Follower follower;
    private PathChain pathChain;


    private boolean shotFired = false;
    private boolean objectDetected = false;
    private boolean motorRunning = false;
    private boolean flywheelRunning = false;
    private boolean TimeStamp = true;
    private boolean ShooterTimeStamp = true;




    double x;
    double y;
    double heading;
    private final Pose currentPose = new Pose(x, y, heading);
    private PIDController outtake1_pid;
    private PIDController outtake2_pid;








    private double tx, ty, ta;
    private ElapsedTime shootingTime = new ElapsedTime();
    private final Pose shootPose = new Pose(67, 67, Math.toRadians(67));
    private final Pose parkPose = new Pose(67, 67, Math.toRadians(67));
    private PathChain pathToShoot, pathToPark;




    private boolean rbprev=false;
    private boolean lbprev=false;
    private ElapsedTime intaketimer = new ElapsedTime();



    @Override
    public void opInit() {

//        holdServo.setPosition(0.5);
        frontLeft = robot.frontLeft;
        frontRight = robot.frontRight;
        backLeft = robot.backLeft;
        backRight = robot.backRight;
        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2motor = robot.outtake2Motor;
        //flywheelMotor = robot.flywheelMotor;
        //limelight = robot.limelight;
        holdServo = hardwareMap.get(Servo.class, "holdServo");
//        outtake1 = hardwareMap.get(Servo.class, "outtake1");
//        outtake2 = hardwareMap.get(Servo.class, "outtake2");

        shoot1Servo = hardwareMap.get(Servo.class, "shoot1Servo");
        shoot2Servo = hardwareMap.get(Servo.class, "shoot2Servo");







        // limelight.pipelineSwitch(0);
        // limelight.start();
        // follower = Constants.createFollower(hardwareMap);
        // pathChain = new PathChain();
        outtake1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake1Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        motor1 = outtake1Motor;
        motor2 = outtake2motor;


// ✅ Initialize PID controller
        controller = new PIDController(kP, kI, kD);


// ✅ Initialize voltage sensor safely
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        }


    }


    @Override
    public void opLoop() {



        drive();
        //detectionValuesMatcha();
        activeIntake();
        shoot();
        transfer();


//        double x = follower.getPose().getX();
//        double y = follower.getPose().getY();
//        double heading = follower.getPose().getHeading();
//        follower.update();


    }


    private void drive() {
        double turn = dbzGamepad1.right_stick_x;
        double straight = -dbzGamepad1.left_stick_y;
        double strafe = dbzGamepad1.left_stick_x;




        // Mecanum drive calculations
        double frontLeftPower = (straight - strafe - turn);
        double frontRightPower = (straight + strafe + turn);
        double backLeftPower = (straight + strafe - turn);
        double backRightPower = (straight - strafe + turn);


        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );


        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        frontLeft.setPower(frontLeftPower * powMult);
        frontRight.setPower(frontRightPower * powMult);
        backLeft.setPower(backLeftPower * powMult);
        backRight.setPower(backRightPower * powMult);


        if (dbzGamepad1.y) {
            pathToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, parkPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading())
                    .build();
        }
    }

    private void transfer(){
        if(dbzGamepad1.right_trigger>0.1){
            robot.holdServo.setPosition(holdPos);


        }
        else if(dbzGamepad1.left_trigger>0.1){
            robot.holdServo.setPosition(holdPos2);


        }

        shoot1Servo.setPosition(shootPos);
        shoot2Servo.setPosition(shootPos);

        if(dbzGamepad1.dpad_up){
            shootPos+=0.01;
        }
        else if(dbzGamepad1.dpad_down){
            shootPos-=0.01;
        }
    }
 /*  private void detectionValuesMatcha() {


       LLResult result = limelight.getLatestResult();


       if (result != null) {
           if (result.isValid()) {


               List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();


               if (detectorResults != null) {
                   for (LLResultTypes.DetectorResult dr : detectorResults) {


                       tx = dr.getTargetXDegrees();
                       ty = dr.getTargetYDegrees();
                       ta = dr.getTargetArea();
                   }
                   if (detectorResults.isEmpty()) {
                       objectDet7yg ected = false;
                   } else {
                       objectDetected = true;
                   }
               }


           }
       }
   }  */


    private void activeIntake() {
        boolean rbpress= dbzGamepad1.right_bumper && !rbprev;
        boolean lbpress = dbzGamepad1.left_bumper && !lbprev;
        if (rbpress && intaketimer.milliseconds()>100){
            if (intakeMotor.getPower() != 1){
                intakeMotor.setPower(1);
                motorRunning = true;

            } else{
                intakeMotor.setPower(0);
                motorRunning= false;
            }
            intaketimer.reset();
        }
        if (lbpress && intaketimer.milliseconds()>100) {

            if (intakeMotor.getPower() != -1) {
                intakeMotor.setPower(-1);
                motorRunning = true;

            } else {
                intakeMotor.setPower(0);
                motorRunning = false;
            }
            intaketimer.reset();
        }
    }


    private void shoot () {
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

        if (dbzGamepad1.a && !flywheelRunning) {
            shootingTime.reset();
            outtake1Motor.setPower(-power);
            outtake2motor.setPower(power);

            flywheelRunning = true;

            shootingTime.reset();
        }
        if(dbzGamepad1.x && flywheelRunning) {
            outtake1Motor.setPower(flywheelPowerOff);
            outtake2motor.setPower(flywheelPowerOff);
            flywheelRunning = false;
        }
    }






    @Override
    public void opLoopHook() {


    }


    @Override
    public void opTeardown() {


    }
}
