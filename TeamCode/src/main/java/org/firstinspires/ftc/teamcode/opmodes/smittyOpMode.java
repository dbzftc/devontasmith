package org.firstinspires.ftc.teamcode.opmodes;


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


import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;


import java.util.List;


@TeleOp(name = "smittyOpMode")
public class smittyOpMode extends DbzOpMode {
    private final double powMult = 1.0;


    protected DcMotorEx frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx intakeMotor, flywheelMotor;
    protected Limelight3A limelight;
    protected Servo holdServo;


    private final double flywheelPower = 0.85;
    private final double flywheelPowerOff = 0.00;
    private final double hoodAngle = 58.61;


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




    private double tx, ty, ta;
    private ElapsedTime shootingTime = new ElapsedTime();
    private final Pose shootPose = new Pose(67, 67, Math.toRadians(67));
    private final Pose parkPose = new Pose(67, 67, Math.toRadians(67));
    private PathChain pathToShoot, pathToPark;


    public static double holdPos = 0.25;
    public static double holdPos2 = 0.05;

    private boolean rbprev=false;
    private boolean lbprev=false;
    private ElapsedTime intaketimer = new ElapsedTime();



    @Override
    public void opInit() {
//        holdServo.setPosition(0.5);
        //frontLeft = robot.frontLeft;
        //frontRight = robot.frontRight;
       // backLeft = robot.backLeft;
       // backRight = robot.backRight;
//        intakeMotor = robot.intakeMotor;
        //flywheelMotor = robot.flywheelMotor;
        //limelight = robot.limelight;
        holdServo = hardwareMap.get(Servo.class, "holdServo");



        // limelight.pipelineSwitch(0);
        // limelight.start();
       // follower = Constants.createFollower(hardwareMap);
       // pathChain = new PathChain();




    }


    @Override
    public void opLoop() {


        //drive();
        //detectionValuesMatcha();
        activeIntake();
        //shoot();
        transfer();



//        double x = follower.getPose().getX();
//        double y = follower.getPose().getY();
//        double heading = follower.getPose().getHeading();
//        follower.update();


    }


//    private void drive() {
//        double turn = -dbzGamepad1.right_stick_x;
//        double straight = -dbzGamepad1.left_stick_y;
//        double strafe = dbzGamepad1.left_stick_x;
//
//
//
//
//        // Mecanum drive calculations
//        double frontLeftPower = (straight - strafe - turn);
//        double frontRightPower = (straight + strafe + turn);
//        double backLeftPower = (straight + strafe - turn);
//        double backRightPower = (straight - strafe + turn);
//
//
//        double maxPower = Math.max(
//                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
//                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
//        );
//
//
//        if (maxPower > 1.0) {
//            frontLeftPower /= maxPower;
//            frontRightPower /= maxPower;
//            backLeftPower /= maxPower;
//            backRightPower /= maxPower;
//        }
//        frontLeft.setPower(frontLeftPower * powMult);
//        frontRight.setPower(frontRightPower * powMult);
//        backLeft.setPower(backLeftPower * powMult);
//        backRight.setPower(backRightPower * powMult);
//
//
//        if (dbzGamepad1.a) {
//            pathToShoot = follower.pathBuilder()
//                    .addPath(new BezierLine(currentPose, parkPose))
//                    .setLinearHeadingInterpolation(currentPose.getHeading(), parkPose.getHeading())
//                    .build();
//        }
//    }

private void transfer(){
    if(dbzGamepad1.dpad_up){
        holdServo.setPosition(holdPos);


    }
    else if(dbzGamepad1.dpad_down){
        holdServo.setPosition(holdPos2);


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


//        private void shoot () {
//            if (dbzGamepad1.a && !flywheelRunning) {
//
//
//                shootingTime.reset();
//                // pathToShoot = follower.pathBuilder()
//                //         .addPath(new BezierLine(currentPose, shootPose))
//                //        .setLinearHeadingInterpolation(currentPose.getHeading(),shootPose.getHeading())
//                //     .build();
//
//
//                flywheelMotor.setPower(flywheelPower);
//                flywheelRunning = true;
//
//
//                // if(shootingTime.seconds()>5){
//                //  holdServo.setPosition(holdPos2);
//                // }
//
//
//                if (dbzGamepad1.x && flywheelRunning) {
//                    flywheelMotor.setPower(flywheelPowerOff);
//                    flywheelRunning = false;
//                }
//                shootingTime.reset();
//            }
//
//
//        }





    @Override
    public void opLoopHook() {


    }


    @Override
    public void opTeardown() {


    }
}
