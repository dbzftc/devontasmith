package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

import java.util.List;

@TeleOp(name = "SimpleOpMode")
public class AdvancedOpMode extends DbzOpMode {
    private final double powMult = 1.0;

    protected DcMotorEx frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx intakeMotor, transfer1Motor, transfer2Motor, flywheelMotor;
    protected ColorSensor colorSensor;
    protected DistanceSensor distanceSensor;
    protected Limelight3A limelight;

    private final double flywheelPower = 0.85;
    private final double hoodAngle = 58.61;

    private Follower follower;
    private PathChain pathChain;

    private boolean shotFired = false;
    private boolean objectDetected = false;

    double x;
    double y;
    double heading;
    private final Pose currentPose = new Pose(x, y, heading);


    private double tx, ty, ta;
    private ElapsedTime shootingTime = new ElapsedTime();
    private final Pose shootPose = new Pose(67, 67, Math.toRadians(67));
    private final Pose parkPose = new Pose(67, 67, Math.toRadians(67));
    private PathChain pathToShoot, pathToPark;

 @Override
    public void opInit() {

     frontLeft = robot.frontLeft;
     frontRight = robot.frontRight;
     backLeft = robot.backLeft;
     backRight = robot.backRight;
     intakeMotor = robot.intakeMotor;
     transfer1Motor = robot.transfer1Motor;
     transfer2Motor = robot.transfer2Motor;
     flywheelMotor = robot.flywheelMotor;
     colorSensor = robot.colorSensor;
     distanceSensor = robot.distanceSensor;
     limelight = robot.limelight;

     limelight.pipelineSwitch(0);
     limelight.start();
     follower = Constants.createFollower(hardwareMap);
     pathChain = new PathChain();


    }

    @Override
    public void opLoop() {

     driveDubaiChocolate();
     detectionValuesMatcha();
     activeIntake67();
     shootLabubu();
     double x = follower.getPose().getX();
     double y = follower.getPose().getY();
     double heading = follower.getPose().getHeading();
     follower.update();
    }

    private void driveDubaiChocolate() {
        double turn = dbzGamepad1.left_stick_x;
        double straight = dbzGamepad1.left_stick_y;
        double strafe = dbzGamepad1.right_stick_x;


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

        if(dbzGamepad1.a){
            pathToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, parkPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(),parkPose.getHeading())
                    .build();
        }
    }
    private void detectionValuesMatcha() {

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
                        objectDetected = false;
                    } else {
                        objectDetected = true;
                    }
                }

            }
        }
    }
    private void activeIntake67(){
     if(dbzGamepad1.dpad_up){
         intakeMotor.setPower(1);
     }
     else if(dbzGamepad1.dpad_down){
         intakeMotor.setPower(-1);
     }
     else{
         intakeMotor.setPower(0);
     }
    }
    private void shootLabubu() {
        if (dbzGamepad1.x && !shotFired) {
            pathToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, shootPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(),shootPose.getHeading())
                    .build();
            flywheelMotor.setPower(flywheelPower);
            shootingTime.reset();
            shotFired = true;
            if (shootingTime.seconds() > 1) {
                transfer1Motor.setPower(1);
                transfer2Motor.setPower(1);
                if (shootingTime.seconds() > 2) {
                    transfer1Motor.setPower(0);
                    transfer2Motor.setPower(0);
                    shotFired = false;
                }
            }
        }
    }





    @Override
    public void opLoopHook(){

    }
    @Override
    public void opTeardown(){

    }
}