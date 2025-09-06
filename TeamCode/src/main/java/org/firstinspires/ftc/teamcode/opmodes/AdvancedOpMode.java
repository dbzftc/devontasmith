package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@TeleOp(name = "spmOpMode")
public class AdvancedOpMode extends DbzOpMode {
    private final double powMult = 1.0;

    //motors
    protected DcMotorEx frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx horiz;
    protected ColorSensor colorSensor;
    protected DistanceSensor distanceSensor;
    protected Limelight3A limelight;
    protected DcMotorEx turretMotor;
    protected DcMotorEx hoodMotor;

    //angles
    double turretTargetAngle = 0;  // degrees
    double hoodTargetAngle = 0;


    //timer
    ElapsedTime timer = new ElapsedTime();


 @Override
    public void opInit() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //motors
        frontLeft = robot.frontLeft;
        frontRight = robot.frontRight;
        backLeft = robot.backLeft;
        backRight = robot.backRight;
        horiz = robot.horiz;
        turretMotor = robot.turretMotor;
        hoodMotor = robot.hoodMotor;




        //limelight
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.pipelineSwitch(6);
        limelight.start();
    }

    @Override
    public void opLoop() {

     drive();
    }

    private void drive() {
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
    }

    private double calculateTurretAngle(double roboX, double roboY, double targetX, double targetY) {
        double angle = Math.atan2(targetY - roboY, targetX - roboX);
        angle = Math.toDegrees(angle);
        return angle;
    }

    @Override
    public void opLoopHook(){

    }
    @Override
    public void opTeardown(){

    }
}