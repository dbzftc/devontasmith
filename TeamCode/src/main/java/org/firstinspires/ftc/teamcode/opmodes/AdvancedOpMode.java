package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@TeleOp(name = "spmOpMode")
public class AdvancedOpMode extends DbzOpMode {
    private final double powMult = 1.0;

    //motors
    protected DcMotorEx frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx horiz;

    //pid
    double Kp = 0.01; // Proportional gain
    double Ki = 0.0;  // Integral gain
    double Kd = 0.0;  // Derivative gain
    double Kg = 0.0;  // Feedforward gain

    double targetPosition = 1000;
    double integralSum = 0;
    double lastError = 0;

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
        backRight.setPower(backLeftPower * powMult);
        backLeft.setPower(backRightPower * powMult);
    }

    private void pidMath(){
        double currentPosition = robot.horiz.getCurrentPosition();
        double error = targetPosition - currentPosition;

    // Integral
        integralSum += error * timer.seconds();

     // Derivative
        double derivative = (error - lastError) / timer.seconds();

    // PID output
        double power = Kp * error + Ki * integralSum + Kd * derivative;


        double pidOutput = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        double ffOutput = Kg; // feedforward
        double totalOutput = pidOutput + ffOutput;

        totalOutput = Math.max(-1, Math.min(1, totalOutput));

        horiz.setPower(totalOutput);

        lastError = error;
        timer.reset();



    }

    @Override
    public void opLoopHook(){

    }
    @Override
    public void opTeardown(){

    }
}