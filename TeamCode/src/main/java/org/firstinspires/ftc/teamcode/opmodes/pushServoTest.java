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
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.List;
@Config
@TeleOp(name = "pushServoTest")
public class pushServoTest extends DbzOpMode {
    private ElapsedTime intaketimer = new ElapsedTime();

    public static double power = -0.2;
//    public static double servooffset = 0.023;

    public static double holdservp = 0.1;
    protected DcMotorEx intakeMotor, outtake1Motor, outtake2Motor;
    private DcMotorEx motor1, motor2;
    public static double servooffset = 0.023;
    public static double Push0 = 0.06;
    public static double Push1 = 0.4;
    public static double Push2 = 0.6;
    public static double Push3 = 0.66;

    private Servo rightpushServo, leftpushServo, holdServo;

    public static String servoLeftName = "leftpushServo";
    public static String servoRightName = "rightpushServo";

    private boolean shootLast = false;
    private boolean shooting = false;
    public static double holdOpenPos = 0.2;
    public static double holdClosePos = 0.1;

    @Override
    public void opInit() {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtake1Motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2Motor");
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;

        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2Motor.setDirection(DcMotorEx.Direction.FORWARD);

        leftpushServo = hardwareMap.get(Servo.class, servoLeftName);
        rightpushServo = hardwareMap.get(Servo.class, servoRightName);
        holdServo = hardwareMap.get(Servo.class, "holdServo");
    }


    @Override
    public void opLoop() {
        if (!shooting) {
            leftpushServo.setPosition(Push0);
            rightpushServo.setPosition(Push0 - servooffset);
        }
        holdServo.setPosition(holdservp);
        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);
        shoot();
      }

    private void shoot() {
        boolean rightTriggerHeld = dbzGamepad1.right_trigger > 0.1;
        if (rightTriggerHeld && !shootLast && !shooting) {
            holdServo.setPosition(holdOpenPos);
            leftpushServo.setPosition(Push1);
            rightpushServo.setPosition(Push1-servooffset);

            intaketimer.reset();
            shooting = true;
        }

        if (shooting && intaketimer.milliseconds() > 250) {
            leftpushServo.setPosition(Push2);
            rightpushServo.setPosition(Push2-servooffset);
        }

        if (shooting && intaketimer.milliseconds() > 500) {
            leftpushServo.setPosition(Push3);
            rightpushServo.setPosition(Push3-servooffset);
        }

        if (shooting && intaketimer.milliseconds() > 1000) {
            leftpushServo.setPosition(Push0);
            rightpushServo.setPosition(Push0-servooffset);
            holdServo.setPosition(holdClosePos);

            shooting = false;
        }


        shootLast = rightTriggerHeld;
    }

    @Override
    public void opLoopHook() {}

    @Override
    public void opTeardown() {}
}