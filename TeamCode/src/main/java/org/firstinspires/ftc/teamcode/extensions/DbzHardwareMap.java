package org.firstinspires.ftc.teamcode.extensions;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DbzHardwareMap {

    public DcMotorEx frontRight, backRight, backLeft, frontLeft;
//    public DcMotorEx hoodMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx outtake1Motor;

    public DcMotorEx outtake2Motor;
//    public DcMotorEx transfer1Motor;
//    public DcMotorEx transfer2Motor;
//    public DcMotorEx flywheelMotor;
//    public ColorSensor colorSensor;
//    public DistanceSensor distanceSensor;
//    public Limelight3A limelight;
//    private Follower follower;

    public Servo holdServo;
    public Servo outtake1;
    public Servo outtake2;

    public enum Motor {
        frontright("frontRight"),
        backright("backRight"),
        backleft("backLeft"),
        frontleft("frontLeft"),
//        hood("hoodMotor"),
        intake("intakeMotor"),

        outtake1("outtake1Motor"),
        outtake2("outtake2Motor");
//        flywheel("flywheelMotor");

        private final String name;
        Motor(String name) { this.name = name; }
        public String getName() { return name; }
    }

    public DbzHardwareMap(HardwareMap hwMap) {
        frontRight = hwMap.get(DcMotorEx.class, Motor.frontright.getName());
        backRight = hwMap.get(DcMotorEx.class, Motor.backright.getName());
        backLeft = hwMap.get(DcMotorEx.class, Motor.backleft.getName());
        frontLeft = hwMap.get(DcMotorEx.class, Motor.frontleft.getName());
//
//        hoodMotor = hwMap.get(DcMotorEx.class, Motor.hood.getName());
        intakeMotor = hwMap.get(DcMotorEx.class, Motor.intake.getName());
        outtake1Motor = hwMap.get(DcMotorEx.class, Motor.outtake1.getName());
        outtake2Motor = hwMap.get(DcMotorEx.class, Motor.outtake2.getName());
//        flywheelMotor = hwMap.get(DcMotorEx.class, Motor.flywheel.getName());
        holdServo = hwMap.get(Servo.class, "holdServo");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2Motor.setDirection(DcMotorSimple.Direction.FORWARD);


//        limelight = hwMap.get(Limelight3A.class, "limelight");
    }
}