package org.firstinspires.ftc.teamcode.extensions;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DbzHardwareMap {

    public DcMotorEx frontRight, backRight, backLeft, frontLeft;
    public DcMotorEx horiz, slideMotorL, slideMotorR;
    public Servo xServo, yServo, rotateServo, clawServo;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public Limelight3A limelight;
    public DcMotorEx turretMotor;
    public DcMotorEx hoodMotor;

    public enum Motor {
        FRONT_RIGHT("frontRight"),
        BACK_RIGHT("backRight"),
        BACK_LEFT("backLeft"),
        FRONT_LEFT("frontLeft"),
        HORIZ("horiz");

        private final String name;
        Motor(String name) { this.name = name; }
        public String getName() { return name; }
    }

    public enum ServoName {
        X("xServo"),
        Y("yServo"),
        ROTATE("rotateServo"),
        CLAW("clawServo");

        private final String name;
        ServoName(String name) { this.name = name; }
        public String getName() { return name; }
    }

    public DbzHardwareMap(HardwareMap hwMap) {
        frontRight = hwMap.get(DcMotorEx.class, Motor.FRONT_RIGHT.getName());
        backRight = hwMap.get(DcMotorEx.class, Motor.BACK_RIGHT.getName());
        backLeft = hwMap.get(DcMotorEx.class, Motor.BACK_LEFT.getName());
        frontLeft = hwMap.get(DcMotorEx.class, Motor.FRONT_LEFT.getName());
        horiz = hwMap.get(DcMotorEx.class, Motor.HORIZ.getName());

        xServo = hwMap.get(Servo.class, ServoName.X.getName());
        yServo = hwMap.get(Servo.class, ServoName.Y.getName());
        rotateServo = hwMap.get(Servo.class, ServoName.ROTATE.getName());
        clawServo = hwMap.get(Servo.class, ServoName.CLAW.getName());

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
