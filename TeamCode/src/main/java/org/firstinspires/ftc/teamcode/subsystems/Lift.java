package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift implements Subsystem {

    private DcMotorEx motor1;
    public DcMotorEx motor2;
    public Servo horizontalServo1;
    public Servo horizontalServo2;


    private static double kP = 0.022, kI = 0, kD = 0.0003; //0.6, 0, 0.01
    private static double ff = 0.16;

    private static int MAX_VEL;
    private static int MAX_ACCEL;

    public static double integralSum = 0;
    private double lastError = 0;

    public double targetHeight = 0;
    private double currentHeight;

    private final double ticks_to_inches = 1.0;
    private final double MAX_POWER = 1;

    private double PID;

    private MotionProfile profile;

    ElapsedTime PIDTimer = new ElapsedTime();

    ElapsedTime mpTimer = new ElapsedTime();

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        motor1 = hardwareMap.get(DcMotorEx.class, "lift1");
        motor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        horizontalServo1 = hardwareMap.get(Servo.class, "horizontal1");
        horizontalServo2 = hardwareMap.get(Servo.class, "horizontal2");
        //reverse correctly
        motor1.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void init() {
        setTargetHeight(0);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        //MotionState state = profile.get(mpTimer.seconds());

        //double instantTargetPosition = state.getX();

        PID = PIDController(targetHeight, currentHeight);

        //PID = PIDController(instantTargetPosition, currentHeight);


        if (targetHeight > 1900) {
            targetHeight = 1900;
        }
        currentHeight = getCurrentHeight();

        if (targetHeight < 31 && currentHeight < 60) {
            setLiftPower(0);
        } else if (motor1.getPower() > 0.5 && motor1.getCurrent(CurrentUnit.AMPS) > 10) {
            setLiftPower(-1);
        } else if (Math.abs(targetHeight - currentHeight) < 50) {
            setLiftPower(ff);
        } else {
            setLiftPower(PID);
        }
    }

    @Override
    public void update(TelemetryPacket packet) {

    }

    public void setLiftPower(double power) {
        power = Range.clip(power, -0.3,MAX_POWER);
        motor2.setPower(power);
        motor1.setPower(power);
    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * PIDTimer.seconds();
        double derivative = (error - lastError) / PIDTimer.seconds();
        lastError = error;

        PIDTimer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }

//    public void setTargetHeight(double height) {
//        targetHeight = height;
//    }

    public Double getTargetHeight() {
        return targetHeight;
    }


    public double getCurrentHeight() {
        double height = motor2.getCurrentPosition() * ticks_to_inches;
        return height;
    }

    public double getMotorPower() {
        return motor2.getPower();
    }

    public double getPID() {
        return PID;
    }

    public void setFF(double input) {
        ff = input;
    }

    public void setPID (double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }

    public void setTargetHeight(double height) {
        targetHeight = height;

    }

    public void setHorizontalPosition(double distance) {
        horizontalServo1.setPosition(distance);
        horizontalServo2.setPosition(1-distance);
    }

}
