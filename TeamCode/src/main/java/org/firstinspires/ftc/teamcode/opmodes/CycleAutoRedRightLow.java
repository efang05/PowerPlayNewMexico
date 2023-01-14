package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@Autonomous
public class CycleAutoRedRightLow extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-64,Math.toRadians(180));
    Pose2d Preload_POSE = new Pose2d(35,-10.5, Math.toRadians(180.5));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
    private ElapsedTime timer;

    public double liftHigh = 1100;
    public double liftMid = 700;
    public double liftLow = 350;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 630;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 1.0;
    public double hzslidesin = 0.3;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.closeClaw();
        sleep(1500);
        robot.intake.dropArm();
        detector.init(hardwareMap, telemetry);

        robot.turret.MAX_POWER = 0.725;
        robot.drive.voltagemode = "auto";

        TrajectorySequence preload = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(hzslidesin);

                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.turret.setTargetAngle(495);
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.6);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(0);
                    robot.lift.setTargetHeight(160);
                    robot.intake.centerArm();
                })
                //Pick Up Cone
                .lineToLinearHeading(new Pose2d(51,-8, Math.toRadians(179)))
                //.back(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(60);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })



                //Cycle #1
                .forward(17)
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.turret.setTargetAngle(515);
                    robot.intake.setArmPos(0.6);
                    robot.lift.setTargetHeight(liftHigh);
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(0);
                    robot.lift.setTargetHeight(100);
                    robot.intake.setArmPos(0.55);
                })
                //Pick Up Cone
                .lineToLinearHeading(new Pose2d(51,-10, Math.toRadians(179)))
                //.back(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(50);
                    robot.intake.centerArm();
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })



                //Cycle #2
                .forward(17)
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.turret.setTargetAngle(515);
                    robot.intake.setArmPos(0.6);
                    robot.lift.setTargetHeight(liftHigh);
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(0);
                    robot.lift.setTargetHeight(160);
                    robot.intake.setArmPos(0.55);
                })
                .lineToLinearHeading(new Pose2d(53,-12, Math.toRadians(179)))
                //.back(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(30);
                    robot.intake.centerArm();
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })


                //Cycle #3
                .lineToLinearHeading(new Pose2d(33,-16, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.turret.setTargetAngle(515);
                    robot.intake.setArmPos(0.6);
                    robot.lift.setTargetHeight(liftHigh);
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(0);
                    robot.lift.setTargetHeight(100);
                    robot.intake.setArmPos(0.55);
                })
                .lineToLinearHeading(new Pose2d(53,-14, Math.toRadians(179)))
                //.back(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(10);
                    robot.intake.setArmPos(0.5);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)


                //Cycle #4
                .lineToLinearHeading(new Pose2d(33,-18, Math.toRadians(181)))
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.turret.setTargetAngle(515);
                    robot.intake.setArmPos(0.6);
                    robot.lift.setTargetHeight(liftHigh);
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(0);
                    robot.lift.setTargetHeight(100);
                    robot.intake.setArmPos(0.55);
                })
                .lineToLinearHeading(new Pose2d(53,-16, Math.toRadians(179)))
                //.back(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(10);
                    robot.intake.setArmPos(0.48);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })


                //Cycle #5
                .lineToLinearHeading(new Pose2d(35,-22, Math.toRadians(182)))
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.turret.setTargetAngle(515);
                    robot.intake.setArmPos(0.6);
                    robot.lift.setTargetHeight(liftHigh);
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(640);
                    robot.lift.setTargetHeight(100);
                    robot.intake.centerArm();
                    robot.intake.closeClaw();
                })
                .back(30)
                .build();

        robot.drive.setPoseEstimate(START_POSE);

        SleeveDetectionPipeline.Color Parking = detector.getColor();

        waitForStart();

        if (Parking == SleeveDetectionPipeline.Color.RED) {

        } else if (Parking == SleeveDetectionPipeline.Color.MAGENTA) {

        } else {

        }


        robot.drive.followTrajectorySequenceAsync(preload);
        //robot.drive.followTrajectorySequenceAsync(cycleLow);

        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret target", robot.turret.getTargetAngle());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            robot.update();
            //drive.update();
        }
    }
}