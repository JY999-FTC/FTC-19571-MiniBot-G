package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp

public class TestDrive_G extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    double driveTrain_Factor = 0.8;

    // Declarations of hardware, Best practice to declare in class because if declare in runOpMode can only be used in there
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx rightFront;

    private static final boolean USE_WEBCAM = true;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override

    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            updateTelemetry();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);

            /*
            driveTrain(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);

            if (gamepad1.x)
                moveMotor(leftFront, 0.5);
            if (gamepad1.y)
                moveMotor(leftBack, 0.5);
            if (gamepad1.a)
                moveMotor(rightBack, 0.5);
            if (gamepad1.b)
                moveMotor(rightFront, 0.5);

            updateTelemetry(); // A method to display information
             */

        }// while loop end

    }// OpMode end

    public void moveMotor(DcMotorEx motor,double movePower) {
        motor.setPower(movePower);
    }
    public void driveTrain(double rightStickX, double rightStickY, double leftStickX) {
        double x = rightStickX * 1.1; // Counteract imperfect strafing
        double y = -rightStickY; // Remember, Y stick value is reversed
        double rx = leftStickX * 0.7; // turning speed

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftTop_Power = (y + x + rx) / denominator;
        double leftBot_Power = (y - x + rx) / denominator;
        double rightTop_Power = (y - x - rx) / denominator;
        double rightBot_Power = (y + x - rx) / denominator;

        leftFront.setPower(leftTop_Power * driveTrain_Factor);
        leftBack.setPower(leftBot_Power * driveTrain_Factor);
        rightBack.setPower(rightBot_Power * driveTrain_Factor);
        rightFront.setPower(rightTop_Power * driveTrain_Factor);
    }// controller drive end

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    public void updateTelemetry() {
        telemetry.addData("RunTime: ", runtime);
        telemetry.addLine();
        telemetry.addData("Left JoyStick: ", gamepad1.left_stick_x);
        telemetry.addLine();
        telemetry.addData("Right JoyStick: ", gamepad1.right_stick_x);
        telemetry.addLine();
        telemetry.addData("leftFront Power: ", leftFront.getPower());
        telemetry.addData("leftFront Velocity: ", leftFront.getVelocity());
        telemetry.addData("leftFront Current Position: ", leftFront.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("leftBack Power: ", leftBack.getPower());
        telemetry.addData("leftBack Velocity: ", leftBack.getVelocity());
        telemetry.addData("leftBack Current Position: ", leftBack.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("rightBack Power: ", rightBack.getPower());
        telemetry.addData("rightBack Velocity: ", rightBack.getVelocity());
        telemetry.addData("rightBack Current Position: ", rightBack.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("rightFront Power: ", rightFront.getPower());
        telemetry.addData("rightFront Velocity: ", rightFront.getVelocity());
        telemetry.addData("rightFront Current Position: ", rightFront.getCurrentPosition());
        telemetry.addLine();
        //telemetry.addData("par Position & Velocity: ", par.getPositionAndVelocity());
        //telemetry.addData("par Direction: ", par.getDirection());
        //telemetry.addData("par Class: ", par.getClass());
        telemetry.addLine();
        //telemetry.addData("perp Position & Velocity: ", perp.getPositionAndVelocity());
        //telemetry.addData("perp Direction: ", perp.getDirection());
        //telemetry.addData("perp Class: ", perp.getClass());
        telemetry.addLine();

        telemetry.update();
    } // update telemetry end

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}// class end