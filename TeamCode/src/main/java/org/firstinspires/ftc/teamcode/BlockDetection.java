package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

/*
While opmode is running:
	If !hasBlock:
		If !hasTarget:
			// check for certain color of blob later
		Blobs = Get all blobs
		Largest_blob = blobs.sort()[0]
		Target_color = largest_blob.color => get average pixel values i guess
		Target_coord = largest_blob.center()
		hasTarget = true
		If hasTarget and !isMoving:
		robot.moveX(robot_current_pos, robot_current_pos - convert(cameraCenter-target_coord.x)
robot.moveY(robot_current_pos, robot_current_pos - convert(cameraCenter-target_coord.y)
	isMoving = true



		If (centerColor == target_color and a blob center exists in center (center is range of x and y coordinates)
			Pick up block
			hasBlock = true
			Move back to start of submersible
			Move left until there is a target

 */


@Autonomous(name="TestCamera", group="Linear OpMode")
public class BlockDetection extends LinearOpMode {

    private static double convert(double pixels) {
        // we would be able to convert pixels to an actual value in inches
        return 1.0;
    }
    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "camera1"))
                .build();

        List<ColorBlobLocatorProcessor.Blob> blobs;
        ColorBlobLocatorProcessor.Blob largestBlob;
        String targetColor;
        double[] targetCoord = new double[2];

        boolean hasBlock = false;
        boolean hasTarget = false;
        boolean isMoving = false;
        double[] center = {160, 120};

        // logic is a bit flawed but we'll get to it sometime
        while (opModeIsActive()) {
            if (!hasBlock) {
                if (!hasTarget) {
                    blobs = colorLocator.getBlobs();
                    ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
                    largestBlob = blobs.get(0);
                    RotatedRect boxFit = largestBlob.getBoxFit();
                    targetCoord[0] = boxFit.center.x;
                    targetCoord[1] = boxFit.center.y;
                    hasTarget = true;
                }
                if (hasTarget && !isMoving) {
                /*
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
                */
                    double distanceX = convert(center[0] - targetCoord[0]);
                    double distanceY = convert(center[1] - targetCoord[1]);
                    isMoving = true;
                }
                if (isMoving && hasTarget) {
                    PredominantColorProcessor.Result result = colorSensor.getAnalysis();
                    // temporary for testing, will try yellow later
                    if (result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) {
                        // pick up block
                    }
                }
            }

        }
    }
}
