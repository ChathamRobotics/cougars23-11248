/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutonBot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Color Detecting Auton", group = "Sensor")
public class ColorDetectingAuton extends LinearOpMode {
    OpenCvWebcam webcam;
    ElapsedTime runtime = new ElapsedTime();
    AutonBot robot = new AutonBot();

    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        ColorDetectingPipeline pipeline = new ColorDetectingPipeline();

        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("error", errorCode);
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addLine("Initing robot");
        telemetry.update();

        robot.init(hardwareMap);
        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        runtime.reset();
        robot.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double totalPurplePixels = 0;
        double totalGreenPixels = 0;
        double totalOrangePixels = 0;
        double cycles = 0;

        while (runtime.seconds() < 5) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", "%.2f", webcam.getFps());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Pixel colors (PGO)", "[" + pipeline.getPixelColors()[0] + ", " + pipeline.getPixelColors()[1] + ", " + pipeline.getPixelColors()[2] + "]");
            telemetry.update();

            if (pipeline.getPixelColors()[0] + pipeline.getPixelColors()[1] + pipeline.getPixelColors()[2] != 0) {
                totalPurplePixels += pipeline.getPixelColors()[0];
                totalGreenPixels += pipeline.getPixelColors()[1];
                totalOrangePixels += pipeline.getPixelColors()[2];

                cycles += 1;
                Log.d("Color Detection", "(PGO:) [" + totalPurplePixels + ", " + totalGreenPixels + ", " + totalOrangePixels + "] Cycles: " + cycles);
            }
        }
        webcam.stopStreaming();

        double[] avgPixelColors = {totalPurplePixels / cycles, totalGreenPixels / cycles, totalOrangePixels / cycles};
        // Offset for env pixels
        avgPixelColors[0] -= 0; // Purple
        avgPixelColors[1] -= 0; // Green
        avgPixelColors[2] -= 0; // Orange

        int maxPixels = 0;
        int nextPixels = 0;
        for (int i = 0; i < avgPixelColors.length; i++) {
            if (avgPixelColors[i] > avgPixelColors[maxPixels]) {
                nextPixels = maxPixels;
                maxPixels = i;
            }
        }

        String[] colorNames = {"purple", "green", "orange"};

        telemetry.addData("Max", (int)(avgPixelColors[maxPixels] * 100) / 100 + ": " + colorNames[maxPixels]);
        telemetry.addData("Next", (int)(avgPixelColors[nextPixels] * 100) / 100 + ": " + colorNames[nextPixels]);
        telemetry.addData("Confidence", (int)((avgPixelColors[maxPixels] - avgPixelColors[nextPixels]) / avgPixelColors[maxPixels] * 100 * 100) / 100 + "%");
        telemetry.update();

        robot.command(3, 80, "move", this);

        switch (maxPixels) {
            case 0:
                //purple
                robot.command(3, -40, "strafe", this);
                break;
            case 2:
                //orange
                robot.command(3, 40, "strafe", this);
                break;
        }
    }
}

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */