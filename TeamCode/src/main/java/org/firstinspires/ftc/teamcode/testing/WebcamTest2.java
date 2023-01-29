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

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp(name = "Webcam Test 2", group = "Testing")
public class WebcamTest2 extends LinearOpMode
{
    OpenCvWebcam webcam;
    ElapsedTime runtime = new ElapsedTime();
    private double lastInput = 0;

    @Override
    public void runOpMode()
    {
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
        SamplePipeline pipeline = new SamplePipeline();

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
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
            public void onError(int errorCode)
            {
                telemetry.addData("error", errorCode);
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", "%.2f", webcam.getFps());
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Pixel colors (PGO)", "[" + pipeline.getPixelColors()[0] + ", " + pipeline.getPixelColors()[1] + ", " + pipeline.getPixelColors()[2] + "]");
            telemetry.addData("Saturation Margin", pipeline.satMargin);
            telemetry.addData("Runtime seconds", runtime.seconds());
            telemetry.addData("lastInput", lastInput);
            telemetry.update();

            if (gamepad1.dpad_up && lastInput + 0.1 < runtime.seconds()) {
                pipeline.satMargin += 0.01;
                lastInput = runtime.seconds();
            }
            if (gamepad1.dpad_down && lastInput + 0.1 < runtime.seconds()) {
                pipeline.satMargin -= 0.01;
                lastInput = runtime.seconds();
            }


            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
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
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        private static final int COLOR_RANGE = 25;

        private volatile double[] pixelColors = {0,0,0};
        public double satMargin = 0.3;
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            double middlePixelHue = hueFromRGB(input.get(input.rows() / 2, input.cols() / 2));

            int orangePixels = 0;
            int greenPixels = 0;
            int purplePixels = 0;

            for (int i = 0; i < input.rows(); i+=10) {
                for (int j = 0; j < input.cols(); j+=10) {
                    double[] pixelRGB = input.get(i, j);
                    if (isGray(pixelRGB)) continue;
                    double pixelHue = hueFromRGB(pixelRGB);
                    String detectedColor = colorFromHue(pixelHue);
                    switch (detectedColor) {
                        case "orange":
                            orangePixels += 1;
                            break;
                        case "green":
                            greenPixels += 1;
                            break;
                        case "purple":
                            purplePixels += 1;
                            break;
                    }
                }
            }

            Imgproc.putText(input, "Hue: " + middlePixelHue, new Point(100, 50), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255,0,0), 3);
            Imgproc.putText(input, "Color detected: " + colorFromHue(middlePixelHue), new Point(100, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255,0,0), 3);
            Imgproc.circle(input, new Point(input.cols()/2f, input.rows()/2f), 5, new Scalar(0,255,0), 3);
            Imgproc.putText(input, "orange: " + orangePixels + " green: " + greenPixels + " purple: " + purplePixels, new Point(0, 150), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255,0,0), 3);

            pixelColors[0] = purplePixels;
            pixelColors[1] = greenPixels;
            pixelColors[2] = orangePixels;
            return input;
        }

        /**
         * Get the number of pixels of each color
         *
         * @return number of pixels in each color in PGO
         */
        public double[] getPixelColors() {
            return pixelColors;
        }

        private double hueFromRGB(double[] rgbPixel) {
            double hue = 0;
            double r = rgbPixel[0] / 255;
            double g = rgbPixel[1] / 255;
            double b = rgbPixel[2] / 255;

            double cmax = Math.max(r, Math.max(g, b));
            double cmin = Math.min(r, Math.min(g, b));
            double delta = cmax - cmin;

            if (cmax == cmin) hue = 0;
            if (cmax == r) hue = 60 * (((g - b)/delta) % 6);
            if (cmax == g) hue = 60 * ((b - r)/delta + 2);
            if (cmax == b) hue = 60 * ((r - g)/delta + 4);
            if (hue < 0) hue += 360;

            return hue;
        }

        private String colorFromHue(double hue) {
            if (hue - 300 < COLOR_RANGE && hue - 300 > -COLOR_RANGE) return "purple";
            if (hue - 90 < COLOR_RANGE && hue - 90 > -COLOR_RANGE) return "green";
            if (hue - 180 < COLOR_RANGE && hue - 180 > -COLOR_RANGE) return "orange";
            return "none";
        }

        private boolean isGray(double[] rgbPixel) {
            double r = rgbPixel[0] / 255;
            double g = rgbPixel[1] / 255;
            double b = rgbPixel[2] / 255;

            double cmax = Math.max(r, Math.max(g, b));
            double cmin = Math.min(r, Math.min(g, b));
            double delta = cmax - cmin;

            if (cmax == 0) return true;
            if (delta / cmax < satMargin) return true;
            return false;
        }


        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}