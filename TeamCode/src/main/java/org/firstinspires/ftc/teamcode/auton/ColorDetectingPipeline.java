package org.firstinspires.ftc.teamcode.auton;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectingPipeline extends OpenCvPipeline {
    private static final int COLOR_RANGE = 30;

    private volatile double[] pixelColors = {0,0,0};
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
        if (delta / cmax < 0.3) return true;
        return false;
    }
}
