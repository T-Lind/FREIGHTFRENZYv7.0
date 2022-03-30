package org.firstinspires.ftc.teamcode.CameraPipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TSEDetectionPipeline extends OpenCvPipeline{
    int level = 0;
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
    int line = 245;

    static final int CB_CHAN_IDX = 0;

    @Override
    public Mat processFrame(Mat input) {
        return analyze(input);
    }

    public Mat analyze(Mat input){

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2GRAY);

        Imgproc.dilate(input, input, dilateElement);

        Mat copy = new Mat();
        input.copyTo(copy);

       // Core.extractChannel(input, copy, CB_CHAN_IDX);
        Imgproc.threshold(copy, input, 199, 200, Imgproc.THRESH_BINARY);
    try {
        for(int x=0;x<90;x++)
            if ((int) input.get(line, 250+x)[0]  > 0) {
                level = 2;
                x = 100;
            }
        for(int x=0;x<100;x++)
            if ((int) input.get(line, 1+x)[0] > 0) {
                level = 1;
                x = 100;
            }
        for(int x=0;x<100;x++)
            if ((int) input.get(line, 539+x)[0] > 0) {
                level = 3;
                x=100;
            }
    }
    catch(Exception e){
        level = -1;
    }
        Imgproc.line(input, new Point(0, line), new Point(640, line), new Scalar(100,50,150), 4);
        Imgproc.line(input, new Point(0, line+20), new Point(640, line+20), new Scalar(100,50,150), 4);
        return input;
    }

    public int getLevel(){
        return level;
    }
}
