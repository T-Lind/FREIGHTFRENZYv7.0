package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CubeDetectionPipeline.CB_CHAN_MASK_THRESHOLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Range;

import java.util.ArrayList;

public class DuckDetectionPipeline extends OpenCvPipeline{
    double ducc_x = -1;
    double ducc_y = -1;

    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));


    static final int CB_CHAN_IDX = 2;

    @Override
    public Mat processFrame(Mat input) {
        return analyze(input);
    }
    public Mat analyze(Mat input){

        Core.extractChannel(input, input, CB_CHAN_IDX);
        Imgproc.threshold(input, input, 10, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.dilate(input, input, dilateElement);


        Imgproc.line(input, new Point(0,input.rows()/2),new Point(input.cols(),input.rows()/2), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);

        return input;
    }



    public double getDucc_x(){
        return ducc_x;
    }
    public double getDucc_y(){
        return ducc_y;
    }
}
