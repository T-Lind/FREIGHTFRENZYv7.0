package org.firstinspires.ftc.teamcode;

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

public class NewDetectionPipeline extends OpenCvPipeline{
    int level = 0;
    boolean inside = false;

    @Override
    public Mat processFrame(Mat input) {
        return analyze(input);
    }
    public Mat analyze(Mat input){
        int local_level = 0;

        Mat output1 = new Mat(input.rows(), input.cols(), input.type());
        Mat output2 = new Mat(input.rows(), input.cols(), input.type());

        /*double angle = 180;
        Point rotPoint= new Point(input.cols()/2.0, input.rows()/2.0);
        Mat rotMat = Imgproc.getRotationMatrix2D( rotPoint, angle, 1);
        Imgproc.warpAffine(input, output1, rotMat, input.size());*/

        Mat output3 = new Mat(output1.rows(), output1.cols(), output1.type());
        Mat output4 = new Mat(input.rows(), output1.cols(), output1.type());


        Imgproc.Canny(input, output2, 50, 250, 3, true);


        Imgproc.HoughLinesP(output2, output3, 1, Math.PI/180, 20, 25, 10); // runs the actual detection

        // level 1 detection
        for (int x = 0; x < output3.rows(); x++) {
            double[] l = output3.get(x, 0);
            double slope = Math.abs(l[3]-l[1]/(l[2]-l[0]));
            if(((slope > 300 && slope < 1000) && ((l[0] < output1.cols()/2+10) && (l[1] > output1.rows()/3))) && Math.sqrt((l[2]-l[0])*(l[2]-l[0])+(l[3]-l[1])*(l[3]-l[1])) > 100) {
                Imgproc.line(output2, new Point(l[0], l[1]), new Point(l[2], l[3]), new Scalar(100, 100, 100), 7, Imgproc.LINE_4, 0);
                local_level = 1;
            }
        }

        // level 2 detection
        for (int x = 0; x < output3.rows(); x++) {
            double[] l = output3.get(x, 0);
            double slope = Math.abs(l[3]-l[1]/(l[2]-l[0]));
            if(((slope > 300 && slope < 1000) && ((l[0] > output1.cols()/2+130) && (l[1] > output1.rows()/3))) && Math.sqrt((l[2]-l[0])*(l[2]-l[0])+(l[3]-l[1])*(l[3]-l[1])) > 100) {
                Imgproc.line(output2, new Point(l[0], l[1]), new Point(l[2], l[3]), new Scalar(100, 100, 100), 7, Imgproc.LINE_4, 0);
                local_level = 2;
            }
        }

        // level 1 lines
        Imgproc.line(output2, new Point(output1.cols()/2+10,output1.rows()/3),new Point(output1.cols()/2+10,input.rows()), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        Imgproc.line(output2, new Point(0,output1.rows()/3),new Point(output1.cols()/2+10,output1.rows()/3), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);

        // line 2 lines
        Imgproc.line(output2, new Point(output1.cols()/2+130,output1.rows()/3),new Point(output1.cols()/2+130,input.rows()), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        Imgproc.line(output2, new Point(output1.cols()/2+130,output1.rows()/3),new Point(output1.cols(),output1.rows()/3), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);

        level = local_level;

        return output2;
    }
    public int getLevel(){
        return level;
    }
}
