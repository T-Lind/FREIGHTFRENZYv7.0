package org.firstinspires.ftc.teamcode.CameraPipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NewDetectionPipeline extends OpenCvPipeline{
    int level = 0;
    int last_level = 0;

    Mat output1;
    Mat output2;
    Mat output3;
    Mat saved = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        return analyze(input);
    }
    public Mat analyze(Mat input){
        if(output1 == null) {
            output1 = new Mat(input.rows(), input.cols(), input.type());
            output2 = new Mat(input.rows(), input.cols(), input.type());
            output3 = new Mat(output1.rows(), output1.cols(), output1.type());
        }

        int one_count = 0;
        int two_count = 0;
        int three_count = 0;

        input.copyTo(saved);



        Imgproc.Canny(input,
                output2,
                50,
                250,
                3,
                true);
        Imgproc.HoughLinesP(output2,
                output3,
                1,
                Math.PI/180,
                6
                ,
                5,
                10); // runs the actual detection

        for (int x = 0; x < output3.rows(); x++) {
            double[] l = output3.get(x, 0);
            double slope = Math.abs(l[3]-l[1]/(l[2]-l[0]));
            if((l[1] < output1.rows()/2.0) && l[1] > 20  && l[0] < 14*output1.cols()/16.0){//&& (l[1] < output1.rows()/6  || l[3] < output1.rows()/6) && l[0] < 14*output1.cols()/16.0){
                Imgproc.line(saved,
                        new Point(l[0], l[1]),
                        new Point(l[2], l[3]),
                        new Scalar(100, 100, 100),
                        7,
                        Imgproc.LINE_4,
                        0);
                if(l[0] < (2.5/10)*output1.cols())
                    one_count++;
                else if(l[0] < 2*output1.cols()/3.0)
                    two_count++;
                else
                    three_count++;
            }
        }

        // level 1 lines
        Imgproc.line(saved, new Point(0,10),new Point(75,10), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        Imgproc.line(saved, new Point(75,10),new Point(75,160), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        Imgproc.line(saved, new Point(0,160),new Point(75,160), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        // line 2 lines
        Imgproc.line(saved, new Point(230,10),new Point(345,10), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        Imgproc.line(saved, new Point(345,10),new Point(345,160), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        Imgproc.line(saved, new Point(230,160),new Point(345,160), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);
        Imgproc.line(saved, new Point(230,10),new Point(230,160), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);

        if((one_count > two_count/4) && (one_count > three_count))
            level = 1;
        else if((two_count/2 < three_count) && (three_count > one_count))
            level = 3;
        else
            level = 2;

        return saved;
    }
    public int getLevel(){
        return level;
    }
}
