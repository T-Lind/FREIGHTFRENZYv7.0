
package org.firstinspires.ftc.teamcode.CameraPipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;


public class imagePipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input){
        return input;
    }
}