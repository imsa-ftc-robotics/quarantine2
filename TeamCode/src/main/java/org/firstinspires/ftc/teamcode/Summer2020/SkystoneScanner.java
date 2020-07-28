package org.firstinspires.ftc.teamcode.Summer2020;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

public class SkystoneScanner {
    public static enum Result {
        LEFT,
        CENTER,
        RIGHT,
        ERROR
    }
    public static Result scan() { return new SkystoneScanner()._scan(); }

    OpenCvCamera camera;
    Result result = Result.ERROR;
    boolean resulted = false;
    private Result _scan() {
        RobotOpMode op_mode = RobotOpMode.running_opmode;
        int camera_view_id = op_mode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op_mode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, camera_view_id);
        camera.openCameraDevice();
        camera.setPipeline(new ThePipelineOfDoom());
        camera.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);

        op_mode.sleep(1500);
        Calendar calendar = Calendar.getInstance();
        Date time = calendar.getTime();
        while (!resulted && time.compareTo(calendar.getTime()) > -3000) op_mode.sleep(100);
        Result my_result = result;

        camera.stopStreaming();
        return result;
    }

    //Stuff adapted from the code so things work
    public static float rectHeight = .6f/8f;
    public static float rectWidth = 1.5f/8f;

    public static final float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    public static final float offsetY = -0.5F/8F;//1.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    public static final float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    public static final float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    public static final float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    public static final int rows = 640;
    public static final int cols = 480;

    private static enum Stage
    {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }

    private class ThePipelineOfDoom extends OpenCvPipeline {


        //From here on was copied
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();



        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {

            //YEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEdddddddddddddddddEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEeT THis bit is mine
            int valMid = -1;
            int valLeft = -1;
            int valRight = -1;


            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //YEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEdddddddddddddddddEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEeT THis bit is mine
            if (valLeft == 0 && valMid > 200 && valRight > 200) result = Result.LEFT;
            else if (valLeft > 200 && valMid == 0 && valRight > 200) result = Result.CENTER;
            else if (valLeft > 200 && valMid > 200 && valRight == 0) result = Result.RIGHT;
            else result = Result.ERROR;
            resulted = true;

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);


            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }
    }
}
