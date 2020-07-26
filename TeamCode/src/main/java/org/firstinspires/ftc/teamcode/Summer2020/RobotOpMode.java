package org.firstinspires.ftc.teamcode.Summer2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



public abstract class RobotOpMode extends LinearOpMode {

    public DcMotorEx leftBackDrive;
    public DcMotorEx rightBackDrive;
    public DcMotorEx leftFrontDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx Intake_Left;
    public DcMotorEx Intake_Right;
    public DcMotorEx Lift_1;
    public DcMotorEx Lift_2;

    public Servo foundation_left;
    public Servo foundation_right;
    public Servo deposit_outtake;
    public Servo deposit_clamper;
    public Servo transfer;
    public Servo capstone;
    public CRServo park;

    public BNO055IMU imu;

    public Orientation angles;

    public DistanceSensor stone_checker;


    /**
     *     @TODO
     *     test this
     */
    public double FOUNDATION_RIGHT_DOWN = 0.17;
    public double FOUNDATION_LEFT_DOWN = 0.4;
    public double FOUNDATION_RIGHT_UP = 0.73;
    public double FOUNDATION_LEFT_UP = 1;

    public double DEPOSIT_CLAMPED = 0.4;
    public double DEPOSIT_UNCLAMPED = 0.21;

    public final double CAPSTONE_DOWN = 0;
    public final double CAPSTONE_UP = 0;

    public double DEPOSIT_OUT = 0.2;
    public double DEPOSIT_IN = 0.84;

    public double TRANSFER_UP = 0.7;
    public double TRANSFER_DOWN = 0.1;

    public static final int ticks_per_inch = 537 / 12;

    public DigitalChannel lift_bottom;


    protected boolean initialize_hardware = true;

    public static RobotOpMode running_opmode;

    public void runOpMode() {
        //Initialize hardware
        if (initialize_hardware) {
            leftBackDrive = (DcMotorEx)hardwareMap.get("backLeft");
            leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
            leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightBackDrive = (DcMotorEx)hardwareMap.get("backRight");
            rightBackDrive.setDirection((DcMotorEx.Direction.REVERSE));
            rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftFrontDrive = (DcMotorEx)hardwareMap.get("frontLeft");
            leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
            leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFrontDrive = (DcMotorEx)hardwareMap.get("frontRight");
            rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
            rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            Intake_Left = (DcMotorEx)hardwareMap.get("intakeLeft");
            Intake_Left.setDirection(DcMotorEx.Direction.FORWARD);
            Intake_Right = (DcMotorEx)hardwareMap.get("intakeRight");
            Intake_Right.setDirection(DcMotorEx.Direction.REVERSE);

            capstone = (Servo)hardwareMap.get("capstone");

            Lift_1 = (DcMotorEx)hardwareMap.get("lift1");
            Lift_1.setDirection(DcMotorEx.Direction.FORWARD);
            //Lift_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Lift_2 = (DcMotorEx)hardwareMap.get("lift2");
            Lift_2.setDirection(DcMotorEx.Direction.FORWARD);
            //Lift_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            foundation_left = (Servo)hardwareMap.get("foundationLeft");
            foundation_left.setDirection(Servo.Direction.FORWARD);
            foundation_left.setPosition(FOUNDATION_LEFT_UP);
            foundation_right = (Servo)hardwareMap.get("foundationRight");
            foundation_right.setDirection((Servo.Direction.REVERSE));
            foundation_right.setPosition(FOUNDATION_RIGHT_UP);

            transfer = (Servo)hardwareMap.get("transfer");
            transfer.setDirection(Servo.Direction.FORWARD);

            lift_bottom = (DigitalChannel) hardwareMap.get("lift_bottom");
            lift_bottom.setMode(DigitalChannel.Mode.INPUT);

            deposit_clamper = (Servo)hardwareMap.get("depositClamper");
            deposit_outtake = (Servo)hardwareMap.get("depositOuttake");

            stone_checker = (DistanceSensor)hardwareMap.get("stone_checker");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)stone_checker;



            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            stone_checker = (DistanceSensor)hardwareMap.get("stone_checker");

            int target_tollerance = 20;
            double p = 7.2;
            leftBackDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            leftBackDrive.setPositionPIDFCoefficients(p);
            leftBackDrive.setTargetPositionTolerance(target_tollerance);
            leftFrontDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            leftFrontDrive.setPositionPIDFCoefficients(p);
            leftFrontDrive.setTargetPositionTolerance(target_tollerance);
            rightBackDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            rightBackDrive.setPositionPIDFCoefficients(p);
            rightBackDrive.setTargetPositionTolerance(target_tollerance);
            rightFrontDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            rightFrontDrive.setPositionPIDFCoefficients(p);
            rightFrontDrive.setTargetPositionTolerance(target_tollerance);

            Lift_1.setTargetPositionTolerance(15);
            Lift_2.setTargetPositionTolerance(15);

            park = (CRServo)hardwareMap.get("park");

            telemetry.addData("Status","all set");
            telemetry.update();
        }

        try {
            running_opmode = this;
            op_mode();
        }
        finally {
            running_opmode = null;
        }
    }
    /** Code for the OP mode implementing this class */
    public abstract void op_mode();

    public boolean hasStone(){
        return(stone_checker.getDistance(DistanceUnit.INCH)<1.8);
    }

    public double getFirstAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getSecondAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).secondAngle;
    }

    public double getThirdAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).thirdAngle;
    }

    public void reorientIMU(double targetAngle, double left, double right, double threshold, double kp, double ki, double kd) {
        //get the current value in radians
        double currentValue = getFirstAngle();
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle);
        //initialize PID variables
        double error;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        //convert the threshold to radians
        threshold = Math.toRadians(threshold);
        useEncoders();
        while (Math.abs(targetAngle - currentValue) > threshold && opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle- currentValue;
            //integral is the summation of all the past error
            integral += error;
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError;
            //multiply each value by their respective constants and sum to get outuput
            output = (error * kp) + (integral * ki) + (derivative * kd);

            //set motor power based output value
            leftFrontDrive.setPower(output * left);
            leftBackDrive.setPower(output * left);
            rightFrontDrive.setPower(output * right);
            rightBackDrive.setPower(output * right);

            //get the current value from the IMU
            currentValue = getFirstAngle();
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Left Power", leftBackDrive.getPower());
            telemetry.addData("Right Power", rightBackDrive.getPower());
            telemetry.update();
            //make the last error equal to the current error
            lastError = error;
        }
        stopDrivetrain();
    }

    public void stopDrivetrain(){
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void useEncoders(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void moveWithEncoders(double motorPower, int sleepTime){
        useEncoders();

        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(motorPower);

        sleep(sleepTime);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public void strafeAngle(double speed, double degrees, int sleepTime){
        //set runmode to RUN_USING_ENCODERS
        useEncoders();

        //convert angle to radians
        double radians = Math.toRadians(degrees);

        //subtract pi/4 because the rollers are angled pi/4 radians
        double robotAngle = radians - (Math.PI/4);
        double[] motorPower;
        motorPower = new double[4];

        //set motor powers based on the specified angle
        motorPower[0] = Math.cos(robotAngle);
        motorPower[1] = Math.sin(robotAngle);
        motorPower[2] = Math.sin(robotAngle);
        motorPower[3] = Math.cos(robotAngle);


        //because of limitations with the sin and cos functions, the motors are not always going at the speed that is specified
        //in order to do this, we multiply each motor power by the desired speed over the highest motor power
        double maxPower = 0;
        for (double power : motorPower) {
            if (Math.abs(power) > maxPower) {
                maxPower = Math.abs(power);
            }
        }

        double ratio;

        if (maxPower == 0) {
            ratio = 0;
        } else {
            ratio = speed / maxPower;
        }

        double leftFront = Range.clip((ratio * motorPower[0]), -1, 1);
        double rightFront = Range.clip((ratio * motorPower[1]), -1, 1);
        double leftBack = Range.clip((ratio * motorPower[2]) , -1, 1);
        double rightBack = Range.clip((ratio * motorPower[3]), -1, 1);

        //set motor powers
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);

        sleep(sleepTime);

        stopDrivetrain();
    }

    public void strafingPID(double motorPower, double sleepTime, double kp, double ki, double kd){
        double targetAngle = getFirstAngle();
        double targetTime = getRuntime()+(sleepTime/1000);


        leftBackDrive.setPower(-motorPower);
        leftFrontDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(-motorPower);

        double error = 0;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;
        double outputChange;


        while ((getRuntime()<targetTime)&&!isStopRequested()){
            error = targetAngle-getFirstAngle();
            integral += error;
            derivative = error-lastError;
            outputChange = (error*kp)+(integral*ki)+(derivative*kd);


            leftBackDrive.setPower(-motorPower-outputChange);
            leftFrontDrive.setPower(motorPower-outputChange);
            rightBackDrive.setPower(motorPower+outputChange);
            rightFrontDrive.setPower(-motorPower+outputChange);

            lastError = error;

        }

        stopDrivetrain();
    }

    public double imu_7573coords(double current, double target) {
        double forwards = Math.abs(current + (Math.PI * 2) - target);
        double backwards = Math.abs(current - (Math.PI * 2) - target);
        double normal = Math.abs(current - target);
        double min = Math.min(forwards, Math.min(backwards, normal));
        if (min == forwards) { return current + (Math.PI * 2); }
        else if (min == backwards) { return current  - (Math.PI * 2); }
        else { return current; }
    }


    public void wrapIMU(double targetAngle, double left, double right, double threshold, double kp, double ki, double kd) {
        //get the current value in radians
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle);
        double currentValue = imu_7573coords(getFirstAngle(), targetAngle);
        //initialize PID variables
        double error = targetAngle - currentValue;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        //convert the threshold to radians
        threshold = Math.toRadians(threshold);
        useEncoders();
        while (Math.abs(error) > threshold && opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle- currentValue;
            //integral is the summation of all the past error
            integral += error;
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError;
            //multiply each value by their respective constants and sum to get outuput
            output = (error * kp) + (integral * ki) + (derivative * kd);

            //set motor power based output value
            leftFrontDrive.setPower(output * left);
            leftBackDrive.setPower(output * left);
            rightFrontDrive.setPower(output * right);
            rightBackDrive.setPower(output * right);

            //get the current value from the IMU
            currentValue = imu_7573coords(getFirstAngle(), targetAngle);
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Left Power", leftBackDrive.getPower());
            telemetry.addData("Right Power", rightBackDrive.getPower());
            telemetry.update();
            //make the last error equal to the current error
            lastError = error;
        }
        stopDrivetrain();
    }


    public void drivingPID(double power, double sleepTime, double kp, double ki, double kd){
        useEncoders();
        double motorVel = power*2800;
        double targetAngle = Math.toDegrees(getFirstAngle());
        double error = 0;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;
        leftFrontDrive.setVelocity(motorVel);
        leftBackDrive.setVelocity(motorVel);
        rightBackDrive.setVelocity(motorVel);
        rightFrontDrive.setVelocity(motorVel);
        double outputChange;
        double targetTime = getRuntime()+(sleepTime/1000);
        while((getRuntime()<targetTime)&&!isStopRequested()){
            error = targetAngle - getFirstAngle();
            integral+=error;
            derivative = error-lastError;
            outputChange = (error*kp)+(integral*ki)+(derivative*kd);

            leftBackDrive.setVelocity(motorVel-outputChange);
            leftFrontDrive.setVelocity(motorVel-outputChange);
            rightBackDrive.setVelocity(motorVel+outputChange);
            rightFrontDrive.setVelocity(motorVel+outputChange);

            lastError = error;

        }
        stopDrivetrain();
    }

    public void resetDriveEncoders(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveToPosition(double motorPower, int ticks){
        resetDriveEncoders();

        double motorVelocity = motorPower*2700;

        leftBackDrive.setTargetPosition(ticks);
        leftFrontDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);

        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        rightFrontDrive.setVelocity(motorVelocity);
        rightBackDrive.setVelocity(motorVelocity);
        leftBackDrive.setVelocity(motorVelocity);
        leftFrontDrive.setVelocity(motorVelocity);

        Future wait = Future.WaitMilliseconds(5000);
        while(leftBackDrive.isBusy() && !isStopRequested() && !wait.poll()){
            telemetry.addData("Status", "not there yet");
            telemetry.addData("left back", leftBackDrive.getCurrentPosition());
            telemetry.addData("left front", leftFrontDrive.getCurrentPosition());
            telemetry.addData("right back", rightBackDrive.getCurrentPosition());
            telemetry.addData("right front", rightFrontDrive.getCurrentPosition());
            telemetry.addData("busy1", leftBackDrive.isBusy());
            telemetry.addData("busy2", leftFrontDrive.isBusy());
            telemetry.addData("busy3", rightFrontDrive.isBusy());
            telemetry.addData("busy4", rightBackDrive.isBusy());
            telemetry.addData("Target", leftBackDrive.getTargetPosition());
            telemetry.update();
        }

        stopDrivetrain();

    }

    public void strafe(double power, int sleepTime){
        resetDriveEncoders();

        useEncoders();

        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);

        sleep(sleepTime);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public static final int LIFT_MIN = -100;
    public static final int LIFT_MAX = 3250;

    private boolean lift_killed = false;

    protected void set_lift_checked(int ticks) {
        int pos = Math.max(LIFT_MIN, Math.min(ticks, LIFT_MAX));
        Lift_1.setTargetPosition(pos);
        Lift_2.setTargetPosition(pos);
        if (lift_killed) {
            Lift_1.setMode(DcMotor.RunMode.RUN_TO_POSITION );
            Lift_2.setMode(DcMotor.RunMode.RUN_TO_POSITION );
            Lift_1.setPower(1);
            Lift_2.setPower(1);
        }
    }



    protected void kill_lift() {
        Lift_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift_1.setPower(0);
        Lift_2.setPower(0);
        Lift_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_killed = true;
    }

}
