package org.firstinspires.ftc.teamcode.Summer2020

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.*

abstract class RobotOpModeRe : LinearOpMode() {
    lateinit var leftBackDrive: DcMotorEx
    lateinit var rightBackDrive: DcMotorEx
    lateinit var leftFrontDrive: DcMotorEx
    lateinit var rightFrontDrive: DcMotorEx
    lateinit var Intake_Left: DcMotorEx
    lateinit var Intake_Right: DcMotorEx
    lateinit var Lift_1: DcMotorEx
    lateinit var Lift_2: DcMotorEx
    lateinit var foundation_left: Servo
    lateinit var foundation_right: Servo
    lateinit var deposit_outtake: Servo
    lateinit var deposit_clamper: Servo
    lateinit var transfer: Servo
    lateinit var capstone: Servo
    lateinit var park: CRServo
    lateinit var imu: BNO055IMU
    lateinit var angles: Orientation
    lateinit var stone_checker: DistanceSensor

    /**
     * @TODO
     * test this
     */
    val FOUNDATION_RIGHT_DOWN = 0.17
    val FOUNDATION_LEFT_DOWN = 0.4
    val FOUNDATION_RIGHT_UP = 0.73
    val FOUNDATION_LEFT_UP = 1.0
    val DEPOSIT_CLAMPED = 0.4
    val DEPOSIT_UNCLAMPED = 0.21
    val CAPSTONE_DOWN = 0.0
    val CAPSTONE_UP = 0.0
    val DEPOSIT_OUT = 0.2
    val DEPOSIT_IN = 0.84
    val TRANSFER_UP = 0.7
    val TRANSFER_DOWN = 0.1
    var lift_bottom: DigitalChannel? = null
    protected var initialize_hardware = true
    override fun runOpMode() {
        //Initialize hardware
        if (initialize_hardware) {
            leftBackDrive = hardwareMap["backLeft"] as DcMotorEx
            leftBackDrive.direction = DcMotorSimple.Direction.FORWARD
            leftBackDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            rightBackDrive = hardwareMap["backRight"] as DcMotorEx
            rightBackDrive.direction = DcMotorSimple.Direction.REVERSE
            rightBackDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            leftFrontDrive = hardwareMap["frontLeft"] as DcMotorEx
            leftFrontDrive.direction = DcMotorSimple.Direction.FORWARD
            leftFrontDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            rightFrontDrive = hardwareMap["frontRight"] as DcMotorEx
            rightFrontDrive.direction = DcMotorSimple.Direction.REVERSE
            rightFrontDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            Intake_Left = hardwareMap["intakeLeft"] as DcMotorEx
            Intake_Left.direction = DcMotorSimple.Direction.FORWARD
            Intake_Right = hardwareMap["intakeRight"] as DcMotorEx
            Intake_Right.direction = DcMotorSimple.Direction.REVERSE
            capstone = hardwareMap["capstone"] as Servo
            Lift_1 = hardwareMap["lift1"] as DcMotorEx
            Lift_1.direction = DcMotorSimple.Direction.FORWARD
            //Lift_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Lift_2 = hardwareMap["lift2"] as DcMotorEx
            Lift_2.direction = DcMotorSimple.Direction.FORWARD
            //Lift_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            foundation_left = hardwareMap["foundationLeft"] as Servo
            foundation_left.direction = Servo.Direction.FORWARD
            foundation_left.position = FOUNDATION_LEFT_UP
            foundation_right = hardwareMap["foundationRight"] as Servo
            foundation_right.direction = Servo.Direction.REVERSE
            foundation_right.position = FOUNDATION_RIGHT_UP
            transfer = hardwareMap["transfer"] as Servo
            transfer.direction = Servo.Direction.FORWARD
            lift_bottom = hardwareMap["lift_bottom"] as DigitalChannel
            lift_bottom!!.mode = DigitalChannel.Mode.INPUT
            deposit_clamper = hardwareMap["depositClamper"] as Servo
            deposit_outtake = hardwareMap["depositOuttake"] as Servo
            stone_checker = hardwareMap["stone_checker"] as DistanceSensor
            val sensorTimeOfFlight = stone_checker as Rev2mDistanceSensor?
            val parameters = BNO055IMU.Parameters()
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            parameters.calibrationDataFile = "BNO055IMUCalibration.json" // see the calibration sample opmode
            parameters.loggingEnabled = true
            parameters.loggingTag = "IMU"
            parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU::class.java, "imu")
            imu.initialize(parameters)
            stone_checker = hardwareMap["stone_checker"] as DistanceSensor
            val target_tollerance = 20
            val p = 7.2
            leftBackDrive.setVelocityPIDFCoefficients(12.20372439 * 0.1, 12.20372439 * 0.01, 0.0, 12.20372439)
            leftBackDrive.setPositionPIDFCoefficients(p)
            leftBackDrive.targetPositionTolerance = target_tollerance
            leftFrontDrive.setVelocityPIDFCoefficients(12.20372439 * 0.1, 12.20372439 * 0.01, 0.0, 12.20372439)
            leftFrontDrive.setPositionPIDFCoefficients(p)
            leftFrontDrive.targetPositionTolerance = target_tollerance
            rightBackDrive.setVelocityPIDFCoefficients(12.20372439 * 0.1, 12.20372439 * 0.01, 0.0, 12.20372439)
            rightBackDrive.setPositionPIDFCoefficients(p)
            rightBackDrive.targetPositionTolerance = target_tollerance
            rightFrontDrive.setVelocityPIDFCoefficients(12.20372439 * 0.1, 12.20372439 * 0.01, 0.0, 12.20372439)
            rightFrontDrive.setPositionPIDFCoefficients(p)
            rightFrontDrive.targetPositionTolerance = target_tollerance
            Lift_1.targetPositionTolerance = 15
            Lift_2.targetPositionTolerance = 15
            park = hardwareMap["park"] as CRServo
            telemetry.addData("Status", "all set")
            telemetry.update()
        }
        try {
            running_opmode = this
            op_mode()
        } finally {
            running_opmode = null
        }
    }

    /** Code for the OP mode implementing this class  */
    abstract fun op_mode()
    fun hasStone(): Boolean {
        return stone_checker.getDistance(DistanceUnit.INCH) < 1.8
    }

    val firstAngle: Double
        get() = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()

    val secondAngle: Double
        get() = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).secondAngle.toDouble()

    val thirdAngle: Double
        get() = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).thirdAngle.toDouble()

    fun reorientIMU(targetAngle: Double, left: Double, right: Double, threshold: Double, kp: Double, ki: Double, kd: Double) {
        //get the current value in radians
        var targetAngle = targetAngle
        var threshold = threshold
        var currentValue = firstAngle
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle)
        //initialize PID variables
        var error: Double
        var derivative: Double
        var integral = 0.0
        var lastError = 0.0
        var output: Double
        //convert the threshold to radians
        threshold = Math.toRadians(threshold)
        useEncoders()
        while (Math.abs(targetAngle - currentValue) > threshold && opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle - currentValue
            //integral is the summation of all the past error
            integral += error
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError
            //multiply each value by their respective constants and sum to get outuput
            output = error * kp + integral * ki + derivative * kd

            //set motor power based output value
            leftFrontDrive.power = output * left
            leftBackDrive.power = output * left
            rightFrontDrive.power = output * right
            rightBackDrive.power = output * right

            //get the current value from the IMU
            currentValue = firstAngle
            telemetry.addData("Current Value", currentValue)
            telemetry.addData("Target", targetAngle)
            telemetry.addData("Left Power", leftBackDrive.power)
            telemetry.addData("Right Power", rightBackDrive.power)
            telemetry.update()
            //make the last error equal to the current error
            lastError = error
        }
        stopDrivetrain()
    }

    fun stopDrivetrain() {
        leftBackDrive.power = 0.0
        leftFrontDrive.power = 0.0
        rightFrontDrive.power = 0.0
        rightBackDrive.power = 0.0
    }

    fun useEncoders() {
        leftFrontDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftBackDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightFrontDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightBackDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftFrontDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftBackDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightBackDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightFrontDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun moveWithEncoders(motorPower: Double, sleepTime: Int) {
        useEncoders()
        leftFrontDrive.power = motorPower
        leftBackDrive.power = motorPower
        rightBackDrive.power = motorPower
        rightFrontDrive.power = motorPower
        sleep(sleepTime.toLong())
        leftFrontDrive.power = 0.0
        leftBackDrive.power = 0.0
        rightBackDrive.power = 0.0
        rightFrontDrive.power = 0.0
    }

    fun strafeAngle(speed: Double, degrees: Double, sleepTime: Int) {
        //set runmode to RUN_USING_ENCODERS
        useEncoders()

        //convert angle to radians
        val radians = Math.toRadians(degrees)

        //subtract pi/4 because the rollers are angled pi/4 radians
        val robotAngle = radians - Math.PI / 4
        val motorPower: DoubleArray
        motorPower = DoubleArray(4)

        //set motor powers based on the specified angle
        motorPower[0] = Math.cos(robotAngle)
        motorPower[1] = Math.sin(robotAngle)
        motorPower[2] = Math.sin(robotAngle)
        motorPower[3] = Math.cos(robotAngle)


        //because of limitations with the sin and cos functions, the motors are not always going at the speed that is specified
        //in order to do this, we multiply each motor power by the desired speed over the highest motor power
        var maxPower = 0.0
        for (power in motorPower) {
            if (Math.abs(power) > maxPower) {
                maxPower = Math.abs(power)
            }
        }
        val ratio: Double
        ratio = if (maxPower == 0.0) {
            0.0
        } else {
            speed / maxPower
        }
        val leftFront = Range.clip(ratio * motorPower[0], -1.0, 1.0)
        val rightFront = Range.clip(ratio * motorPower[1], -1.0, 1.0)
        val leftBack = Range.clip(ratio * motorPower[2], -1.0, 1.0)
        val rightBack = Range.clip(ratio * motorPower[3], -1.0, 1.0)

        //set motor powers
        leftFrontDrive.power = leftFront
        rightFrontDrive.power = rightFront
        leftBackDrive.power = leftBack
        rightBackDrive.power = rightBack
        sleep(sleepTime.toLong())
        stopDrivetrain()
    }

    fun strafingPID(motorPower: Double, sleepTime: Double, kp: Double, ki: Double, kd: Double) {
        val targetAngle = firstAngle
        val targetTime = runtime + sleepTime / 1000
        leftBackDrive.power = -motorPower
        leftFrontDrive.power = motorPower
        rightBackDrive.power = motorPower
        rightFrontDrive.power = -motorPower
        var error = 0.0
        var integral = 0.0
        var derivative = 0.0
        var lastError = 0.0
        var outputChange: Double
        while (runtime < targetTime && !isStopRequested) {
            error = targetAngle - firstAngle
            integral += error
            derivative = error - lastError
            outputChange = error * kp + integral * ki + derivative * kd
            leftBackDrive.power = -motorPower - outputChange
            leftFrontDrive.power = motorPower - outputChange
            rightBackDrive.power = motorPower + outputChange
            rightFrontDrive.power = -motorPower + outputChange
            lastError = error
        }
        stopDrivetrain()
    }

    fun imu_7573coords(current: Double, target: Double): Double {
        val forwards = Math.abs(current + Math.PI * 2 - target)
        val backwards = Math.abs(current - Math.PI * 2 - target)
        val normal = Math.abs(current - target)
        val min = Math.min(forwards, Math.min(backwards, normal))
        return if (min == forwards) {
            current + Math.PI * 2
        } else if (min == backwards) {
            current - Math.PI * 2
        } else {
            current
        }
    }

    fun wrapIMU(targetAngle: Double, left: Double, right: Double, threshold: Double, kp: Double, ki: Double, kd: Double) {
        //get the current value in radians
        //convert the target to radians
        var targetAngle = targetAngle
        var threshold = threshold
        targetAngle = Math.toRadians(targetAngle)
        var currentValue = imu_7573coords(firstAngle, targetAngle)
        //initialize PID variables
        var error = targetAngle - currentValue
        var derivative: Double
        var integral = 0.0
        var lastError = 0.0
        var output: Double
        //convert the threshold to radians
        threshold = Math.toRadians(threshold)
        useEncoders()
        while (Math.abs(error) > threshold && opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle - currentValue
            //integral is the summation of all the past error
            integral += error
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError
            //multiply each value by their respective constants and sum to get outuput
            output = error * kp + integral * ki + derivative * kd

            //set motor power based output value
            leftFrontDrive.power = output * left
            leftBackDrive.power = output * left
            rightFrontDrive.power = output * right
            rightBackDrive.power = output * right

            //get the current value from the IMU
            currentValue = imu_7573coords(firstAngle, targetAngle)
            telemetry.addData("Current Value", currentValue)
            telemetry.addData("Target", targetAngle)
            telemetry.addData("Left Power", leftBackDrive.power)
            telemetry.addData("Right Power", rightBackDrive.power)
            telemetry.update()
            //make the last error equal to the current error
            lastError = error
        }
        stopDrivetrain()
    }

    fun drivingPID(power: Double, sleepTime: Double, kp: Double, ki: Double, kd: Double) {
        useEncoders()
        val motorVel = power * 2800
        val targetAngle = Math.toDegrees(firstAngle)
        var error = 0.0
        var integral = 0.0
        var derivative = 0.0
        var lastError = 0.0
        leftFrontDrive.velocity = motorVel
        leftBackDrive.velocity = motorVel
        rightBackDrive.velocity = motorVel
        rightFrontDrive.velocity = motorVel
        var outputChange: Double
        val targetTime = runtime + sleepTime / 1000
        while (runtime < targetTime && !isStopRequested) {
            error = targetAngle - firstAngle
            integral += error
            derivative = error - lastError
            outputChange = error * kp + integral * ki + derivative * kd
            leftBackDrive.velocity = motorVel - outputChange
            leftFrontDrive.velocity = motorVel - outputChange
            rightBackDrive.velocity = motorVel + outputChange
            rightFrontDrive.velocity = motorVel + outputChange
            lastError = error
        }
        stopDrivetrain()
    }

    fun resetDriveEncoders() {
        leftFrontDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftBackDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightFrontDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightBackDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun moveToPosition(motorPower: Double, ticks: Int) {
        resetDriveEncoders()
        val motorVelocity = motorPower * 2700
        leftBackDrive.targetPosition = ticks
        leftFrontDrive.targetPosition = ticks
        rightBackDrive.targetPosition = ticks
        rightFrontDrive.targetPosition = ticks
        leftBackDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        leftFrontDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightBackDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightFrontDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightFrontDrive.velocity = motorVelocity
        rightBackDrive.velocity = motorVelocity
        leftBackDrive.velocity = motorVelocity
        leftFrontDrive.velocity = motorVelocity
        val wait = Future.WaitMilliseconds(5000)
        while (leftBackDrive.isBusy && !isStopRequested && !wait.poll()) {
            telemetry.addData("Status", "not there yet")
            telemetry.addData("left back", leftBackDrive.currentPosition)
            telemetry.addData("left front", leftFrontDrive.currentPosition)
            telemetry.addData("right back", rightBackDrive.currentPosition)
            telemetry.addData("right front", rightFrontDrive.currentPosition)
            telemetry.addData("busy1", leftBackDrive.isBusy)
            telemetry.addData("busy2", leftFrontDrive.isBusy)
            telemetry.addData("busy3", rightFrontDrive.isBusy)
            telemetry.addData("busy4", rightBackDrive.isBusy)
            telemetry.addData("Target", leftBackDrive.targetPosition)
            telemetry.update()
        }
        stopDrivetrain()
    }

    fun strafe(power: Double, sleepTime: Int) {
        resetDriveEncoders()
        useEncoders()
        leftBackDrive.power = -power
        leftFrontDrive.power = power
        rightBackDrive.power = power
        rightFrontDrive.power = -power
        sleep(sleepTime.toLong())
        leftBackDrive.power = 0.0
        leftFrontDrive.power = 0.0
        rightBackDrive.power = 0.0
        rightFrontDrive.power = 0.0
    }

    private var lift_killed = false
    protected fun set_lift_checked(ticks: Int) {
        val pos = Math.max(LIFT_MIN, Math.min(ticks, LIFT_MAX))
        Lift_1.targetPosition = pos
        Lift_2.targetPosition = pos
        if (lift_killed) {
            Lift_1.mode = DcMotor.RunMode.RUN_TO_POSITION
            Lift_2.mode = DcMotor.RunMode.RUN_TO_POSITION
            Lift_1.power = 1.0
            Lift_2.power = 1.0
        }
    }

    protected fun kill_lift() {
        Lift_1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Lift_2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Lift_1.power = 0.0
        Lift_2.power = 0.0
        Lift_1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        Lift_2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lift_killed = true
    }

    companion object {
        const val ticks_per_inch = 537 / 12
        var running_opmode: RobotOpModeRe? = null
        const val LIFT_MIN = -100
        const val LIFT_MAX = 3250
    }
}