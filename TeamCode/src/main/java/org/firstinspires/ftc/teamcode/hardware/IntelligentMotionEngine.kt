package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max

/**
 * The Intelligent Motion Engine uses an array of sensory inputs to allow for accurate, precise, and predictable
 * movement of the robot.
 */
class IntelligentMotionEngine(private val parentOpMode: OpMode) {

    private var lf: DcMotorEx = parentOpMode.hardwareMap.get(DcMotorEx::class.java, "lf")
    private var lb: DcMotorEx = parentOpMode.hardwareMap.get(DcMotorEx::class.java, "lb")
    private var rf: DcMotorEx = parentOpMode.hardwareMap.get(DcMotorEx::class.java, "rf")
    private var rb: DcMotorEx = parentOpMode.hardwareMap.get(DcMotorEx::class.java, "rb")
    private var imu: BNO055IMU = parentOpMode.hardwareMap.get(BNO055IMU::class.java, "imu")

    //driving math constants
    private val COUNTS_PER_ROTATION = 1120
    private val WHEEL_DIAMETER = 4.0
    private val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI

    init {
        rf.direction = DcMotorSimple.Direction.REVERSE
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER)
        setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.FLOAT)
        // imu setup
        val parameters: BNO055IMU.Parameters = BNO055IMU.Parameters()
        with(parameters) {
            mode = BNO055IMU.SensorMode.IMU
            angleUnit = BNO055IMU.AngleUnit.DEGREES
            accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            calibrationDataFile = "BNO055IMUCalibration.json" // see the calibration sample opmode
            loggingEnabled = true
            loggingTag = "IMU"
            accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        }
        imu.initialize(parameters)
    }

    fun runMotors(lfPow: Double, lbPow: Double, rfPow: Double, rbPow: Double) {
        lf.power = lfPow
        lb.power = lbPow
        rf.power = rfPow
        rb.power = rbPow
    }
    fun runMotors(lPow: Double, rPow: Double) = runMotors(lPow, lPow, rPow, rPow)
    fun runMotors(power: Double) = runMotors(power, power)

    /**
     * Sets motor powers based on an Inverse Kinematics algorithm.
     * @param vtX translational power along the X axis (-1 to 1)
     * @param vtY translational power along the Y axis (-1 to 1)
     * @param vR rotational power around the Z axis (-1 to 1)
     */
    fun arcadeMecanum(vtX: Double, vtY: Double, vR: Double) {
        //telemetry.addData("Inputs: ", listOf(vtX, vtY, vR))
        // calculate raw motor powers
        val lfPow : Double = vtY + vtX - vR
        val rfPow : Double = vtY - vtX + vR
        val lbPow : Double = vtY - vtX - vR
        val rbPow : Double = -(vtY + vtX + vR)
        // get the max wheel power
        val wMax : Double = max(lfPow, max(rfPow, max(lbPow, rbPow)))
        //telemetry.addData("Max Speed: ", wMax)
        // scale the motor powers
        val powers: List<Double> = listOf(lfPow, rfPow, lbPow, rbPow).map {x -> x/ max(1.0, wMax) }
        // set the motor powers
        lf.power = powers[0]
        rf.power = powers[1]
        lb.power = powers[2]
        rb.power = powers[3]
        parentOpMode.telemetry.addData( "Powers: ", getMotorPowers())
    }
    /**
     * Sets motor powers based on an Inverse Kinematics algorithm using input from a Gamepad.
     * Two joysticks are used - the X and Y axes of the left joystick control the linear motion of the robot,
     * and the X axis of the right joystick controls rotational motion.
     * @param gamepad the gamepad used to drive
     */
    fun arcadeMecanum(gamepad: Gamepad) = arcadeMecanum(Math.pow(-gamepad.left_stick_x.toDouble(), 3.0),
            Math.pow(gamepad.left_stick_y.toDouble(), 3.0),
            Math.pow(gamepad.right_stick_x.toDouble(), 3.0))

    /**
     * Sets motor powers based on an Inverse Kinematics algorithm designed to act similarly to a tank drive system
     * when used with a Gamepad.
     * @param left_y the left joystick Y axis
     * @param right_y the right joystick y axis
     * @param left_trigger the left trigger
     * @param right_trigger the right trigger
     * @param kT a turning multiplier, used to tune gamepad sensitivity
     */
    fun tankMecanum(left_y: Double, right_y: Double,
                    left_trigger: Double, right_trigger: Double, kT: Double = 0.5) {
        // set up movement variables
        val vtY : Double = (left_y + right_y) / 2
        val vR : Double = (left_y - right_y) / 2
        val vtX : Double = right_trigger - left_trigger
        // then plug those into arcadeMecanum() - we're done!
        arcadeMecanum(vtX = vtX, vtY = vtY, vR = (kT*vR))
    }
    /**
     * Sets motor powers based on an Inverse Kinematics algorithm using a Gamepad, designed to work like a tank drive.
     * The joysticks behave identically to a tank drive, and the triggers allow for strafing.
     * @param gamepad the gamepad used to drive
     */
    fun tankMecanum(gamepad: Gamepad) = tankMecanum(Math.pow(gamepad.left_stick_y.toDouble(), 3.0),
            Math.pow(gamepad.right_stick_y.toDouble(), 3.0),
            Math.pow(gamepad.left_trigger.toDouble(), 3.0),
            Math.pow(gamepad.right_trigger.toDouble(), 3.0))

    fun encoderCountDrive(target: Int, kP: Double = 0.03) {
        resetEncoders()
        var error = getEncoderPositions().average().toInt() - target
        while (abs(error) > 2) {
            val power = error * kP
            runMotors(power)
            error = target - getEncoderPositions().average().toInt()
        }
        runMotors(0.0)
        resetEncoders()
    }
    fun encoderRotationDrive(rTarget: Double, kP: Double = 0.03) {
        resetEncoders()
        val target = rTarget * COUNTS_PER_ROTATION
        encoderCountDrive(target.toInt(), kP)
    }
    fun encoderInchDrive(dTarget: Double, kP: Double = 0.03) {
        val rTarget = dTarget / WHEEL_CIRCUMFERENCE
        encoderRotationDrive(rTarget, kP)
    }

    fun encoderCountStrafe(target: Int, kP: Double = 0.03) {
        resetEncoders()
        var error = getEncoderPositions().average().toInt() - target
        while (abs(error) > 2) {
            val lCfgPower = -(error * kP)
            val rCfgPower = error * kP
            runMotors(lCfgPower, rCfgPower, rCfgPower, lCfgPower)
            error = target - getEncoderPositions().average().toInt()
        }
        runMotors(0.0)
        resetEncoders()
    }
    fun encoderRotationStrafe(rTarget: Double, kP: Double = 0.03) {
        val target = rTarget * COUNTS_PER_ROTATION
        encoderCountStrafe(target.toInt(), kP)
    }

    fun pTurn(target: Int, kP: Double = 0.04) {
        var error = getOrientation().toDouble() - target
        while (error > 2.0) {
            runMotors(lPow = error * kP, rPow = -error * kP)
            error = getOrientation().toDouble() - target
        }
        runMotors(0.0)
    }

    /**
     * Resets the encoders to be all fresh.
     */
    fun resetEncoders() {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    fun getEncoderPositions() : List<Int> = listOf(lf.currentPosition, rf.currentPosition, lb.currentPosition, rb.currentPosition)

    fun getOrientation() : Float =
            imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle

    fun getMotorPowers() : List<Double> = listOf(lf.power, rf.power, lb.power, rb.power)

    fun setMotorModes(mode: DcMotor.RunMode) {
        lf.mode = mode
        lb.mode = mode
        rf.mode = mode
        rb.mode = mode
    }

    fun setZeroPowerBehaviors(behavior: DcMotor.ZeroPowerBehavior) {
        lf.zeroPowerBehavior = behavior
        lb.zeroPowerBehavior = behavior
        rf.zeroPowerBehavior = behavior
        rb.zeroPowerBehavior = behavior
    }

}