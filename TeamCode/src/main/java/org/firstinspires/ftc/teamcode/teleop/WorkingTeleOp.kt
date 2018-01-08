package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.hardware.ClawLift
import kotlin.math.abs

@TeleOp(name = "WORKING TeleOp")
class WorkingTeleOp : OpMode() {

    lateinit var lf: DcMotor
    lateinit var lb: DcMotor
    lateinit var rf: DcMotor
    lateinit var rb: DcMotor
    lateinit var imu: BNO055IMU

    lateinit var clawLift: ClawLift

    override fun init() {
        lf = hardwareMap.dcMotor.get("lf")
        lb = hardwareMap.dcMotor.get("lb")
        rf = hardwareMap.dcMotor.get("rf")
        rb = hardwareMap.dcMotor.get("rb")

        lf.direction = DcMotorSimple.Direction.FORWARD
        lb.direction = DcMotorSimple.Direction.FORWARD
        rf.direction = DcMotorSimple.Direction.REVERSE
        rb.direction = DcMotorSimple.Direction.FORWARD

        lf.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        lb.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rf.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rb.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        lf.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lb.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rf.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rb.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val lift_left: DcMotor = hardwareMap.dcMotor.get("lift_left")
        val lift_right: DcMotor = hardwareMap.dcMotor.get("lift_right")
        val claw_left: Servo = hardwareMap.servo.get("claw_left")
        val claw_right: Servo = hardwareMap.servo.get("claw_right")
        clawLift = ClawLift(this, lift_left, lift_right, claw_left, claw_right)
        clawLift.initialize()

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
        // initialize the IMU
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        // apply IMU configuration
        imu.initialize(parameters)
    }

    override fun loop() {
        when {
            gamepad2.x -> clawLift.openClaw()
            gamepad2.b -> clawLift.closeClaw()
        }
        when {
            gamepad1.left_trigger > 0 -> strafe(2 * gamepad1.left_trigger.toDouble())
            gamepad1.right_trigger > 0 -> strafe(-2 * gamepad1.right_trigger.toDouble())
            gamepad1.left_bumper -> pTurn(0, 0.04)
            gamepad1.right_bumper -> pTurn(-90, 0.04)
            gamepad1.b -> {
                if (Math.abs(getOrientation()).toInt() == 180) {
                    pTurn(-90, 0.04)
                } else {
                    pTurn((Math.ceil(getOrientation().toDouble() / 90) * 90).toInt(), 0.04)
                }
            }
            gamepad1.x -> {
                if (Math.abs(getOrientation()).toInt() == 180) {
                    pTurn(90, 0.04)
                } else {
                    pTurn((Math.ceil(getOrientation().toDouble() / 90) * 90).toInt(), 0.04)
                }
            }
        }
        tankDrive(-gamepad1.left_stick_y.toDouble(), -gamepad1.right_stick_y.toDouble())
        clawLift.runLift(0.33 * gamepad2.left_stick_y)

    }

    fun strafe(power: Double) {
        lf.power = power
        lb.power = -power
        rf.power = -power
        rb.power = power
    }

    fun tankDrive(lPow: Double, rPow: Double) {
        lf.power = lPow
        lb.power = lPow
        rf.power = rPow
        rb.power = rPow
    }

    fun pTurn(target: Int, kP: Double) {
        var error = 3.0
        while (abs(error) > 2.0) {
            error = getOrientation().toDouble() - target
            tankDrive(lPow = error * kP, rPow = -error * kP)
        }
        tankDrive(0.0, 0.0)
    }

    fun getOrientation() =
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle

}

