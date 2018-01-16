package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.Rebound

@Autonomous(name = "So Auto Didn't Save...")
//@Disabled
class DeadReckoningAuto : LinearOpMode() {

    private lateinit var drivetrain : Rebound

    override fun runOpMode() {
        val lf : DcMotor = hardwareMap.dcMotor.get("lf")
        val lb : DcMotor = hardwareMap.dcMotor.get("lb")
        val rf : DcMotor = hardwareMap.dcMotor.get("rf")
        val rb : DcMotor = hardwareMap.dcMotor.get("rb")
        val imu : BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")
        drivetrain = Rebound(this, lf, lb, rf, rb, imu)
        drivetrain.initialize(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
        waitForStart()
    }


}