package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.Rebound

@Autonomous(name = "Blue Far", group = "Encoders")
//@Disabled
class EncoderAutoBlueFar : LinearOpMode() {

    private lateinit var drivetrain : Rebound

    override fun runOpMode() {
        val lf : DcMotor = hardwareMap.dcMotor.get("lf")
        val lb : DcMotor = hardwareMap.dcMotor.get("lb")
        val rf : DcMotor = hardwareMap.dcMotor.get("rf")
        val rb : DcMotor = hardwareMap.dcMotor.get("rb")
        val imu : BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")
        drivetrain = Rebound(this, lf, lb, rf, rb, imu)
        drivetrain.initialize(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
        //val mediaPlayer: MediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.up_and_away)
        //mediaPlayer.start()
        waitForStart()
        telemetry.addLine("Initial Travel.")
        drivetrain.setTargetTravelInches(14.0)
        drivetrain.setMotorPowers(0.5)
        while (drivetrain.areMotorsBusy()) {
            sleep(25)
        }
        drivetrain.stopAndResetEncoders()
        sleep(1000)

        /*do {
            drivetrain.pTurn(90, 0.4)
        } while (drivetrain.getMotorPowers().first() != 0.0)
        sleep(1000)*/

        drivetrain.setTargetRotationStrafe(1.0, -1.0)
        drivetrain.setMotorPowers(0.5)
        while (drivetrain.areMotorsBusy()) {
            sleep(25)
        }
        drivetrain.stopAndResetEncoders()

        drivetrain.setTargetTravelInches(2.0)
        drivetrain.setMotorPowers(0.5)
        while (drivetrain.areMotorsBusy()) {
            sleep(25)
        }
        drivetrain.stopAndResetEncoders()
        telemetry.addLine("Auto routine completed!")
        while (opModeIsActive()) {
            sleep(100)
        }
        opModeIsActive()
        //mediaPlayer.stop()
    }

}