package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.FourBar
import org.firstinspires.ftc.teamcode.hardware.Rebound
import java.util.*

@TeleOp(name = "Rebound TeleOp")
@Disabled
class ReboundTeleOp : OpMode() {

    private lateinit var drivetrain : Rebound
    private lateinit var glyphHandler: FourBar
    private var tankStyleControl : Boolean = false

    override fun init() {
        // setup
        val lf: DcMotor = hardwareMap.dcMotor.get("lf")
        val lb: DcMotor = hardwareMap.dcMotor.get("lb")
        val rf: DcMotor = hardwareMap.dcMotor.get("rf")
        val rb: DcMotor = hardwareMap.dcMotor.get("rb")
        val imu: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")
        drivetrain = Rebound(this, lf, lb, rf, rb, imu)
        drivetrain.initialize(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT)
        telemetry.addLine("rebound. v1.0")
        telemetry.addLine("Drivetrain initialized!")
        // check if encoders are working
        if (Collections.frequency(drivetrain.getMotorModes(), DcMotor.RunMode.RUN_USING_ENCODER) == 4) {
            telemetry.addLine("Encoders enabled!")
        }
        telemetry.addData("Initial Orientation: ", drivetrain.getOrientation())
        //mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.up_and_away)
        // set up the claw
        val lift_left : DcMotor = hardwareMap.dcMotor.get("lift_left")
        val lift_right : DcMotor = hardwareMap.dcMotor.get("lift_right")
        val claw_left : Servo = hardwareMap.servo.get("claw_left")
        val claw_right : Servo = hardwareMap.servo.get("claw_right")
        glyphHandler = FourBar(this)
    }

    override fun loop() {
        val orientation = drivetrain.getOrientation()
        telemetry.addData("Orientation: ", orientation)
        telemetry.addData("Jordan Drive ", if(tankStyleControl) "Enabled." else "Disabled.")
        when {
            gamepad2.dpad_up -> glyphHandler.runLift(0.25)
            gamepad2.dpad_down -> glyphHandler.runLift(-0.25)
            gamepad2.a -> glyphHandler.setClaw(0.8)
            gamepad2.b -> glyphHandler.setClaw(0.4)
            !gamepad2.dpad_up || gamepad2.dpad_down -> glyphHandler.runLift()

            gamepad1.a -> drivetrain.pTurn(90, 0.04)
            gamepad1.b -> drivetrain.pTurn(0, 0.04)
            gamepad1.left_trigger > 0 -> drivetrain.arcadeMecanum((-gamepad1.left_trigger).toDouble(), 0.0, 0.0)
            gamepad1.right_trigger > 0 -> drivetrain.arcadeMecanum(gamepad1.right_trigger.toDouble(), 0.0, 0.0)
        }

        glyphHandler.runLift(0.33*gamepad2.left_stick_y)

        if (tankStyleControl) {drivetrain.tankMecanum(gamepad1)} else {drivetrain.arcadeMecanum(gamepad1)}
        telemetry.addData("Lift Powers: ", glyphHandler.getMotorPowers())
    }
}