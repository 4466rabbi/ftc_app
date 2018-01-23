package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.FourBar
import org.firstinspires.ftc.teamcode.hardware.IntelligentMotionEngine

@TeleOp(name = "Drive TeleOp")
//@Disabled
class DrivingTeleOp : LinearOpMode() {

    override fun runOpMode() {
        val drivetrain = IntelligentMotionEngine(this)
        val fourBar = FourBar(this)

        waitForStart()

        while (opModeIsActive()) {
            drivetrain.arcadeMecanum(gamepad1)
            when {
                gamepad2.x -> fourBar.setClaw(0.8)
                gamepad2.b -> fourBar.setClaw(0.4)
                gamepad2.left_trigger > 0F -> fourBar.runLift(-0.25)
                gamepad2.right_trigger > 0F -> fourBar.runLift(0.25)
            }
            telemetry.addData("Orientation: ", drivetrain.getOrientation())
            telemetry.addData("Lift powers: ", fourBar.getMotorPowers())
            telemetry.addData("Claw positions: ", fourBar.getClawPositions())
            telemetry.addData("Encoder Vals: ", drivetrain.getEncoderPositions())
            telemetry.update()
        }
        drivetrain.runMotors(0.0)
    }

}