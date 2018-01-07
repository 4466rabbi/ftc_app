package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

class ClawLift(private val parentOpMode: OpMode,
               private val lift_left: DcMotor, private val lift_right: DcMotor,
               private val claw_left: Servo, private val claw_right: Servo) {

    fun initialize() {
        lift_left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift_right.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift_right.direction = DcMotorSimple.Direction.REVERSE
        claw_left.direction = Servo.Direction.REVERSE
        parentOpMode.telemetry.addLine("4466 ClawLift v1.0")
        parentOpMode.telemetry.addLine("Running as expected.")
    }

    fun getMotorPowers() : List<Double> = listOf(lift_left.power, lift_right.power)

    fun openClaw(position: Double = 0.8) {
        claw_left.position = position
        claw_right.position = position
    }

    fun closeClaw(position: Double = 0.4) {
        claw_left.position = position
        claw_right.position = position
    }

    fun runLift(power: Double = 0.0) {
        lift_left.power = power
        lift_right.power = power
    }
}