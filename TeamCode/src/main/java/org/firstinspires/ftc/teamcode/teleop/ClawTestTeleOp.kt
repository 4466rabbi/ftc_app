package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo

@Autonomous(name = "Claw Test Env.")
class ClawTestTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val claw_left: Servo = hardwareMap.servo.get("claw_left")
        val claw_right: Servo = hardwareMap.servo.get("claw_right")
        claw_left.direction = Servo.Direction.REVERSE
        claw_left.position = 0.15
        claw_right.position = 0.0
        waitForStart()
        claw_left.position = 0.25
        claw_right.position = 0.1
        sleep(5000)
        claw_left.position = 0.35
        claw_right.position = 0.2
        sleep(5000)
        claw_left.position = 0.45
        claw_right.position = 0.3
        sleep(5000)
        claw_left.position = 0.55
        claw_right.position = 0.4
        sleep(5000)
        claw_left.position = 0.65
        claw_right.position = 0.5
        sleep(5000)
        claw_left.position = 0.75
        claw_right.position = 0.6
        sleep(5000)
        claw_left.position = 0.85
        claw_right.position = 0.7
        sleep(5000)
        claw_left.position = 0.95
        claw_right.position = 0.8
        sleep(5000)
        claw_left.position = 1.0
        claw_right.position = 0.9
        sleep(5000)
        claw_left.position = 1.0
        claw_right.position = 1.0
        sleep(5000)
    }

}