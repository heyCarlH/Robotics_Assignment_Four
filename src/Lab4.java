import java.util.LinkedList;
import java.util.Queue;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class Lab4 {
	private static int heading;
	private static double[] coord; // coord[0] is x, coord[1] is y
	private static double[] hitPoint;
	private static EV3LargeRegulatedMotor rightWheel;
	private static EV3LargeRegulatedMotor leftWheel;
	private static SensorMode sonic;
	private static float[] sample_sonic;
	private static SensorMode touch;
	private static float[] sample_touch;
	private static SensorMode touch2;
	private static float[] sample_touch2;

	public static void main(String[] args) {
// initialize robot
		rightWheel = new EV3LargeRegulatedMotor(MotorPort.A);
		leftWheel = new EV3LargeRegulatedMotor(MotorPort.B);
		rightWheel.setSpeed(200);
		leftWheel.setSpeed(200);
		heading = 90;
		coord = new double[2];
		hitPoint = new double[2];
		coord[0] = 100.0;
		EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(MotorPort.D);

// start ultrasonic sensor
		EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S2);
		sonic = (SensorMode) ultrasensor.getDistanceMode();
		sample_sonic = new float[sonic.sampleSize()];
		sensorMotor.rotate(-90);

// set up bump sensor
		EV3TouchSensor touchsensor = new EV3TouchSensor(SensorPort.S1);
		touch = touchsensor.getTouchMode();
		sample_touch = new float[touch.sampleSize()];
		EV3TouchSensor touchsensor2 = new EV3TouchSensor(SensorPort.S3);
		touch2 = touchsensor2.getTouchMode();
		sample_touch2 = new float[touch2.sampleSize()];

// let's go!
		System.out.println("Press any key to start");
		Button.waitForAnyPress();
		long timer = System.currentTimeMillis();
		long debugTimer = System.currentTimeMillis();
		rotateToMLine();
		while (!onGoal()) {
			if (System.currentTimeMillis() - debugTimer >= 2000) {
				System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);
				debugTimer = System.currentTimeMillis();
			}

			touch.fetchSample(sample_touch, 0);
			touch2.fetchSample(sample_touch2, 0);

// keep fetching data from the bump sensor
			if (sample_touch[0] != 0.0 || sample_touch2[0] != 0.0) {
// record hit point
				hitPoint[0] = coord[0];
				hitPoint[1] = coord[1];

// stop motors with brakes on if bumped into something
				leftWheel.stop(true);
				rightWheel.stop(true);

// move backward a bit in order to turn
				leftWheel.startSynchronization();
				leftWheel.backward();
				rightWheel.backward();
				leftWheel.endSynchronization();
				Delay.msDelay(800);
				updateBackCoord(0.8);
				leftWheel.stop(true);
				rightWheel.stop(true);

// turn and move forward a bit
				turn(90);
				System.out.println(heading);
				System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);
				System.out.println("Bang!");
				leftWheel.forward();
				rightWheel.forward();
				Delay.msDelay(1000);
				updateCoord(1);
				traceObstacle();
				leftWheel.stop(true);
				rightWheel.stop(true);
				rotateToMLine();
			}

			if (System.currentTimeMillis() - timer >= 500) {
				rightWheel.forward();
				leftWheel.forward();
				updateCoord(0.5);
				timer = System.currentTimeMillis();
			}
		}
		rightWheel.stop();
		leftWheel.stop();
		System.out.println("on goal");
		System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);
		Delay.msDelay(8000);
	}

	private static boolean onGoal() {
		if (coord[0] > 170 && coord[0] < 190 && coord[1] > 190 && coord[1] < 210) {
			return true;
		}
		return false;
	}

	private static boolean onMLine() {
		if (coord[1] > (coord[0] * 2.25 - 235) && coord[1] < (coord[0] * 2.25 - 215)) {
			Sound.beep();
			return true;
		}
		return false;
	}

	private static void rotateToMLine() {
		int turnDegree = 0;
		if (heading > 66 && heading < 246) { // turn right
			turnDegree = heading - 66;
		} else if (heading <= 66 && heading >= 0) { // turn left
			turnDegree = heading - 66;
		} else { // turn left
			turnDegree = -(360 - heading + 66);
		}
		if (coord[1] > 180) { // if current y is above goal y
			turnDegree += 180;
		}
		turn(turnDegree);
	}

	private static void traceObstacle() {
		long timer = System.currentTimeMillis();
		long debugTimer = System.currentTimeMillis();

// take moving average of the sensor
		Queue<Float> lastThreeSamples = new LinkedList<>();
		float lastThreeSum = 0;
		for (int i = 0; i < 3; i++) {
			sonic.fetchSample(sample_sonic, 0);
			lastThreeSamples.add(sample_sonic[0] > 0.6f ? 0.5f : sample_sonic[0]);
			lastThreeSum += sample_sonic[0] > 0.6f ? 0.5f : sample_sonic[0];
		}
		float movingAverage = (float) lastThreeSum / 3;

		while (!(onMLine())) {
			if (onGoal()) {
				System.out.println("miehahahaha");
				break;
			}
			touch.fetchSample(sample_touch, 0);
			touch2.fetchSample(sample_touch2, 0);

// keep fetching data from the bump sensor
			if (sample_touch[0] != 0.0 || sample_touch2[0] != 0.0) {
// stop motors with brakes on if bumped into something
				leftWheel.stop(true);
				rightWheel.stop(true);

				leftWheel.startSynchronization();
				leftWheel.backward();
				rightWheel.backward();
				leftWheel.endSynchronization();
				Delay.msDelay(800);
				updateBackCoord(0.8);
				leftWheel.stop(true);
				rightWheel.stop(true);

				turn(90);
				leftWheel.forward();
				rightWheel.forward();
				Delay.msDelay(1000);
				updateCoord(1);
			}

			if (System.currentTimeMillis() - debugTimer >= 1000) {
				System.out.println("heading: " + heading);
				System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);
				debugTimer = System.currentTimeMillis();
			}

			if (System.currentTimeMillis() - timer >= 500) {
				if (movingAverage < 0.05f) { // turn right a lot
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(20);
				} else if (movingAverage < 0.1f) { // turn right a little
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(10);
				} else if (movingAverage < 0.2f) { // go straight

				} else if (movingAverage < 0.25f) { // turn left a little
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(-10);
				} else if (movingAverage < 0.8f) { // turn left a lot
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(-15);
				} else { // something goes way too wrong
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(-20);
					leftWheel.forward();
					rightWheel.forward();
					Delay.msDelay(1500);
					updateCoord(1.5);
				}
				updateCoord(0.5);
				timer = System.currentTimeMillis();
			}

			leftWheel.forward();
			rightWheel.forward();

// take new measurement
			lastThreeSum -= lastThreeSamples.poll();
			sonic.fetchSample(sample_sonic, 0);
			if (sample_sonic[0] > 0.6f) {
				lastThreeSamples.add(0.5f);
			} else {
				lastThreeSamples.add(sample_sonic[0]);
			}
			lastThreeSum += sample_sonic[0] > 0.6f ? 0.5f : sample_sonic[0];
			movingAverage = lastThreeSum / 3;
		}
	}

	private static void testCoord() {
		rightWheel.setSpeed(200);
		leftWheel.setSpeed(200);

		System.out.println("heading: " + heading);
		rightWheel.forward();
		leftWheel.forward();
		Delay.msDelay(5000);
		updateCoord(5);
		rightWheel.stop();
		leftWheel.stop();
		System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);

		turn(90);
		System.out.println("heading: " + heading);
		rightWheel.forward();
		leftWheel.forward();
		Delay.msDelay(5000);
		updateCoord(5);
		rightWheel.stop();
		leftWheel.stop();
		System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);

		turn(90);
		System.out.println("heading: " + heading);
		rightWheel.forward();
		leftWheel.forward();
		Delay.msDelay(5000);
		updateCoord(5);
		rightWheel.stop();
		leftWheel.stop();
		System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);

		turn(90);
		System.out.println("heading: " + heading);
		while (!backToRange()) {
			rightWheel.forward();
			leftWheel.forward();
			Delay.msDelay(500);
			updateCoord(0.5);
			System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);
		}

		rightWheel.stop();
		leftWheel.stop();

		Delay.msDelay(5000);
	}

	private static void testBack() {
		leftWheel.startSynchronization();
		leftWheel.backward();
		rightWheel.backward();
		leftWheel.endSynchronization();
		System.out.println(leftWheel.getSpeed());
		Delay.msDelay(800);
		updateCoord(0.8);
		System.out.println("x: " + (int) coord[0] + " y: " + (int) coord[1]);
		Delay.msDelay(5000);
	}

	private static void turn(int degree) {
		leftWheel.rotate(degree * 4);
		heading -= degree;
		heading = heading % 360;
		System.out.println(heading);
	}

	private static boolean backToRange() {
		if (-10 < coord[0] && coord[0] < 10 && -10 < coord[1] && coord[1] < 10)
			return true;
		return false;
	}

// @param: time is in seconds
	private static void updateCoord(double time) {
		double leftSpeed = (leftWheel.getSpeed() * Math.PI / 180) * 2.7;
		double rightSpeed = (rightWheel.getSpeed() * Math.PI / 180) * 2.7;
		double groundSpeed = (leftSpeed + rightSpeed) / 2;
		coord[0] = coord[0] + groundSpeed * time * Math.cos(heading * Math.PI / 180);
		coord[1] = coord[1] + groundSpeed * time * Math.sin(heading * Math.PI / 180);
	}

// @param: time is in seconds
	private static void updateBackCoord(double time) {
		double leftSpeed = (leftWheel.getSpeed() * Math.PI / 180) * 2.7;
		double rightSpeed = (rightWheel.getSpeed() * Math.PI / 180) * 2.7;
		double groundSpeed = (leftSpeed + rightSpeed) / 2;
		coord[0] = coord[0] - groundSpeed * time * Math.cos(heading * Math.PI / 180);
		coord[1] = coord[1] - groundSpeed * time * Math.sin(heading * Math.PI / 180);
	}

	private static double getGoalDistance(double[] point) {
		return Math.pow(Math.pow(point[0], 2) + Math.pow(point[1], 2), 0.5);
	}
}