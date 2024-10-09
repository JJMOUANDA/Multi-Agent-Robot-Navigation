import java.util.ArrayList;
import java.util.List;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;

public class AutonomyTwo extends Robot {
	private int timeStep;
	private DistanceSensor[] distanceSensor;	
	private Motor leftMotor;
	private Motor rightMotor;
	private Camera camera;
	private Emitter emitter;
	private Receiver receiver;
	private LED[] leds;
	
	
	public AutonomyTwo() {
		timeStep = 128;  // set the control time step


		// Sensors initialization 
		// IR distance sensors
		distanceSensor = new DistanceSensor[8];
		String[] sensorNames = {
				"ps0", "ps1", "ps2", "ps3",
				"ps4", "ps5", "ps6", "ps7"
		};

		for (int i = 0; i < 8; i++) {
			distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
			distanceSensor[i].enable(timeStep);
		}

		// Camera
		camera=this.getCamera("camera");
		camera.enable(timeStep);
		camera.recognitionEnable(timeStep);

		// Actuators initialization
		// Motors
		leftMotor = this.getMotor("left wheel motor");
		rightMotor = this.getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		leftMotor.setVelocity(0.0);
		rightMotor.setVelocity(0.0);

		// LEDS
		leds = new LED[10];
		String[] ledsNames = {
				"led0", "led1", "led2", "led3",
				"led4", "led5", "led6", "led7",
				"led8", "led9"
		};
		for (int i = 0; i < 10; i++) {
			leds[i] = this.getLED(ledsNames[i]);
		}
	}

	/**
	 * 
	 * @return a double array with values for each IR sensor 
	 * Each value is between approx. [67 ; 750 (very close - contact)]
	 * (see https://cyberbotics.com/doc/guide/epuck)
	 */
	protected double[] readDistanceSensorValues() {
		// read sensors outputs
		double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
		for (int i = 0; i < 8 ; i++)
			psValues[i] = distanceSensor[i].getValue();

		return psValues;
	}

	/**
	 * 
	 * @param left : a value between [-100;100]%
	 * @param right : a value between [-100;100]%
	 */
	protected void move(double left, double right) {
		double max=6.2;
		getMotor("left wheel motor").setVelocity(left * max / 100);
		getMotor("right wheel motor").setVelocity(right * max / 100);
	}
	
	/**
	 * Switch on / off a LED according to its num ([0;9])
	 * @param num
	 * @param on : true if the LED is to be switched on, 
	 * or false if the LED is to be switched off
	 */
	protected void setLED(int num, boolean on) {
		if(num < 10) {
			leds[num].set(on ? 1 : 0);
		}
	}

	/**
	 * 
	 * @return an empty list if nothing is detected by the camera, 
	 * a list of CameraRecognitionObject otherwise (see https://cyberbotics.com/doc/reference/camera#camera-recognition-object)
	 */
	protected List<CameraRecognitionObject> cameraDetection() {
		ArrayList<CameraRecognitionObject> detected=new ArrayList<>();
		int nb=camera.getRecognitionNumberOfObjects();
		if(nb >0) {
			CameraRecognitionObject[] objects=camera.getRecognitionObjects();
			for(int i=0;i<objects.length;i++) {
				detected.add(objects[i]);
			}
		}
		return detected;
	}

	/**
	 * Look in a List of camera detected objects if the target is one of them 
	 * @param detected: a List of camera detected objects
	 * @return the target (a specific CameraRecognitionObject) or null
	 */
	protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("cible") == 0)
				return ob;
		}
		return null;		
	}

	/**
	 * Look in a List of camera detected objects if other robots are recognized 
	 * @param detected: a List of camera detected objects
	 * @return a List of CameraRecognitionObject representing the other robots
	 */
	protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
		ArrayList<CameraRecognitionObject> robots=new ArrayList<>();
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("e-puck") == 0)
				robots.add(ob);
		}
		return robots;		
	}

        /**
         * The main method of the robot behaviour
         */	
        public void run() {		
            // Current state of the robot
            String currentState = "Forward";
            
            // Control loop
            while (step(timeStep) != -1) {	
                // Get sensor and camera readings
                double[] distances = readDistanceSensorValues();
                List<CameraRecognitionObject> objects = cameraDetection();
                CameraRecognitionObject target = targetDetected(objects);
                
                switch (currentState) {
                    case "Forward":
                        // Check if there's an obstacle on the left (distances[5] or distances[6])
                        if (distances[5] > 80 || distances[6] > 80) {
                            currentState = "TurnRight";
                        } 
                        // Check if there's an obstacle on the right (distances[1] or distances[2])
                        else if (distances[1] > 80 || distances[2] > 80) {
                            currentState = "TurnLeft";
                        } 
                        // Check if the target is detected
                        else if (target != null) {
                            currentState = "MoveToTarget";
                        } 
                        // Otherwise, continue moving forward
                        else {
                            move(50, 50);  // Move forward at 50% speed
                        }
                        break;
        
                    case "TurnLeft":
                        // Turn left to avoid obstacle on the right
                        move(-50, 50);  // Left wheel moves backward, right wheel moves forward
                        // If the path is clear in front (distances[0] and distances[7]), go back to Forward
                        if (distances[0] < 70 && distances[7] < 70) {
                            currentState = "Forward";
                        }
                        break;
        
                    case "TurnRight":
                        // Turn right to avoid obstacle on the left
                        move(50, -50);  // Right wheel moves backward, left wheel moves forward
                        // If the path is clear in front (distances[0] and distances[7]), go back to Forward
                        if (distances[0] < 70 && distances[7] < 70) {
                            currentState = "Forward";
                        }
                        break;
        
                    case "MoveToTarget":
                        // Move towards the target when detected
                        move(60, 60);  // Move faster towards the target
                        // If the target is no longer detected, go back to Forward
                        if (target == null) {
                            currentState = "Forward";
                        }
                        break;
                }
            }
        }

	public static void main(String[] args) {
		AutonomyTwo controller = new AutonomyTwo();
		controller.run();
	}
}