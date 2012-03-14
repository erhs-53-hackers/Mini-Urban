package Team53;

import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.addon.ColorSensorHT;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.PIDController;
import lejos.robotics.Color;

public class Robot {

    LightSensor lightSensor;
    ColorSensorHT RcolorSensor;
    ColorSensorHT LcolorSensor;
    PIDController pid;
    DifferentialPilot pilot;
    float speed = 400;
    final int parkingSpotLength = 22;
    final int parkingSpotDistance = 22;
    int lastValue = 0, target;
    float P, I, D;

    public Robot() {
        RcolorSensor = new ColorSensorHT(SensorPort.S1);
        LcolorSensor = new ColorSensorHT(SensorPort.S2);
        //lightSensor = new LightSensor(SensorPort.S1);
        System.out.println("Press left to calibrate");
        while (!Button.LEFT.isPressed()) {
        }
        System.out.println("Calibrating...");
        LcolorSensor.initBlackLevel();
        RcolorSensor.initBlackLevel();
        System.out.println("Press enter to start");
        while (!Button.ENTER.isPressed()) {
        }
    }

    public void calibratePID(float kp, float ki, float kd) {
        P = kp;
        I = ki;
        D = kd;

        pid.setPIDParam(PIDController.PID_KP, kp);
        pid.setPIDParam(PIDController.PID_KI, ki);
        pid.setPIDParam(PIDController.PID_KD, kd);

    }

    public void setColor(int color) {
        target = color;
        pid = new PIDController(color);
    }

    public void calibratePilot(float wheelDiameter, float trackWidth) {
        pilot = new DifferentialPilot(wheelDiameter, trackWidth, Motor.C, Motor.B);
        pilot.setTravelSpeed(25);
        pilot.setRotateSpeed(10);

    }

    public void turnLeft() {
        System.out.println("Turning...");
        pilot.travel(5);
        pilot.setTravelSpeed(2);
        Motor.B.setSpeed(200);
        Motor.C.setSpeed(25);
        Motor.B.forward();
        Motor.C.forward();
        while (!"white".equals(getColor(LcolorSensor))) {
            System.out.println(getColor(LcolorSensor));
        }
        pilot.stop();

        pilot.setTravelSpeed(25);
    }

    public void turnRight() {
        System.out.println("Turning...");
        pilot.travel(5);
        pilot.setTravelSpeed(2);
        Motor.B.setSpeed(25);
        Motor.C.setSpeed(200);
        Motor.B.forward();
        Motor.C.forward();

        while ("black".equals(getColor(RcolorSensor))) {
            System.out.println(getColor(RcolorSensor));
        }
        pilot.stop();

        pilot.setTravelSpeed(25);
    }

    public void printColors() {
        //System.out.println("Yellow: " + RcolorSensor.getRGBComponent(ColorSensorHT.YELLOW));
        //System.out.println("White: " + RcolorSensor.getRGBComponent(ColorSensorHT.WHITE));
        System.out.println("R Black: " + RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
        //System.out.println("R Black: " + RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));
        //System.out.println("Left Red: " + LcolorSensor.getRGBComponent(ColorSensorHT.RED));
        //System.out.println("Left Green: " + LcolorSensor.getRGBComponent(ColorSensorHT.GREEN));
        //System.out.println("Left Blue: " + LcolorSensor.getRGBNormalized(ColorSensorHT.BLUE));




    }

    boolean checkForStop(ColorSensorHT sensor) {

        if ("red".equals(getColor(sensor))) {
            return true;
        }
        return false;

    }

    public void checkColor(ColorSensorHT sensor) {
        if (sensor.getRGBComponent(ColorSensorHT.WHITE) == 255) {
            speed = 400;
        } else {
            speed = 200;
        }
        //System.out.println("sensor:" + sensor.getRGBComponent(ColorSensorHT.BLACK));

    }

    private String getColor(ColorSensorHT xcolor) {
        int[] xcolorInput = {xcolor.getRGBComponent(Color.RED), xcolor.getRGBComponent(Color.GREEN), xcolor.getRGBComponent(Color.BLUE)};
        int colorAvgValue = (xcolorInput[0] + xcolorInput[1] + xcolorInput[2]) / 3;
        int redVal = (xcolorInput[0] - colorAvgValue);
        int greenVal = (xcolorInput[1] - colorAvgValue);
        int blueVal = (xcolorInput[2] - colorAvgValue);
        if (Math.abs(redVal) < 15 && Math.abs(greenVal) < 15 && Math.abs(blueVal) < 15) {
            if (xcolorInput[0] <= 100 && xcolorInput[1] <= 100 && xcolorInput[2] <= 100) {
                return "black";


            }
        } else if (xcolorInput[0] > 100 && xcolorInput[1] > 100 && xcolorInput[2] > 100) {
            return "white";
        } else if (redVal > 15 && greenVal <= 0) { //Consideration of blue value unnessesary;
            return "red";
        } else if (redVal > 15 && greenVal > 15) { //Consideration of blue value unnessesary
            return "yellow";
        } else if (redVal <= 0 && greenVal <= 10 && blueVal > 15) {
            return "blue";
        } else if (redVal <= 0 && greenVal > 15) {
            return "green";
        }
        return "???";
    }

    public void hugRight() {
        pid = new PIDController(target);
        pid.setPIDParam(PIDController.PID_KP, P);
        pid.setPIDParam(PIDController.PID_KI, I);
        pid.setPIDParam(PIDController.PID_KD, D);
        while (!checkForStop(LcolorSensor)) {
            checkColor(RcolorSensor);


            float value = pid.doPID(RcolorSensor.getRGBComponent(ColorSensorHT.BLACK));

            //System.out.println(value);
            System.out.println("Following...");

            Motor.B.setSpeed(speed - (speed * (value / 128 / 5)));
            Motor.B.forward();
            Motor.C.setSpeed(speed + (speed * (value / 128 / 5)));
            Motor.C.forward();
        }
        pilot.stop();




        //checkForStop(Direction.Left);


    }

    public void hugLeft() {
        pid = new PIDController(target);
        pid.setPIDParam(PIDController.PID_KP, P);
        pid.setPIDParam(PIDController.PID_KI, I);
        pid.setPIDParam(PIDController.PID_KD, D);
        while (!checkForStop(RcolorSensor)) {
            checkColor(LcolorSensor);


            float value = pid.doPID(LcolorSensor.getRGBComponent(ColorSensorHT.BLACK));

            //System.out.println(value);
            System.out.println("Following...");

            Motor.B.setSpeed(speed + (speed * (value / 128 / 5)));
            Motor.B.forward();
            Motor.C.setSpeed(speed - (speed * (value / 128 / 5)));
            Motor.C.forward();

        }

        //checkForStop(Direction.Right);


    }

    public void park(int spotNum, int parkingSide, boolean flag) {
        /*if (parkingSide == 1) {
            if (LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) > 60 & LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) < 80) {
                pilot.travel(parkingSpotDistance);
                turnLeft();
                pilot.travel(parkingSpotLength);
                flag = true;
            } 
            else {
                        while (!checkForStop(RcolorSensor)) {
                checkColor(LcolorSensor);
                float value = pid.doPID(LcolorSensor.getRGBComponent(ColorSensorHT.BLUE));
                //System.out.println(value);
                System.out.println("Following...");

                Motor.B.setSpeed(speed + (speed * (value / 128 / 5)));
                Motor.B.forward();
                Motor.C.setSpeed(speed - (speed * (value / 128 / 5)));
                Motor.C.forward();
            
        }

            }
        }   
        else if (parkingSide == 0) {
              if (LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) > 60 & LcolorSensor.getRGBComponent(ColorSensorHT.BLUE) < 80) {
                pilot.travel(17 * spotNum);
                turnRight();
                pilot.travel(parkingSpotLength);
                flag = false;
                } else { 
                  hugRight(); 
                } 
       }
         * 
         */
      while (Motor.B.getTachoCount() == 7.2*360*spotNum) {
            checkColor(RcolorSensor);


            float value = pid.doPID(LcolorSensor.getRGBComponent(ColorSensorHT.BLUE));

            //System.out.println(value);
            System.out.println("Following...");

            Motor.B.setSpeed(speed + (speed * (value / 128 / 5)));
            Motor.B.forward();
            Motor.C.setSpeed(speed - (speed * (value / 128 / 5)));
            Motor.C.forward();

        }

  }
  
    

    public void getOutOfpark(int parkingSide) {
        pilot.travel(-parkingSpotLength);
        if (parkingSide == 1) {
            turnRight();
        } else if (parkingSide == 0) {
            turnLeft();
        }
    }
}