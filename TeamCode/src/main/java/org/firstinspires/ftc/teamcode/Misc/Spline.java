package org.firstinspires.ftc.teamcode.Misc;
import java.util.ArrayList;

public class Spline {
    private double coefficient;
    private double power;

    private double x1;
    private double x2;

    private double y1;
    private double y2;

    private double radiusX;
    private double radiusY;
    private double radius;

    private int degrees;
    private ArrayList<Integer> degreesToTurn;
    private int instancesCalled;



    public Spline(double co, double p, double n, double m){
        coefficient = co;
        power = p;
        x1 = n;
        x2 = m;
        y1 = (coefficient * Math.pow(x1, power));
        y2 = (coefficient * Math.pow(x2, power));

        solveForDegrees();

        degreesToTurn = new ArrayList<Integer>();
        instancesCalled = 0;
    }

    public void calculateDerivative(){
        coefficient *= power;
        power -= 1;
    }

    public double calculateArcLength(){
        calculateDerivative();
        power *= 2;
        coefficient = Math.pow(coefficient, 2);
        calculateIntegral();

        return (coefficient * Math.pow(x2, power)) - (coefficient * Math.pow(x1, power)) + x2 - x1;
    }

    public void calculateIntegral(){
        power ++;
        coefficient /= power;
    }

    public void solveForRadius(){
        double RHS = Math.pow(y1, 2) - Math.pow(y2, 2) + Math.pow(x1, 2) - Math.pow(x2, 2);
        radiusX = RHS/(2 * (x1 - x2));
        radiusY = 0;

        radius = Math.sqrt(Math.pow(y2 - radiusY, 2) -  Math.pow(x2 - radiusX, 2));
    }

    public void solveForDegrees(){
        solveForRadius();
        degrees = (int) calculateArcLength() / (int) radius;
        if(degrees % 2 != 0)
            degrees++;

        while(degrees / 2 != 0){
            degreesToTurn.add(degrees);
        }
    }

    public double returnDegreesToTurn(){
        double output = 0.0;
        if(instancesCalled == degreesToTurn.size() * 2)
            output = 0.0;
        else if(instancesCalled >= 0 && instancesCalled < degreesToTurn.size())
            output = (degreesToTurn.get(instancesCalled));
        else
            output =  (degreesToTurn.get(instancesCalled % degreesToTurn.size())) * -1;

        instancesCalled++;
        return output;
    }
}
