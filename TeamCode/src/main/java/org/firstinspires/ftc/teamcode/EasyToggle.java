package org.firstinspires.ftc.teamcode;

public class EasyToggle {
    private String button;
    private boolean buttonWas;
    private int controller;
    private boolean buttonIs;
    private boolean buttonChange;

    public EasyToggle(String b, boolean w, int o, boolean i, boolean c){
        button = b;
        buttonWas = w;
        controller = o;
        buttonIs = i;
        buttonChange = c;
    }

    public void updateStart(boolean u){
        buttonIs = u;
    }

    public void updateEnd(){
        buttonWas = buttonIs;
    }

    public boolean nowTrue(){
        return(buttonIs && !buttonWas);
    }

    public boolean nowFalse(){
        return(!buttonIs && buttonWas);
    }



}
