package edu.tum.cs.util;

public class Stopwatch {
    
    private long startTime = 0;
    private long stopTime = 0;
    private long elapsed = 0;
    private boolean running = false;
    
    public void start() {        
        this.running = true;
        this.startTime = System.currentTimeMillis();
    }
    
    public void stop() {
        this.stopTime = System.currentTimeMillis();
        this.running = false;
        elapsed += stopTime - startTime;
    }
    
    /**
     * gets elapsed time in msecs
     * @return
     */
    public long getElapsedTime() {
        if(running)
        	return elapsed + System.currentTimeMillis() - startTime;
        else
            return elapsed;
    }    
    
    /**
     * gets elapsed time in seconds
     * @return
     */
    public double getElapsedTimeSecs() {
    	return (double)getElapsedTime() / 1000;
    }
}