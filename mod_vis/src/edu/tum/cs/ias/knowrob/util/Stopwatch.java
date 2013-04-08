package edu.tum.cs.ias.knowrob.util;

/**
 * Stop watch millisecond accuracy. For measuring different time spans.
 * 
 * @author Stefan Profanter
 *
 */
public class Stopwatch {
    
	/**
	 * Stopwatch start time
	 */
    private long startTime = 0;
	/**
	 * Stopwatch stop time
	 */
    private long stopTime = 0;
	/**
	 * Stopwatch elapsed time
	 */
    private long elapsed = 0;
    /**
     * Is stopwatch currently running?
     */
    private boolean running = false;
    
    /**
     * Start stopwatch
     */
    public void start() {        
        this.running = true;
        this.startTime = System.currentTimeMillis();
    }
    
    /**
     * Stop stopwatch
     */
    public void stop() {
        this.stopTime = System.currentTimeMillis();
        this.running = false;
        elapsed += stopTime - startTime;
    }
    
    /**
     * gets elapsed time in msecs
     * @return elapsed time in milliseconds
     */
    public long getElapsedTime() {
        if(running)
        	return elapsed + System.currentTimeMillis() - startTime;
        else
            return elapsed;
    }    
    
    /**
     * gets elapsed time in seconds
     * @return elapsed time in seconds
     */
    public double getElapsedTimeSecs() {
    	return (double)getElapsedTime() / 1000;
    }
}