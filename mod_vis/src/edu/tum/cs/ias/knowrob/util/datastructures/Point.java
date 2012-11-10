package edu.tum.cs.ias.knowrob.util.datastructures;

@Deprecated
public class Point {
	public float x;
	public float y;
	public float z;
	public float episode_nr;
	public float time;
	public float color;
	public String name;
	public String action = null;
	
	public Point(float xx, float yy, float zz, float epsd_nr, float ntime, String nname){
		x = xx;
		y = yy;
		z = zz;
		episode_nr = epsd_nr;
		time = ntime;
		name = nname;
		action="";
		color=1.0f;
	}
	
	public Point(float xx, float yy, float zz, float epsd_nr, float ntime, String nname, String naction){
		x = xx;
		y = yy;
		z = zz;
		episode_nr = epsd_nr;
		time = ntime;
		name = nname;
		action = naction;
		color=1.0f;
	}	
	
	public Point(float xx, float yy, float zz, float color, float epsd_nr, float ntime, String nname, String naction){
		x = xx;
		y = yy;
		z = zz;
		episode_nr = epsd_nr;
		time = ntime;
		name = nname;
		action = naction;
		color=1.0f;
	}
	
	public Point(){
		x = 0;
		y = 0;
		z = 0;
		episode_nr = 0;
		color=1.0f;
		time = 0;
		name = null;
		action = null;
	}
}
