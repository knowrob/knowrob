package edu.tum.cs.ias.knowrob.util.datastructures;

@Deprecated
public class ActionData{
	public int episode_nr;
	public String goal;
	public String type;
	public float ent_x;
	public float ent_y;
	public float ent_z;
	public float b21_x;
	public float b21_y;
	
	public ActionData(int nepisode_nr, String ngoal, String ntype, float nent_x, float nent_y, float nent_z, float nb21_x, float nb21_y){		
		episode_nr = nepisode_nr;
		goal = ngoal;
		type = ntype;
		ent_x = nent_x;
		ent_y = nent_y;
		ent_z = nent_z;
		b21_x = nb21_x;
		b21_y = nb21_y;
	}

	
}
