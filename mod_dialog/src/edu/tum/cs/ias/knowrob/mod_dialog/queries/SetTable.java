package edu.tum.cs.ias.knowrob.mod_dialog.queries;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Vector;

import edu.tum.cs.ias.knowrob.mod_dialog.DialogModule;
import edu.tum.cs.logic.parser.ParseException;
import edu.tum.cs.probcog.InferenceResult;
import edu.tum.cs.probcog.Model;
import edu.tum.cs.probcog.ModelPool;
import edu.tum.cs.util.StringTool;

public class SetTable extends DialogQuery {

	protected int participants;
	protected String mealType;
	protected String[] people;
	protected HashMap<Integer,String> participant2name = new HashMap<Integer,String>();
	protected HashMap<String,Integer> name2participant = new HashMap<String,Integer>();
	protected Vector<String[]> evidence = new Vector<String[]>();
	protected ModelPool pool;
	protected Model model;
	protected String regexUtensil, regexGood;
	protected HashMap<String,String> utensils = new HashMap<String,String>();
	protected HashMap<String,String> goods = new HashMap<String,String>();

	public SetTable(DialogModule mod) throws IOException, ParseException, Exception {
		super(mod);		
		registerStates();
		pool = new ModelPool(DialogModule.findRosPackage("srldb") + "/models/models.xml");
		model = pool.getModel("tableSetting_fall09");
		
		// build up some domain data
		Vector<String[]> doms = model.getDomains();
		for(String[] dom : doms) {
			if(dom[0].equals("domutensilT")) {
				for(int i = 1; i < dom.length; i++)
					utensils.put(dom[i].toLowerCase(), dom[i]);
				regexUtensil = "(?:" + StringTool.join("|", utensils.keySet()) + ")";
			}
			if(dom[0].equals("objType_g")) {
				for(int i = 1; i < dom.length; i++)
					goods.put(dom[i].toLowerCase(), dom[i]);
				regexGood = "(?:" + StringTool.join("|", goods.keySet()) + ")";
			}
		}
		System.out.println("consumables: " + StringTool.join("|", goods.values()));
		System.out.println("usables: " + StringTool.join("|", utensils.values()));
	}
	
	private void registerStates() {
		this.dialog_module.registerState("set_table_num_participants");
		this.dialog_module.registerState("set_table_family_members");
		this.dialog_module.registerState("set_table_meal_type");
		this.dialog_module.registerState("hypo_meal");
	}
	
	public static String capitalize(String s) {
		return s.substring(0, 1).toUpperCase() + s.substring(1);
	}

	@Override
	public String process(String q) throws IOException, ParseException, Exception {
		q = q.toLowerCase().replace(".", "").replace("?", "");
		
		String regexPersonName = "(?:anna|bert|dorothy|emily|charly)";
		String regexMealType = "(?:breakfast|lunch|dinner)";
		String regexNumber = "(?:one|two|three|four)";
		String regexGeneralParticipant = String.format("(?:%s|participant %s)", regexPersonName, regexNumber);
	
		if(match("start over", q)) {
			setTopLevelState();
			return "OK.";
		}
		
		if(this.haveState(DialogModule.TOP_LEVEL_STATE)) {
			evidence.clear();
			
			// initiate table setting
			if(match("(?:please )?set the table", q)) {
				setState("set_table_meal_type");
				return "OK. What kind of meal will it be?";
			}
			
			// query food consumption preferences
			if(match(String.format("what does (%s) like to have for (%s)", regexPersonName, regexMealType), q)) {
				addEvidence("takesPartIn", "P1", "M");
				String name = capitalize(matcher.group(1));
				addEvidence("name", "P1", name);
				addEvidence("mealT", "M", capitalize(matcher.group(2)));
				Vector<InferenceResult> res = query("consumesAnyIn");
				String[] phrases = new String[]{"Almost always", "Quite often", "Sometimes"};
				Double[] probs = new Double[]{0.85, 0.5, 0.00001};
				String text = "";
				for(int i = 0; i < phrases.length; i++) {
					Vector<String> v = new Vector<String>();
					for(InferenceResult r : res) {
						double upper = i == 0 ? 1.0 : probs[i-1];
						if(r.probability >= probs[i] && r.probability < upper) {
							v.add(r.params[1]);
						}						
					}
					if(!v.isEmpty()) {
						if(text != "") text += " ";
						text += phrases[i] + ", " + name + " likes to have " + StringTool.join(", ", v) + ".";						
					}
				}	
				return text;
			}
			
			// initiate hypothetical reasoning
			if(match("suppose there was a meal", q)) {
				setState("hypo_meal");
				participants = 0;
				return "OK. How many participants were there?";
			}
		}
		
		if(haveState("hypo_meal")) {
			if(participants == 0) {
				if(match(regexNumber, q)) {
					participants = parseNumber(matcher.group());
					if(participants > 0) {
						for(int i = 1; i <= participants; i++)
							addEvidence("takesPartIn", "P" + i, "M");
						return "OK. What else do we know about the meal?";
					}
					return "I don't understand. How many participants were there?";
				}
				return "I don't understand. How many participants were there?"; 
			}
			else {
				System.out.println("participants = " + participants);
				if(match("participant (" + regexNumber + ") was (" + regexPersonName + ")", q)) {
					int no = parseNumber(matcher.group(1));
					String name = matcher.group(2);										
					this.name2participant.put(name, no);
					name = capitalize(name);
					addEvidence("name", "P" + no, name);
					this.participant2name.put(no, name);
					return "OK. Anything else?";
				}
				else if(match("(" + regexGeneralParticipant + ") used (?:an? )?(" + regexUtensil + ")", q)) {
					Integer no = getGeneralParticipant(matcher.group(1));
					if(no == null)
						return "I wasn't told that " + matcher.group(1) + " is one of the participants.";
					String utensil = utensils.get(matcher.group(2));
					addEvidence("usesAnyIn", "P" + no, utensil, "M");
					return "OK. Anything else?";
				}
				else if(match("(" + regexGeneralParticipant + ") consumed (?:an? )?(" + regexGood + ")", q)) {
					Integer no = getGeneralParticipant(matcher.group(1));
					if(no == null)
						return "I wasn't told that " + matcher.group(1) + " is one of the participants.";
					String good = goods.get(matcher.group(2));
					addEvidence("consumesAnyIn", "P" + no, good, "M");
					return "OK. Anything else?";
				}
				else if(match("the type of the meal was (" + regexMealType + ")", q)) {
					String type = capitalize(matcher.group(1));
					addEvidence("mealT", "M", type);
					return "OK. Anything else?";
				}
				else if(match("what type of meal was it", q)) {
					Vector<InferenceResult> res = query("mealT");
					return this.extractResults(res, 1);
				}
				else if(match("what time was it", q)) {
					Vector<InferenceResult> res = query("timeT");
					return this.extractResults(res, 1);
				}
				else if(match("who was participant (" + regexNumber + ")", q)) {
					int no = parseNumber(matcher.group(1));
					Vector<InferenceResult> res = query("name(P" + no + ")");
					return this.extractResults(res, 1);
				}
				else if(match(String.format("what did (%s) consume", regexGeneralParticipant), q)) {
					Integer no = getGeneralParticipant(matcher.group(1));
					if(no == null)
						return "I wasn't told that " + matcher.group(1) + " is one of the participants.";
					Vector<InferenceResult> res = query("consumesAnyIn(P" + no + ",x,M)");
					return this.extractResults(res, 1);
				}
				else if(match("what did (" + regexGeneralParticipant + ") use", q)) {
					Integer no = getGeneralParticipant(matcher.group(1));
					if(no == null)
						return "I wasn't told that " + matcher.group(1) + " is one of the participants.";
					Vector<InferenceResult> res = query("usesAnyIn(P" + no + ",x,M)");
					return this.extractResults(res, 1);
				}
				else
					return "I don't understand. What else do we know about the meal?";
			}
		}
		
		if(haveState("set_table_meal_type")) {
			if(match(regexMealType, q)) {
				regexMealType = capitalize(matcher.group());
				setState("set_table_num_participants");
				return "OK. How many people will participate?";
			}
		}
		
		if(haveState("set_table_num_participants")) {
			if(match("one|two|three|four", q)) {
				participants = parseNumber(matcher.group());
				setState("set_table_family_members");
				return "OK. Anyone family members I know?";
			}
			else {
				return "I don't understand. How many people will participate?";
			}
		}
		
		if(haveState("set_table_family_members")) {
			people = null;			
			String connect = "(?: and |,\\s?)";
			if(match(String.format("(%s(?:%s%s)*)", regexPersonName, connect, regexPersonName), q)) {
				people = matcher.group(1).split(connect);
				for(int i = 0; i < people.length; i++)
					people[i] = capitalize(people[i]);
			}
			else if(match("no|none", q)) {
				people = null;
			}
			else {
				return "I don't understand. Please name the family members or say 'none' if none will participate?";
			}
			
			setTopLevelState();			
			
			return querySetTable();
		}
		
		System.out.println("No match in SetTable");
		
		return null;
	}	
	
	protected Integer getGeneralParticipant(String s) {
		if(s.startsWith("participant")) {
			int no = parseNumber(s.substring("participant ".length()));
			if(no == 0) {				
				return null;
			}
			return no;
		}
		Integer no = name2participant.get(s);
		if(no == null)
			System.out.println("tried to retrieve '" + s + "' from " + StringTool.join(",", name2participant.keySet()));
		return no;
	}
	
	protected String extractResults(Vector<InferenceResult> vres, int argNo) {
		StringBuffer ret = new StringBuffer();
		int i = 0; 
		for(InferenceResult res : vres) {
			if(i++ > 0)
				ret.append(", ");			
			ret.append(res.params[argNo] + " " + String.format("%.2f", res.probability));
		}
		return ret.toString();
	}
	
	protected static int parseNumber(String no) {
		if(no.equals("one"))
			return 1;
		if(no.equals("two"))
			return 2;
		if(no.equals("three"))
			return 3;
		if(no.equals("four"))
			return 4;
		return 0;
	}
	
	protected String querySetTable() {
		//participants = 2;
		//people = new String[]{"Anna"};
		
		// build evidence
		for(int i = 1; i <= this.participants; i++) {
			String person = "P" + i;
			addEvidence("takesPartIn", person, "M");
			if(people != null && i <= people.length)
				addEvidence("name", person, people[i-1]);
		}
		addEvidence("mealT", "M", mealType);
		
		// infer
		Vector<InferenceResult> res;
		try {
			res = query("usesAnyIn");
		} 
		catch (Exception e) {
			e.printStackTrace();
			return null;
		} 
		
		String text = "I will provide ";
		int cnt = 0;
		double threshold = 0.4;
		for(InferenceResult r : res) {
			r.print(System.out);
			if(r.probability > threshold) {
				String name;
				int i = Integer.parseInt(r.params[0].substring(1));
				if(people != null && i <= people.length)
					name = people[i-1];
				else
					name = "Participant " + i;
				String object = r.params[1];
				if(cnt++ > 0)
					text += ", ";
				text += String.format("a %s for %s", object, name);
			}
		}
		
		return text;
	}
	
	protected void addEvidence(String... items) {
		evidence.add(items);
	}
	
	protected Vector<InferenceResult> query(String... queries) throws IOException, ParseException, Exception {		System.out.println("evidence: " + evidence);
		Model m = this.model;
		m.setEvidence(evidence); 
		m.instantiate(); 
		Vector<InferenceResult> res = m.infer(Arrays.asList(queries));
		
		return res;
	}
}
