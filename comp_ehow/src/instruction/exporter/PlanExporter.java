package instruction.exporter;

import instruction.opencyc.OpenCyc;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import org.opencyc.api.CycAccess;
import org.opencyc.api.CycApiException;
import org.opencyc.api.CycObjectFactory;
import org.opencyc.cycobject.CycFort;
import org.opencyc.cycobject.CycList;
import org.opencyc.cycobject.CycObject;
import org.opencyc.cycobject.CycVariable;

public class PlanExporter {
	public static final String MT_UNIVERSAL_VOCABULARY = "#$UniversalVocabularyMt";
	public static final String MT_TUM_KITCHEN_ENVIRONMENT = "#$TUMKitchenEnvironmentMt";
	public static final String MT_TUM_KITCHEN_ENVIRONMENT_PLANNING = "#$TUMKitchenEnvironmentPlanningMt";

	private static String[] spatialRelations = { "TopSide", "RightSide",
			"BottomSide", "CornerOnObject", "CenterOfGeographicalRegion",
			"FrontSide", "Side" };
	/**
	 * List of all action instances of the plan in the correct order, e.g.
	 * "action1", "action2", ...
	 */
	private ArrayList<String> plan;

	private HashMap<String, String> rplName; // mapping from Cyc concepts to RPL names

	/**
	 * Most specific superclass of the action instances in Cyc, e.g.
	 * "puttingsomethingsomewhere1" -> "PuttingSomethingSomewhere"
	 */
	private HashMap<String, String> actionTypes;

	/**
	 * Maps an action instance to a list of assertions associated to that
	 * instance, e.g. "puttingsomethingsomewhere1" -> (isa
	 * puttingsomethingsomewhere1 PuttingSomethingSomewhere), (objectActedOn
	 * puttingsthswh ...)
	 */
	private HashMap<String, CycList> actionParameters;

	/**
	 * Maps an object instance name to a list of all associated assertions, e.g.
	 * "table-pieceoffurniture1" -> (isa table-pieceoffurniture1
	 * Table-PieceOfFurniture)
	 */
	private HashMap<String, CycList> objects;

	/**
	 * Maps each location ID generated to a set of associated assertions, e.g.
	 * "location0" -> "(purposeOf-Generic action1 (...))"
	 */
	private HashMap<String, String> locations;

	private HashMap<String, String> objLocations; // object goal locations

	/**
	 * A list containing all location designators of this plan
	 */
	private List<String> locationDesignators = new ArrayList<String>();

	/**
	 * A list containing all object designators of this plan
	 */
	private List<String> objectDesignators = new ArrayList<String>();

	/**
	 * Maps an action instance to its goal location, e.g.
	 * "puttingsomethingsomewhere1" -> "location0"
	 */
	private Map<String, String> actionLocations = null;

	private static CycAccess cyc = null;
	private static PlanExporter me = null;

	private CycObject kitchenEnvMt = null;

	String designators = new String();
	String goals = new String();

	int locCounter = 0;

	private PlanExporter() throws UnknownHostException, CycApiException,
			IOException {

		cyc = OpenCyc.getInstance().getCycAcces();
		kitchenEnvMt = OpenCyc.getInstance().getKitchenEnvMt();

	}

	public static PlanExporter getInstance() throws UnknownHostException,
			CycApiException, IOException {

		if (me == null)
			me = new PlanExporter();

		return me;

	}

	public void readRplNames() {
		this.rplName = new HashMap<String, String>();

		// TODO: read these values from the KB
		rplName.put("PuttingSomethingSomewhere", "object-at-place");

		// /////////////////////////////////////////////////////////////////////
		// object classes

		rplName.put("StoveTopCookingPot", "cooking-pot");
		rplName.put("Water", "water");
		rplName.put("CookingRange", "cooker");
		rplName.put("MeasuringCup", "");
		rplName.put("Tap-FlowControlDevice", "tap");
		rplName.put("Burner", "hot-plate");
		rplName.put("Soup", "");
		rplName.put("Packet", "");
		rplName.put("StoveTop", "hot-plate");
		rplName.put("HeatingDevice", "hot-plate");
		rplName.put("Content", "");
		rplName.put("Noodle", "pasta");
		rplName.put("Pasta", "pasta");
		rplName.put("TableSalt", "salt");
		rplName.put("Colander", "colander");
		rplName.put("Chair-PieceOfFurniture", "chair");
		rplName.put("Sink", "sink");
		rplName.put("OliveOil", "");
		rplName.put("Tea-Iced", "");
		rplName.put("BakingSoda", "");
		rplName.put("CookingVessel", "pan");
		rplName.put("MixForBakedGoods", "pancake-mix");
		rplName.put("Pancake", "pancake");
		rplName.put("TeaBag", "");
		rplName.put("Tea-Beverage", "");
		rplName.put("ContainerArtifact", "");
		rplName.put("Table-PieceOfFurniture", "table");
		rplName.put("Refrigerator", "refrigerator");
		rplName.put("Kettle", "cooking-pot");
		rplName.put("HotBeveragePot", "cup");
		rplName.put("ContainerLid", "");
		rplName.put("Bottle", "");
		rplName.put("Milk", "");
		rplName.put("Sugar-Table", "");
		rplName.put("SugarBowl", "");
		rplName.put("Teaspoon", "table-spoon");
		rplName.put("DrinkingMug", "cup");
		rplName.put("Saucer", "plate");
		rplName.put("DinnerPlate", "plate");
		rplName.put("Oven", "oven");
		rplName.put("Butter", "");
		rplName.put("Toast", "toast");
		rplName.put("Bread", "toast");
		rplName.put("Bread-Slice", "toast");
		rplName.put("ElectricalToaster", "toaster");
		rplName.put("BreakfastCereal", "");
		rplName.put("Bowl-Eating", "");
		rplName.put("Spoon-SilverwarePiece", "table-spoon");
		rplName.put("Fruit", "");
		rplName.put("MapleSyrup", "");
		rplName.put("PlaceMat", "place-mat");
		rplName.put("Napkin", "napkin");
		rplName.put("Fork-SilverwarePiece", "fork");
		rplName.put("Knife", "knife");

		// /////////////////////////////////////////////////////////////////////
		// action types

		// finding something
		rplName.put("SelectingSomething", "find-object");
		rplName.put("LookingForSomething", "find-object");
		rplName.put("Adopting-SelectingSomething", "find-object");

		// transporting something
		rplName.put("PuttingSomethingSomewhere", "loc");
		rplName.put("TransportationEvent", "loc");
		rplName.put("LiftingAnObject", "object-lifted");
		rplName.put("GettingSomething-HittingAnObject", "object-in-hand");

		rplName.put("TakingSomething", "object-in-hand");
		rplName.put("RemovingSomething", "object-in-hand");

		rplName.put("CoveringSomething", "object-on-object");

		rplName.put("DoingAddition", "object-added-to-container");
		rplName.put("PuttingSomethingIntoSomething", "object-added-to-container");
		rplName.put("Incorporation-Physical", "object-added-to-container");

		// device-related stuff
		rplName.put("TurningOnPoweredDevice", "hot-plate-temperature-on");
		rplName.put("TurningOffPoweredDevice", "hot-plate-temperature-off");
		rplName.put("ChangingDeviceState", "hot-plate-temperature-change");

		// cupboard-related stuff
		rplName.put("OpeningAContainer", "enclosing-object-opened");
		rplName.put("ClosingAContainer", "object-closed");

		// filling and transfilling containers
		rplName.put("FillingProcess", "container-filled");
		rplName.put("FluidFlow-Translation", "container-filled");
		rplName.put("FluidFlow-Translation", "container-content-transfilled");

		// stirring either a container or some stuff inside a container
		rplName.put("Stirring", "container-content-stirred");
		rplName.put("Flipping", "object-flipped");

		// wait-like statements
		rplName.put("Checking-Evaluating", "sleep");
		rplName.put("Observing", "sleep");
		rplName.put("Waiting", "sleep");
		rplName.put("KeepingInACertainState", "sleep");

		rplName.put("CookingFood", "cook");

		// achieve-like statements
		rplName.put("CausingToBeInACertainCondition", "achieve");
		rplName.put("Remembering", "achieve");

		// general
		rplName.put("UsingAnObject", "use");

		// future work:
		// rplName.put("ServingFoodOrDrink", "");
		// rplName.put("IntrinsicStateChangeEvent", ""); // bring the water to a
		// boil
		// rplName.put("EatingAMorsel", "");
		// rplName.put("SpreadingOntoSurface", "");
		// rplName.put("SlicingSomething", "");
		// rplName.put("Cleaning", "");
		// rplName.put("MoppingAFloor", "water-wiped-up");
		// rplName.put("BeingSomething", "");
		// rplName.put("Garnishment", "");

		// Plan categories:
		// CookingFood
		// BoilingFood
		// PreparingFoodOrDrink
		// ArrangingObjects
		// 

		// /////////////////////////////////////////////////////////////////////
		// instances
		// rplName.put("placemat1", "placemat-1");
		// rplName.put("napkin1", "napkin-1");
		// rplName.put("dinnerplate1", "plate-1");
		// rplName.put("fork-silverwarepiece1", "fork-1");
		// rplName.put("knife1", "knife-1");
		// rplName.put("spoon-silverwarepiece1", "spoon-1");
		// rplName.put("drinkingmug1", "cup-1");

	}

	@SuppressWarnings({ "deprecation" })
	public void readActionsForPlanFromKB(String plan)
			throws UnknownHostException, IOException {

		this.plan = new ArrayList<String>();

		CycList query = cyc.makeCycList("(#$methodForAction " + plan
				+ "(#$actionSequence ?LIST))");
		CycVariable var = CycObjectFactory.makeCycVariable("?LIST");
		CycFort mt = cyc.getConstantByName("#$TUMKitchenEnvironmentPlanningMt");

		CycList ret = cyc.askWithVariable(query, var, mt);

		ArrayList<String> list = new ArrayList<String>();
		for (Iterator i = ret.iterator(); i.hasNext();) {
			String str = i.next().toString();
			list.add(str);
		}
		String[] actions = list.get(0).substring(9, list.get(0).length() - 1)
				.split(" ");
		for (String act : actions) {
			this.plan.add(act);
		}
	}

	public void readActionTypesFromKB() throws UnknownHostException,
			IOException {

		// get isa-type of the action in TUMKitchenEnvironmentMt
		actionTypes = new HashMap<String, String>();
		for (String action : this.plan) {
			actionTypes.put(action, cyc.getMinIsas(
					cyc.getConstantByName(action), kitchenEnvMt).toString());
		}

	}

	public void readActionParametersFromKB(ArrayList<String> actionList)
			throws UnknownHostException, IOException {

		actionParameters = new HashMap<String, CycList>();

		// query list of parameters per action
		for (String action : actionList) {
			actionParameters.put(action, arg1GAFsAbout(action,
					MT_TUM_KITCHEN_ENVIRONMENT));
		}
	}

	public void readObjectsFromKB(HashMap<String, CycList> paramList)
			throws UnknownHostException, IOException {
		objects = new HashMap<String, CycList>();

		// iterate over whole list of action parameters and extract objects
		for (String key : paramList.keySet()) {

			CycList curParamList = paramList.get(key);

			for (Iterator i = curParamList.iterator(); i.hasNext();) {
				String curParam = i.next().toString();
				if ((curParam.contains("objectActedOn"))
						|| (curParam.contains("objectMoving"))
						|| (curParam.contains("objectOfStateChange"))) {

					String objectName = curParam.substring(1,
							curParam.length() - 1).split(" ")[1];
					objects.put(objectName.trim(), arg1GAFsAbout(objectName,
							MT_TUM_KITCHEN_ENVIRONMENT));
				}
			}

		}

	}

	public void readLocationsFromKB(HashMap<String, CycList> paramList)
			throws UnknownHostException, IOException {
		locations = new HashMap<String, String>();
		actionLocations = new HashMap<String, String>();

		locCounter = 0;

		// iterate over whole list of action parameters and extract locations
		for (String action : paramList.keySet()) {

			CycList curParamList = paramList.get(action);

			for (Iterator i = curParamList.iterator(); i.hasNext();) {
				String curParam = i.next().toString();
				if (curParam.contains("purposeOf-Generic")) {

					String locName = "location" + locCounter++;
					locations.put(locName, curParam);
					actionLocations.put(action, locName);
				}
			}
		}
	}

	@SuppressWarnings({ "deprecation" })
	public CycList arg1GAFsAbout(String concept, String microtheory)
			throws UnknownHostException, IOException {

		CycObject mt = cyc.getConstantByName(microtheory);
		CycList query = cyc.makeCycList("(?PRED #$" + concept + " ?ARG)");

		CycVariable pred = CycObjectFactory.makeCycVariable("?PRED");
		CycVariable arg = CycObjectFactory.makeCycVariable("?ARG");
		ArrayList<CycVariable> vars = new ArrayList<CycVariable>();
		vars.add(pred);
		vars.add(arg);

		CycList l = cyc.askWithVariables(query, vars, mt);
		return l;
	}

	@SuppressWarnings({ "deprecation" })
	public CycList arg2GAFsAbout(String concept, String microtheory)
			throws UnknownHostException, IOException {

		CycObject mt = cyc.getConstantByName(microtheory);
		CycList query = cyc.makeCycList("(?PRED ?ARG #$" + concept + ")");

		CycVariable pred = CycObjectFactory.makeCycVariable("?PRED");
		CycVariable arg = CycObjectFactory.makeCycVariable("?ARG");
		ArrayList<CycVariable> vars = new ArrayList<CycVariable>();
		vars.add(pred);
		vars.add(arg);

		return cyc.askWithVariables(query, vars, mt);
	}

	public String exportPlanToRPL(String planName) {
		String res = "";
		
		if(actionLocations!=null)
			actionLocations.clear();
		
		if(actionParameters!=null)
			actionParameters.clear();
		if(actionTypes!=null)
			actionTypes.clear();
		if(locationDesignators!=null)
			locationDesignators.clear();
		if(locations!=null)
			locations.clear();
		if(objectDesignators!=null)
			objectDesignators.clear();
		if(objects!=null)		
			objects.clear();
		if(objLocations!=null)
			objLocations.clear();
		
		goals="";
		designators="";
		
		try {

			// init Cyc-Lisp name mapping
			readRplNames();

			// get the list of actions for the plan
			readActionsForPlanFromKB(planName);

			// check for parameters for each action
			readActionParametersFromKB(this.plan);

			// determine action types
			readActionTypesFromKB();

			// create lists of objects and locations
			readObjectsFromKB(this.actionParameters);
			readLocationsFromKB(this.actionParameters);

			// build object designators
			readObjectDesignators(this.objects);

			// build location designators
			readLocationDesignators(this.locations);

			// compose object designators
			for (String objDesig : objectDesignators)
				designators += "         " + objDesig + "\n";

			// compose location designators
			for (String locDesig : locationDesignators)
				designators += "         " + locDesig + "\n";

			// build the achieve statements

			for (String act : plan) {

				String actType = actionTypes.get(act);
				actType = actType.substring(1, actType.length() - 1);
				String rplAction = rplName.get(actType);
				if (rplAction == null) {
					continue;
				}
				if (rplAction.equals("find-entity")) {
					goals += readFindObject(act);
				}

				else if (rplAction.equals("loc")) {
					goals += readObjectAtPlace(act);
				}

				else if (rplAction.equals("object-in-hand")) {
					goals += readObjectPickedUp(act);
				}

				else if (rplAction.equals("object-on-object")) {
					goals += readObjectOnObject(act);
				}

				else if (rplAction.equals("object-added-to-container")) {
					goals += readObjectAddedToContainer(act);
				}

				else if (rplAction.matches("hot-plate-temperature")) {
					goals += readHotPlateTemperature(act);
				}

				else if (rplAction.equals("enclosing-object-opened")) {
					goals += readEnclosingObjectOpened(act);
				}

				else if (rplAction.equals("object-flipped")) {
					goals += readObjectFlipped(act);
				}

				else if (rplAction.equals("object-closed")) {
					goals += readObjectClosed(act);
				}

				else if (rplAction.equals("container-filled")) {
					goals += readContainerFilled(act);
				}

				else if (rplAction.equals("container-content-transfilled")) {
					goals += readContainerContentTransfilled(act);
				}

				else if (rplAction.equals("container-content-stirred")) {
					goals += readContainerContentStirred(act);
				}

				else if (rplAction.equals("sleep")) {
					goals += readWait(act);
				}

				else if (rplAction.equals("cook")) {
					goals += readCook(act);
				}

				else if (rplAction.equals("achieve")) {
					goals += readAchieve(act);
				}

				else if (rplAction.equals("use")) {
					goals += readUse(act);
				}

				else {
					goals += "";
				}
			}

			// combine designators and achieve statements to a plan
			res += "(def-top-level-plan ehow-"
					+ planName.substring(3, planName.length() - 1).split(" ")[0]
							.replaceAll("_", "-") + " ()"
					+ "\n" + "  (with-designators (\n";

			res += designators;
			res += ")\n";

			res += goals;

			res += ")))";

		//	System.out.println(res);

		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (CycApiException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return res;
	}

	private void readObjectDesignators(HashMap<String, CycList> objects) {

		// create the appropriate object designators
		for (String obj : objects.keySet()) {

			String object = "";

			object += "(" + obj + " (object ";

			// get the list of {parameter - value} mappings
			HashMap<String, String> objparams = cycParamListToStringMap(objects
					.get(obj));
			for (String par : objparams.keySet()) {
				String val = objparams.get(par);

				// TODO: check this list and verify that the mappings are
				// correct
				if (par.equals("isa")) {
					object += "'((type " + rplName.get(val)
							+ "))";
				} else if (par.equals("name")) {
					object += " (name " + rplName.get(val) + ")";
				} else if (par.equals("matches")) {
					object += " (matches " + rplName.get(val)
							+ ")";
				} else if (par.equals("matches-internal")) {
					object += " (matches-internal "
							+ rplName.get(val) + ")";
				} else if (par.equals("referenced-by")) {
					object += " (referenced-by "
							+ rplName.get(val) + ")";
				} else if (par.equals("properPhysicalParts")) {
					object += " (part-of " + rplName.get(val)
							+ ")";
				} else if (par.equals("containsPart")) {
					object += "\n              (parent-of " + rplName.get(val)
							+ ")";
				} else if (par.equals("regulating-temperature")) {
					object += "\n              (regulating-temperature "
							+ rplName.get(val) + ")";
				} else if (par.equals("regulating-water-flow")) {
					object += "\n              (regulating-water-flow "
							+ rplName.get(val) + ")";
				} else if (par.equals("fitting")) {
					object += "\n              (fitting " + rplName.get(val)
							+ ")";
				} else if (par.equals("status")) {
					object += "\n              (status " + rplName.get(val)
							+ ")";
				} else if (par.equals("exclude")) {
					object += "\n              (exclude " + rplName.get(val)
							+ ")";
				}
			}
			object += "))";

			objectDesignators.add(object);
		}
	}

	private void readLocationDesignators(HashMap<String, String> locations)
			throws UnknownHostException, IOException {

		this.objLocations = new HashMap<String, String>();

		for (String action : plan) {

			String loc = actionLocations.get(action);

			if (loc == null)
				continue;

			// -> first param is the object, second param is the loc description
			String locExpr = locations.get(loc);

			String preposition = locExpr.substring(20, locExpr.length() - 2)
					.split(" ")[0];
			String forObject = locExpr.substring(20, locExpr.length() - 2)
					.split(" ")[1];
			String locDesc = locExpr.substring(20, locExpr.length() - 2).split(
					" ")[2];
			String locationDesignator = "";

			// recursively build the location designator
			locationDesignator += "(" + loc + " ";

			if (preposition.equals("nextToLikeObjects")) {
				locationDesignator += "(location `((next-to ,"
						+ constructLocDesignatorFromCycString(locDesc, action)
						+ ") (for ," + forObject + ")))";

				this.objLocations.put(forObject, loc);

			} else if ((preposition.equals("in-UnderspecifiedContainer"))||(preposition.equals("into-UnderspecifiedContainer"))) {

				locationDesignator += "(location `((in ,"
						+ constructLocDesignatorFromCycString(locDesc, action)
						+ ") (of ," + forObject + ")))";
				this.objLocations.put(forObject, loc);

			} else if (preposition.equals("from-UnderspecifiedLocation")) {

				locationDesignator += "(location `((in ,"
						+ constructLocDesignatorFromCycString(locDesc, action)
						+ ") (of ," + forObject + ")))";
				this.objLocations.put(forObject, loc);

			} else if (preposition.equals("to-UnderspecifiedLocation")) {
				System.out.println(locDesc);
				locationDesignator += "(location `((in ,"
						+ constructLocDesignatorFromCycString(locDesc, action)
						+ ") (for ," + forObject + ")))";
				this.objLocations.put(forObject, loc);

			} else if (preposition.equals("on-UnderspecifiedSurface")) {

				locationDesignator += "(location `((on ,"
						+ constructLocDesignatorFromCycString(locDesc, action)
						+ ") (for ," + forObject + ")))";
				this.objLocations.put(forObject, loc);

			} else {
				locationDesignator += "(location `((on ,"
						+ constructLocDesignatorFromCycString(locDesc, action)
						+ ") (for ," + forObject + ")))";
				this.objLocations.put(forObject, loc);
			}
			locationDesignator += ")";

			locationDesignators.add(locationDesignator);

		}
	}

	private String constructLocDesignatorFromCycString(String locDesc,
			String action) throws UnknownHostException, IOException {

		// if(this.objects.containsKey(locDesc)) {
		// // location relative to another object -> break the loop
		// return locDesc;
		// } else {
		// GAFs describing the location as part of something else have the
		// location as the second argument (e.g. physicalParts)
		CycList tmpList = arg2GAFsAbout(locDesc, MT_TUM_KITCHEN_ENVIRONMENT);
		HashMap<String, String> locparams = cycParamListToStringMap(tmpList);

		if (locparams.isEmpty()) {
			// has no parts-underspecified: infer that one of the previous
			// instruction
			String locDescType = cyc.getMinIsas(cyc.getConstantByName(locDesc),
					kitchenEnvMt).toString();
			locDescType = locDescType.substring(1, locDescType.length() - 1);

			if (containsString(spatialRelations, locDescType)) {
				String prevLocation = getObjectLocationOfPreviousAction(action);
				String locExpr = locations.get(prevLocation);

//				String preposition = locExpr
//						.substring(20, locExpr.length() - 2).split(" ")[0];
//				String forObject = locExpr.substring(20, locExpr.length() - 2)
//						.split(" ")[1];
				String locDesc2 = locExpr.substring(20, locExpr.length() - 2)
						.split(" ")[2];
				return constructLocDesignatorFromCycString(locDesc2, getPreviousAction(action));
			}
			
			// if the object is not contained in the object designators, add it
			if(!getAllObjectIds().contains(locDesc)) {
				
				String objType = cyc.getMinIsas(cyc.getConstantByName(locDesc), kitchenEnvMt).toString();
				objType = objType.replace((CharSequence) "(", ""); 
				objType = objType.replace((CharSequence) ")", "");
				objType = objType.split(" ")[0];
		
				objectDesignators.add("(" + locDesc + " (object '((type " + rplName.get(objType) + "))))");
			}

			return locDesc;
		} else {
			String locDescType = cyc.getMinIsas(cyc.getConstantByName(locDesc),
					kitchenEnvMt).toString();
			locDescType = locDescType.substring(1, locDescType.length() - 1);
			String[] types = locDescType.split(" ");

			// CornerOnObject has to be processed first because it's possible
			// that the entity is also an instance of LeftSide, RightSide,
			// TopSide or FrontSide (e.g. "top left corner")

			if (containsString(types, "CornerOnObject")) {

				String location = "on-";

				if (containsString(types, "TopSide"))
					location += "back";
				else if (containsString(types, "BottomSide"))
					location += "front";
				if (containsString(types, "LeftSide"))
					location += "-left";
				else if (containsString(types, "RightSide"))
					location += "-right";

				String locDesignator = constructLocDesignatorFromCycString(
						locparams.get("parts-Underspecified"), action);

				if (locDesignator.split(" ").length > 1) {
					// Nested location: build a new location on top level
					String locId = "location" + this.locCounter++;
					locationDesignators.add("(" + locId + " `(a location "
							+ locDesignator + "))");
					return "(" + location + " ," + locId + ")";
				} else
					return "(" + location + " ," + locDesignator + ")";

			} else if (types[0].equals("LeftSide")) {
				String locDesignator = constructLocDesignatorFromCycString(
						locparams.get("parts-Underspecified"), action);
				if (locDesignator.split(" ").length > 1) {
					// Nested location: build a new location on top level
					String locId = "location" + this.locCounter++;
					locationDesignators.add("(" + locId + " `(a location "
							+ locDesignator + "))");
					return "(left-of ," + locId + ")";
				} else
					return "(left-of ," + locDesignator + ")";

			} else if (types[0].equals("RightSide")) {
				String locDesignator = constructLocDesignatorFromCycString(
						locparams.get("parts-Underspecified"), action);
				if (locDesignator.split(" ").length > 1) {
					// Nested location: build a new location on top level
					String locId = "location" + this.locCounter++;
					locationDesignators.add("(" + locId + " `(a location "
							+ locDesignator + "))");
					return "(right-of ," + locId + ")";
				} else
					return "(right-of ," + locDesignator + ")";

			} else if (types[0].equals("CenterOfGeographicalRegion")) {
				String locDesignator = constructLocDesignatorFromCycString(
						locparams.get("parts-Underspecified"), action);
				if (locDesignator.split(" ").length > 1) {
					// Nested location: build a new location on top level
					String locId = "location" + this.locCounter++;
					locationDesignators.add("(" + locId + " `(a location "
							+ locDesignator + "))");
					return "(on-center ," + locId + ")";
				} else
					return "(on-center ," + locDesignator + ")";

			} else if (types[0].equals("Side")) {
				String locDesignator = constructLocDesignatorFromCycString(
						locparams.get("parts-Underspecified"), action);
				if (locDesignator.split(" ").length > 1) {
					// Nested location: build a new location on top level
					String locId = "location" + this.locCounter++;
					locationDesignators.add("(" + locId + " `(a location "
							+ locDesignator + "))");
					return "(next-to ," + locId + ")";
				} else
					return "(next-to ," + locDesignator + ")";

			} else if (types[0].equals("FrontSide")) {
				String locDesignator = constructLocDesignatorFromCycString(
						locparams.get("parts-Underspecified"), action);
				if (locDesignator.split(" ").length > 1) {
					// Nested location: build a new location on top level
					String locId = "location" + this.locCounter++;
					locationDesignators.add("(" + locId + " `(a location "
							+ locDesignator + "))");
					return "(on-front ," + locId + ")";
				} else
					return "(on-front ," + locDesignator + ")";

			} else {
				return locDesc;
			}
		}
		// }
	}

	/**
	 * Recursively searches for all entities that are involved in the specified
	 * action or location. The entity IDs will be stored in the list.
	 * 
	 * @param action
	 * @param entities
	 */
	private void getInvolvedEntities(String id, List<String> entities) {

		if (actionParameters.get(id) != null) {
			// id is an action
			HashMap<String, String> params = cycParamListToStringMap(actionParameters
					.get(id));
			entities.add(params.get("objectActedOn"));

			getInvolvedEntities(objLocations.get(params.get("objectActedOn")),
					entities);

		} else if (getAllLocationIds().contains(id)) {
			// id is a location

			for (String locDesig : locationDesignators) {
				if (id.equals(locDesig.substring(1, locDesig.indexOf(" ")))) {
					for (String obj : getAllObjectIds()) {
						if (locDesig.contains(obj) && !entities.contains(obj))
							entities.add(obj);
					}
					for (String loc : getAllLocationIds()) {
						if (locDesig.indexOf(loc) > 1) {
							getInvolvedEntities(loc, entities);
						}
					}
				}
			}

		}

	}

	private List<String> getAllObjectIds() {
		List<String> ids = new ArrayList<String>();

		for (String objDesig : objectDesignators) {
			String objId = objDesig.substring(1, objDesig.indexOf(" "));
			ids.add(objId);
		}
		return ids;
	}

	private List<String> getAllLocationIds() {
		List<String> ids = new ArrayList<String>();

		for (String locDesig : locationDesignators) {
			String locId = locDesig.substring(1, locDesig.indexOf(" "));
			ids.add(locId);
		}
		return ids;
	}

	private String makePerceiveForEntities(List<String> entities) {
		String str = "";

		for (String e : entities)
			str += "(perceive " + e + ")\n";

		return str;
	}

	private String getPreviousAction(String action) {
		for (int i = 0; i < plan.size(); i++) {

			if (i > 0 && action.equals(plan.get(i)))
				return plan.get(i - 1);
		}
		return null;
	}

	private String getObjectLocationOfPreviousAction(String action) {
		for (int i = 0; i < plan.size(); i++) {

			String loc = actionLocations.get(plan.get(i));

			if (loc != null && action.equals(plan.get(i)) && i > 0)
				return actionLocations.get(plan.get(i - 1));
		}
		return null;
	}

	@SuppressWarnings("unused")
	private String getObjectLocationForAction(String action) {
		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));
		return objLocations.get(params.get("objectActedOn").trim());
	}

	// read parameters for the object-at-place action and create the achieve
	// statement
	private String readObjectAtPlace(String action) {

		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));

		List<String> entities = new ArrayList<String>();
		getInvolvedEntities(action, entities);

		// return the achieve statement
		return "(perceive " + params.get("objectActedOn") + ")\n"
				+ "(achieve `(loc ," + params.get("objectActedOn") + " ," 
				+ objLocations.get(params.get("objectActedOn").trim()) + "))\n";
	}

	private String readFindObject(String action) {

		// translate the parameters into a lisp representation
		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));

		// return the achieve statement
		return "(achieve (findEntity\n" + "      "
				+ params.get("objectActedOn") + "))\n";
	}

	private String readObjectPickedUp(String action) {

		// translate the parameters into a lisp representation
		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));
		
		List<String> entities = new ArrayList<String>();
		getInvolvedEntities(action, entities);
		
		return  "(perceive " + params.get("objectActedOn") + ")\n"
				+ "(achieve `(object-in-hand ," 
				+ params.get("objectActedOn") + "))\n";

	}

	private String readObjectOnObject(String action) {

		// translate the parameters into a lisp representation
		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));

		try {
			if (cyc.getMinIsas(
					cyc.getConstantByName(params.get("objectActedOn")),
					kitchenEnvMt).toString().matches("StoveTopCookingPot")) {

				// object is a pot, has to be covered with a lid which we add to
				// the object designators
				this.designators += "(lid1 `(an entity" + "     (type lid)))";
				return "(achieve (object-on-entity lid1 "
						+ params.get("objectActedOn") + "))\n";

			}
		} catch (Exception e) {
		}

		return "(achieve (object-on-entity))\n";
	}

	private String readObjectAddedToContainer(String action) { // TODO: infer
		// the container
		// the object
		// shall be
		// added to

		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));

		return "(achieve (object-added-to-container "
				+ params.get("objectActedOn") + "))\n";
	}

	private String readHotPlateTemperature(String action) {

		// translate the parameters into a lisp representation
		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));

		if (action.equals("hot-plate-temperature-on")) {

			return "(achieve (hot-plate-temperature\n" + "      "
					+ params.get("objectActedOn") + "\n" + " :on))\n";

		} else if (action.equals("hot-plate-temperature-off")) {

			return "(achieve (hot-plate-temperature\n" + "      "
					+ params.get("objectActedOn") + "\n" + " :off))\n";

		} else {
			// TODO: read the current state and change it
			return "(achieve (hot-plate-temperature\n" + "      "
					+ params.get("objectActedOn") + "\n" + " :off))\n";
		}

	}

	private String readEnclosingObjectOpened(String action) {

		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));
		return "(achieve (enclosing-object-opened\n" + "      "
				+ params.get("objectActedOn") + "))\n";

	}

	private String readObjectFlipped(String action) {

		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));
		
		List<String> entities = new ArrayList<String>();
		getInvolvedEntities(action, entities);
		return "(perceive " + params.get("objectActedOn") + ")\n" + 
				"(achieve `(object-flipped ,"
				+ params.get("objectActedOn") + "))\n";

	}
	
	private String readObjectClosed(String action) {

		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));
		return "(achieve (object-closed\n" + "      "
				+ params.get("objectActedOn") + "))\n";
	}

	private String readContainerFilled(String action) {

		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));
		// TODO
		// if(objectActedOn subClassOf Container)

		// get purposeOf -> withUnderSpecAgent for the filler
		String stuff = params.get("purposeOf-Generic");
		if (stuff != null && stuff.matches(".*with-Underspecified.*")) {
			stuff = stuff.substring(28, stuff.length() - 2).split(" ")[1];
		}

		return "(achieve (container-filled\n" + "      "
				+ params.get("objectActedOn") + "\n" + "      :" + stuff
				+ " 1.0l))\n"; // TODO: quantifier

		// TODO: check if the parameters better fit
		// container-content-transfilled
	}

	private String readContainerContentTransfilled(String action) { // TODO

		// translate the parameters into a lisp representation
		HashMap<String, String> params = cycParamListToStringMap(actionParameters.get(action));

		
		String stuff = params.get("purposeOf-Generic");
		String cont="";
		if (stuff != null && stuff.matches(".*into-UnderspecifiedContainer.*")) {
			cont  = stuff.substring(30, stuff.length() - 1).split(" ")[1];
			stuff = stuff.substring(30, stuff.length() - 1).split(" ")[0];
		}
		
		List<String> entities = new ArrayList<String>();
		getInvolvedEntities(action, entities);
		return "(perceive " + stuff + ")\n" + "(perceive " + cont + ")\n" +
			   "(achieve `(container-content-transfilled ,"+stuff+" ,"+cont+"))\n"; // TODO: check

	}

	private String readContainerContentStirred(String action) { // TODO

		// translate the parameters into a lisp representation
		// HashMap<String, String> params =
		// cycParamListToStringMap(actionParameters.get(action));

		// create object designator for wooden spoon
		// (:tag search-wooden-spoon
		// (setf wooden-spoon (perceive wooden-spoon))
		// (setf wooden-spoon-original-pose (copy-object-values [pose [reference
		// wooden-spoon]])))

		// (achieve (container-content-stirred cooking-pot wooden-spoon))
		// (achieve (loc
		// wooden-spoon
		// (=>designator `(a location (pose ,wooden-spoon-original-pose))) )))

		// find wooden spoon, stir, put spoon back
		// (achieve (container-content-stirred cooking-pot wooden-spoon)
		return "(achieve (container-content-stirred))\n";
	}

	private String readWait(String action) {

		// translate the parameters into a lisp representation
		HashMap<String, String> params = cycParamListToStringMap(actionParameters
				.get(action));

		String timespan = params.get("timeSpan");

		String[] unit_value = { "" };
		if (timespan != null)
			unit_value = timespan.substring(1, timespan.length() - 1)
					.split(" ");

		int time=0;
		if (unit_value.length == 2) {

			// determine the value
			if (unit_value[1].matches("[0-9.]*")) { // time directly given as
				// numeric value
				time = Integer.valueOf(unit_value[1]);
			} else if (unit_value[1].equals("#$Few-Quant")) {
				time = 2;
			}

			// compute value in seconds
			if (unit_value[0].equals("MinutesDuration")) {
				time *= 60;
			} else if (unit_value[0].equals("HoursDuration")) {
				time *= 3600;
			}

		}

		return "(sleep " + time + ")\n";
	}

	private String readCook(String action) { // TODO

		// translate the parameters into a lisp representation
		// HashMap<String, String> params =
		// cycParamListToStringMap(actionParameters.get(action));

		return "(achieve (cook))\n";
	}

	private String readAchieve(String action) { // TODO

		// translate the parameters into a lisp representation
		// HashMap<String, String> params =
		// cycParamListToStringMap(actionParameters.get(action));

		// build and return the achieve statement based on the parameters
		return "(achieve ())\n";
	}

	private String readUse(String action) { // TODO

		// translate the parameters into a lisp representation
		// HashMap<String, String> params =
		// cycParamListToStringMap(actionParameters.get(action));

		// retrieve from the knowledge base how to use the objectActedOn in the
		// given context

		return "(achieve (container-content-transfilled))\n";
	}

	// transform the Cyc list into a mapping {paramName -> paramValue}
	private HashMap<String, String> cycParamListToStringMap(CycList cycPar) {
		HashMap<String, String> res = new HashMap<String, String>();

		for (Iterator i = cycPar.iterator(); i.hasNext();) {
			String curParam = i.next().toString();

			String[] key_val = curParam.substring(1, curParam.length() - 1)
					.split(" ");

			String key = key_val[0];
			String val = "";
			for (int j = 1; j < key_val.length; j++) {
				if (j > 1)
					val += " ";
				val += key_val[j];
			}
			res.put(key, val);

		}

		return res;
	}

	private boolean containsString(String[] list, String str) {
		for (String s : list)
			if (s.equals(str))
				return true;
		return false;
	}

}
