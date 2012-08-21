package edu.tum.cs.ias.knowrob.util;


public interface PrologObjVisInterface {
	
	
	
	//
	// GENERAL CONVENTIONS
	//
	// - Object identifiers are of the form <url>#<local>
	// 	 e.g. http://ias.cs.tum.edu/kb/ias_semantic_map.owl#cupboard1
	//
	// - events, actions, objects are transferred as string identifiers, 
	//   the java applet or the resp. object class then reads the information
	//   it needs for visualisation (duration, positions, etc) by itself
	//
	// - information can be read with rdf_has(S, P, O), rdf_reachable(S, P, O) (exploits transitivity),
	//   rdf_triple(S, P, O) (also calls computables), see also
	//   http://www.swi-prolog.org/pldoc/package/semweb.html
	//
	// - pose sequences are of the form
	// 	   P-(EPISODENR)-(OCCURRENCENR)-(INSTANCENR)-(TIME)-(BECX)-(BECY)-(BECZ)-(ULWX)-(ULWY)-(ULWZ)-(OLWX)-(OLWY)-(OLWZ)-(UBWX)-(UBWY)-(UBWZ)-(OBWX)-(OBWY)-(OBWZ)-(UHWX)-(UHWY)-(UHWZ)-(BRKX)-(BRKY)-(BRKZ)-(OHWX)-(OHWY)-(OHWZ)-(KOX)-(KOY)-(KOZ)-(SEHX)-(SEHY)-(SEHZ)-(OSLX)-(OSLY)-(OSLZ)-(USLX)-(USLY)-(USLZ)-(FULX)-(FULY)-(FULZ)-(FBLX)-(FBLY)-(FBLZ)-(OSRX)-(OSRY)-(OSRZ)-(USRX)-(USRY)-(USRZ)-(FURX)-(FURY)-(FURZ)-(FBRX)-(FBRY)-(FBRZ)-(SBLX)-(SBLY)-(SBLZ)-(OALX)-(OALY)-(OALZ)-(UALX)-(UALY)-(UALZ)-(HALX)-(HALY)-(HALZ)-(FILX)-(FILY)-(FILZ)-(SBRX)-(SBRY)-(SBRZ)-(OARX)-(OARY)-(OARZ)-(UARX)-(UARY)-(UARZ)-(HARX)-(HARY)-(HARZ)-(FIRX)-(FIRY)-(FIRZ) 	 P-(EPISODENR)-(OCCURRENCENR)-(INSTANCENR)-(TIME)-(BECX)-(BECY)-(BECZ)-(ULWX)-(ULWY)-(ULWZ)-(OLWX)-(OLWY)-(OLWZ)-(UBWX)-(UBWY)-(UBWZ)-(OBWX)-(OBWY)-(OBWZ)-(UHWX)-(UHWY)-(UHWZ)-(BRKX)-(BRKY)-(BRKZ)-(OHWX)-(OHWY)-(OHWZ)-(KOX)-(KOY)-(KOZ)-(SEHX)-(SEHY)-(SEHZ)-(OSLX)-(OSLY)-(OSLZ)-(USLX)-(USLY)-(USLZ)-(FULX)-(FULY)-(FULZ)-(FBLX)-(FBLY)-(FBLZ)-(OSRX)-(OSRY)-(OSRZ)-(USRX)-(USRY)-(USRZ)-(FURX)-(FURY)-(FURZ)-(FBRX)-(FBRY)-(FBRZ)-(SBLX)-(SBLY)-(SBLZ)-(OALX)-(OALY)-(OALZ)-(UALX)-(UALY)-(UALZ)-(HALX)-(HALY)-(HALZ)-(FILX)-(FILY)-(FILZ)-(SBRX)-(SBRY)-(SBRZ)-(OARX)-(OARY)-(OARZ)-(UARX)-(UARY)-(UARZ)-(HARX)-(HARY)-(HARZ)-(FIRX)-(FIRY)-(FIRZ)
	// 	 and thus an exception: in order to limit the amount of data to be transferred,
	//   they already contain all information that is required for drawing the poses
	//
	//
	// - ignore if something that is to be added already exists
	// - clearHighlighting before highlighting a new something
	//
	//
	
	
	// add and highlight object (create if not there, highlight in any case)
	// position and dimensions: see KitchenVisApplet
	// type: rdf_has(O, rdf:type, T)
	void addObject(String obj);
	
	
	// add and highlight object and all its children 
	// (read all objects that are linked via parts-relations)
	//
	// (rdf_reachable(Obj, knowrob:describedInMap, Parent));
    // (rdf_reachable(A, knowrob:describedInMap, Parent),rdf_reachable(A, knowrob:properPhysicalPartTypes, Obj))
	//
	void addObjectWithChildren(String obj);
	
	
	// highlight object if exists (do not create a new one)
	void highlightObject(String obj);
	
	
	// add action sequence, position the cursor to the beginning, draw human
	// format: see above
	void addActionSequence(String[] actionseq);

	
	// add door events that are shown when the resp. time is displayed
	// check type with rdf_has(Event, rdf:type, knowrob:OpeningADoor) or ...#ClosingADoor)
	// and read the door id using knowrob:objectActedOn (to be implemented)
	void addDoorEvents(String[] doorevents);
	
	
	// add object interactions
	// check type with rdf_has(Event, rdf:type, knowrob:PickingUpAnObject) or ...PuttingDownAnObject)
	// and further info using knowrob:objectActedOn (implemented), knowrob:toLocation, knowrob:fromLocation (to be implemented)
	void addManipulationActions(String[] manipactions);
	
	
	// set current time to T (jump inside the episode, show the state at time t as far as known)
	void setDisplayTime(float time);



	////////////////////////////////////////////////////////////////////
	// play functions? (low implementation priority)
	//
	void play();
	
	void playFromTime(float time);
	
	void playFromStartTillEnd(float start, float end);
	
	
	
	////////////////////////////////////////////////////////////////////
	// clear buffers
	//
	
	void clear();

	void clearHighlighting();
	
	void clearActions();

	void clearEvents();

	void clearObjects();
	
}
