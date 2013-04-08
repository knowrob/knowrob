package edu.tum.cs.ias.knowrob.vis.actions;

import java.awt.Color;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import javax.vecmath.Vector2f;

import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.vis.applets.PlanVisApplet;

import processing.core.PApplet;

/**
 * This class is used in an Action class for saving parameters used during draw of the action.
 * For example this stores the position/size of the action box.
 * 
 * There are two different drawing types:
 *  - Simple box: Contains only title and properties
 *  - Extended box: Parents and children of action, properties and subsequences.
 * 
 * @author Stefan Profanter
 * @see edu.tum.cs.ias.knowrob.vis.actions.Action
 */
public class ActionDrawInformation {
	
	/**
	 * Text line height
	 */
	public final static float LINE_HEIGHT = 1.5f;
	/**
	 * Padding between sections of the action.
	 * E.g. between properties and subsequence box 
	 */
	public final static float INNER_CONTENT_PADDING = 10f;
	/**
	 * Padding between main box border and content
	 */
	public final static float MAIN_BOX_PADDING = 15f;
	/**
	 * Padding around a single sequence box in the subsequence box
	 */
	public final static float SEQUENCE_BOX_PADDING = 10f;
	/**
	 * total height of the expand box under a subsequence box
	 */
	public final static float EXPAND_BOX_HEIGHT = 18f;
	
	/**
	 * Dimensions of the box-connectors
	 */
	public final static Vector2f CONNECTOR_DIM = new Vector2f(50f, 7f);
	
	/**
	 * Default line/border width
	 */
	private final static float defaultStroke = 1f;
	/**
	 * line/border width of current action 
	 */
	private final static float currentStroke = 1.5f;
	
	
	
	
	
	
	/**
	 * Default border color for boxes
	 */
	private final static Color borderColor = new Color(100, 100, 100);
	/**
	 * Default background color for boxes
	 */
	private final static Color backgroundColor = new Color(30,30,30);
	/**
	 * Default background color for subsequence box
	 */
	private final static Color backgroundBrightColor = new Color(50,50,50);
	/**
	 * Default text color
	 */
	private final static Color textColor = new Color(240,240,240);
	/**
	 * border color for boxes when hovering
	 */
	private final static Color hoverBorderColor = new Color(200, 200, 200);
	/**
	 * background color for boxes when hovering
	 */
	private final static Color hoverBackgroundColor = new Color(60,60,60);
	/**
	 * background color for subsequence box when hovering
	 */
	private final static Color hoverBackgroundBrightColor = new Color(100,100,100);
	/**
	 * text color when hovering
	 */
	private final static Color hoverTextColor = new Color(255,255,255);
	/**
	 * Border color of arrow between sequence boxes
	 */
	private final static Color arrowBorderColor = new Color(27, 56, 102);
	/**
	 * Background color of arrow between sequence boxes
	 */
	private final static Color arrowBackgroundColor = new Color(51,105,192,230);
	/**
	 * Border color of arrow for expanding box
	 */
	private final static Color arrowExpandBorderColor = new Color(94, 61, 61);
	/**
	 * Background color of arrow expanding box
	 */
	private final static Color arrowExpandBackgroundColor = new Color(125,67,67,230);	
	/**
	 * Border color of arrow for expanding box
	 */
	private final static Color hoverArrowExpandBorderColor = new Color(190, 123, 123);
	/**
	 * Background color of arrow expanding box
	 */
	private final static Color hoverArrowExpandBackgroundColor = new Color(190,141,141,230);
	/**
	 * Border color of parents of highlighted action
	 */
	private final static Color highlightBorderColor = new Color(103,134,85);
	/**
	 * Border color of hightlighted action
	 */
	private final static Color highlightBrightBorderColor = new Color(167,217,137);
	
	
	
	
	
	
	
	/**
	 * Type of highlight.
	 * Indicates if action is hightlighted or not and which color to use.
	 * @author Stefan Profanter
	 *
	 */
	public static enum HighlightType {
		/**
		 * Use default color
		 */
		NOT_HIGHTLIGHTED,
		/**
		 * Use highlight color
		 */
		CHILD_HIGHLIGHTED,
		/**
		 * Use bright highlight color
		 */
		THIS_HIGHLIGHTED
	}
	/**
	 * Highlight state of current action
	 */
	private HighlightType highlight = HighlightType.NOT_HIGHTLIGHTED;
	
	
	
	

	/**
	 * Used for intern calculation such as if mouse is hovering.
	 * Indicates if drawSimpleBox or drawExtendedBox was used to draw this action.
	 */
	private boolean drawnAsSimple;
	
	/**
	 * Following colors are set to default colors if not hovering
	 */
	private Color currentBorderColor = borderColor;
	private Color currentBackgroundColor = backgroundColor;
	private Color currentBackgroundBrightColor = backgroundBrightColor;
	private Color currentTextColor = textColor;
	
	/**
	 * Following colors are set to default colors if not hovering over expand box
	 */
	private Color currentExpandBorderColor = arrowExpandBorderColor;
	private Color currentExpandBackgroundColor = arrowExpandBackgroundColor;
	
	/**
	 * parent action of this draw info
	 */
	private Action action;
	
	/**
	 * set to true if parameters need a recalculation.
	 * This is the case if properties/subactions/... have changed
	 */
	private boolean needsRecalculation = true;
	
	/**
	 * True if mouse if hovering over this action
	 */
	private boolean isHover = false;
	
	/**
	 * Width of the name text, used to calculate max box width
	 */
	private float nameWidth;
	
	/**
	 * Maximum width over all key texts, used to align properties text
	 */
	private float maxKeyWidth;
	/**
	 * Maximum width over all value texts, used to calculate max box width
	 */
	private float maxValueWidth;
	/**
	 * Height of properties all together. Used to calculate max box height
	 */
	private float propertiesHeight;
	
	/**
	 * Height of a single text line. Used for calculating text positions
	 */
	private float textHeight;
	
//	/**
//	 * Maximum box height over all parent actions of this action.
//	 * Used to draw all parents with same height
//	 */
//	private float parentsMaxHeight;
//	/**
//	 * Maximum box height over all child actions of this action.
//	 * Used to draw all parents with same height
//	 */
//	private float childrenMaxHeight;
//	
//	/**
//	 * X position of parent boxes
//	 */
//	private float parentStartX;
//	
//	/**
//	 * X position of child boxes
//	 */
//	private float childStartX;
	
	/**
	 * If parent or child boxes are wider than the extended box, the extended box must be shifted a bit to the right.
	 * This shift indicates the following variable.
	 */
	private float boxOffsetLeft;
	
	/**
	 * Size of the sequence box containing the subsequences of an action.
	 */
	private Vector2f sequenceBoxDimension = new Vector2f();
	
	/**
	 * Height for all actions in the sequence box
	 */
	private float maxSubsequenceHeight = 0;
	
	/**
	 * Set to true if this action has additional subsequences which can be expanded.
	 */
	private boolean hasExpandButton = false;

	/**
	 * current position to draw the main box of the action
	 */
	public Vector2f position;

	/**
	 * action-specific position offset (action has been dragged selectively)
	 */
	public Vector2f localPosOffset;

	/**
	 * global position offset (canvas has been dragged)
	 */
	public Vector2f globalPosOffset;
	
	
	
	/**
	 * Position and dimension of Expanded and Simple box
	 */
	private Rectangle boundingBoxExtended = new Rectangle(0,0,0,0);
	private Rectangle boundingBoxSimple = new Rectangle(0,0,0,0);
	
	
	
	/**
	 * Constructor
	 * @param parent Parent action for which this draw infos are
	 */
	public ActionDrawInformation(Action parent)
	{
		action = parent;
		localPosOffset = new Vector2f();
		globalPosOffset = new Vector2f();
		position = new Vector2f();
	}
	
	/**
	 * Notify all subsequences that something has changed and recalculation is needed
	 * @param a
	 */
	private void notifyModifiedSubsequences(Action a)
	{
		if (a==null)
			return;
		a.getDrawInfo().notifyModified();
		notifyModifiedSubsequences(a.getExpandedSequence());
	}
	
	/**
	 * Call this if action properties/subsequences/parents/children changed.
	 * This tells the class to recalculate the dimensions of all boxes
	 */
	public void notifyModified()
	{
		needsRecalculation = true;
		notifyModifiedSubsequences(action.getExpandedSequence());
	}
	
	/**
	 * Height of title box of the action
	 * @return Height of the name box in pixels
	 */
	public float getNameBoxHeight()
	{
		return textHeight*3;
	}
	
	/**
	 * Get width of name in pixels.
	 * @return Width of name in pixels
	 */
	public float getNameWidth()
	{
		return nameWidth;
	}
	
	/**
	 * Get dimension of the simple box.
	 * The simple box contains only title and properties
	 * @return the dimension in pixels of the simple box
	 */
	public Vector2f getSimpleBoxDimension()
	{
		//Max with of name or properties
		float width = Math.max(nameWidth,maxKeyWidth+maxValueWidth)+ MAIN_BOX_PADDING*2;
		
		//Height of name box
		float height = getNameBoxHeight();
		
		height += propertiesHeight+INNER_CONTENT_PADDING + MAIN_BOX_PADDING;
		
		return new Vector2f(width, height);
	}
	
	/**
	 * Get dimension of the extended box.
	 * The extended box contains title, properties and subsequences
	 * @return the dimension in pixels of the simple box
	 */
	public Vector2f getExtendedBoxDimension()
	{
		Vector2f dim = getSimpleBoxDimension();
		dim.x = Math.max(dim.x,sequenceBoxDimension.x + 2*MAIN_BOX_PADDING);
		dim.y += sequenceBoxDimension.y + INNER_CONTENT_PADDING;
		
		return dim;
	}
	
	/**
	 * Needed for calculating the total size of the sequence box.
	 * Adds the size of an expanded box to the current calculated size 
	 * @param applet Applet to draw on
	 * @param currentSequenceBoxSize currently calculated size. If a child-sequence is expanded then it's size will be added to this.
	 * @param currAction current action for which the subsequences should be checked
	 */
	private static void checkAndAddSizeOfExpandedSequence(PApplet applet,Vector2f currentSequenceBoxSize, Action currAction)
	{
		if (currAction.getDrawInfo().needsRecalculation)
			currAction.getDrawInfo().recalculateDimensions(applet);
		if (currAction.getExpandedSequence() != null)
		{
			Action exp = currAction.getExpandedSequence();
			currentSequenceBoxSize.x = Math.max(currentSequenceBoxSize.x,exp.getDrawInfo().sequenceBoxDimension.x+2*SEQUENCE_BOX_PADDING);
			currentSequenceBoxSize.y += exp.getDrawInfo().sequenceBoxDimension.y;
		}
	}
	
	/**
	 * Recalculate the dimension of simple/extended box.
	 * Only executed if needsRecalculation is set to true (notifyModified called)
	 * @param applet Applet used to draw the action. Only used for text calculation.
	 * 	Because width of text is dependent on current text setting
	 */
	public void recalculateDimensions(PApplet applet)
	{
		if (!needsRecalculation)
			return;
		
		float totalWidth = 0;
		float totalHeight = 0;
		
		//Text dimension
		textHeight = applet.textAscent();
		nameWidth = applet.textWidth(action.getLabel());
		
		//Properties of the action
		maxKeyWidth = 0;
		maxValueWidth = 0;
		propertiesHeight = 0;
		
		// add one line to properties for type: info
		propertiesHeight += textHeight* LINE_HEIGHT;
		
		Collection<String> keys = action.getProperties().keySet();
		for (Iterator<String> it = keys.iterator(); it.hasNext(); )
		{
			String key = it.next();
			maxKeyWidth = Math.max(maxKeyWidth, applet.textWidth( OWLThing.getShortNameOfIRI(key)+":  "));
			for (String value : action.getProperty(key))
			{
				maxValueWidth = Math.max(maxValueWidth, applet.textWidth( OWLThing.getShortNameOfIRI(value)));
				propertiesHeight += textHeight* LINE_HEIGHT;
			}
		}
		
		needsRecalculation = false;
		
		//Calculation for sequence boxes
		sequenceBoxDimension.x = SEQUENCE_BOX_PADDING;
		sequenceBoxDimension.y = 0;
		hasExpandButton = false;
		
		List<Action> sub = action.getSubActions();
		synchronized(sub) {
			for (Action a : sub) {

				if (a.getSubActionsCount() > 0)
					hasExpandButton = true;
				ActionDrawInformation inf = a.getDrawInfo();

				inf.recalculateDimensions(applet);

				sequenceBoxDimension.x += inf.getSimpleBoxDimension().x + SEQUENCE_BOX_PADDING;
				sequenceBoxDimension.y = Math.max(sequenceBoxDimension.y, inf.getSimpleBoxDimension().y + 2*SEQUENCE_BOX_PADDING);
			}
		}
		maxSubsequenceHeight = sequenceBoxDimension.y- 2*SEQUENCE_BOX_PADDING;
		if (hasExpandButton)
			sequenceBoxDimension.y += EXPAND_BOX_HEIGHT;
		checkAndAddSizeOfExpandedSequence(applet,sequenceBoxDimension,action);
		
//		//Calculation for parent boxes
//		parentsMaxHeight = 0;
//		float parentsWidth = 0;
//		for (Iterator<Action> i = action.getParentActionsIterator(); i.hasNext(); )
//		{
//			ActionDrawInformation inf = i.next().getDrawInfo();
//			
//			inf.recalculateDimensions(applet);
//			
//			parentsMaxHeight = Math.max(parentsMaxHeight, inf.getSimpleBoxDimension().y);
//			parentsWidth += inf.getSimpleBoxDimension().x+MAIN_BOX_PADDING;
//		}
//		parentsWidth = Math.max(0, parentsWidth-MAIN_BOX_PADDING);
//		
//		totalWidth = Math.max(totalWidth, parentsWidth);
//		totalHeight += parentsMaxHeight+MAIN_BOX_PADDING;
		
//		//Calculation for child boxes
//		childrenMaxHeight = 0;
//		float childrenWidth = 0;
//		for (Iterator<Action> i = action.getChildActionsIterator(); i.hasNext(); )
//		{
//			ActionDrawInformation inf = i.next().getDrawInfo();
//			
//			inf.recalculateDimensions(applet);
//			
//			childrenMaxHeight = Math.max(childrenMaxHeight, inf.getSimpleBoxDimension().y);
//			childrenWidth += inf.getSimpleBoxDimension().x+MAIN_BOX_PADDING;
//		}
//		childrenWidth = Math.max(0, childrenWidth-MAIN_BOX_PADDING);
//		totalWidth = Math.max(totalWidth, childrenWidth);
//		totalHeight += childrenMaxHeight+MAIN_BOX_PADDING;
		
//		this.boxOffsetLeft = Math.max(0, Math.max((parentsWidth-getExtendedBoxDimension().x)/2f,(childrenWidth-getExtendedBoxDimension().x)/2f));
		
//		this.parentStartX = Math.max(0, getExtendedBoxDimension().x/2f-parentsWidth/2f);
//		this.childStartX = Math.max(0, getExtendedBoxDimension().x/2f-childrenWidth/2f);
		
		totalWidth = Math.max(totalWidth, getExtendedBoxDimension().x);
		totalHeight += getExtendedBoxDimension().y;
		
		boundingBoxExtended.width = (int)totalWidth;
		boundingBoxExtended.height = (int)totalHeight;
		
		boundingBoxSimple.width = (int)getSimpleBoxDimension().x;
		boundingBoxSimple.height = (int)getSimpleBoxDimension().y;

	}
	
	/**
	 * Draw border and title of the action with given dimension at given position
	 * @param applet Applet to draw on
	 * @param position position where to draw the box
	 * @param dimension dimension of the box to draw
	 */
	private void drawBorderAndTitle(PApplet applet, Vector2f position, Vector2f dimension)
	{
		//Draw outer border and background
	    applet.fill(currentBackgroundColor.getRed(), currentBackgroundColor.getGreen(), currentBackgroundColor.getBlue(), currentBackgroundColor.getAlpha());
		applet.rect(position.x, position.y, dimension.x, dimension.y);
		
		//Draw box title
		applet.fill(currentBackgroundBrightColor.getRed(), currentBackgroundBrightColor.getGreen(), currentBackgroundBrightColor.getBlue(), currentBackgroundBrightColor.getAlpha());
		applet.rect(position.x,position.y,dimension.x, getNameBoxHeight());
		applet.fill(currentTextColor.getRed(), currentTextColor.getGreen(), currentTextColor.getBlue(), currentTextColor.getAlpha());
		applet.text(action.getLabel(),position.x + MAIN_BOX_PADDING,position.y + textHeight*2f);
	}
	
	/**
	 * Draw the action properties beginning at the given position.
	 * @param applet Applet to draw on
	 * @param position The upper left position to begin the drawing
	 */
	private void drawProperties(PApplet applet, Vector2f position)
	{
		applet.fill(currentTextColor.getRed(), currentTextColor.getGreen(), currentTextColor.getBlue(), currentTextColor.getAlpha());
		
		for (String key : action.getProperties().keySet() ) {
			
			for (String value : action.getProperty(key)) {
				
				applet.text( OWLThing.getShortNameOfIRI(key) + ":",position.x,position.y+textHeight);
				applet.text( OWLThing.getShortNameOfIRI(value),position.x + maxKeyWidth, position.y+textHeight);
	
				position.y += textHeight * LINE_HEIGHT;
			}
		}
	}
	
	/**
	 * Draw the simple box. Contains title and properties
	 * @param applet Applet to draw on
	 * @param position Position where to draw the box
	 * @param minHeight minimum height of the box.
	 */
	public void drawSimpleBox(PApplet applet,Vector2f position, Vector2f drawOffset, float minHeight)
	{
		drawSimpleBox(applet, position, drawOffset, minHeight,false);
	}
	
	/**
	 * Draw the simple box. Contains title and properties
	 * @param applet Applet to draw on
	 * @param position Position where to draw the box
	 * @param minHeight minimum height of the box.
	 */
	public void drawSimpleBox(PApplet applet, Vector2f position, Vector2f drawOffset, float minHeight, boolean drawExpandBox)
	{
		this.drawnAsSimple = true;
		this.position = new Vector2f(position);
		this.globalPosOffset = drawOffset;
		boundingBoxSimple.x = (int)position.x;
		boundingBoxSimple.y = (int)position.y;
		Vector2f tmpPos = new Vector2f(position.x + localPosOffset.x + drawOffset.x, position.y + localPosOffset.y + drawOffset.y);
		recalculateDimensions(applet);
		
		if (highlight == HighlightType.NOT_HIGHTLIGHTED)
			applet.stroke(currentBorderColor.getRed(), currentBorderColor.getGreen(), currentBorderColor.getBlue(), currentBorderColor.getAlpha());
		else if (highlight == HighlightType.CHILD_HIGHLIGHTED)
			applet.stroke(highlightBorderColor.getRed(), highlightBorderColor.getGreen(), highlightBorderColor.getBlue(), highlightBorderColor.getAlpha());
		else if (highlight == HighlightType.THIS_HIGHLIGHTED)
		{
			applet.strokeWeight(currentStroke);
			applet.stroke(highlightBrightBorderColor.getRed(), highlightBrightBorderColor.getGreen(), highlightBrightBorderColor.getBlue(), highlightBrightBorderColor.getAlpha());
		}
		drawBorderAndTitle(applet, tmpPos, new Vector2f(getSimpleBoxDimension().x,Math.max(minHeight, getSimpleBoxDimension().y)));
		applet.strokeWeight(defaultStroke);
		
		if (drawExpandBox)
		{
			applet.stroke(currentExpandBorderColor.getRed(), currentExpandBorderColor.getGreen(), currentExpandBorderColor.getBlue(), currentExpandBorderColor.getAlpha());
		    applet.fill(currentExpandBackgroundColor.getRed(), currentExpandBackgroundColor.getGreen(), currentExpandBackgroundColor.getBlue(), currentExpandBackgroundColor.getAlpha());
		    

			Vector2f from = new Vector2f(tmpPos.x+getSimpleBoxDimension().x/2f,tmpPos.y+Math.max(minHeight, getSimpleBoxDimension().y)+4);
			Vector2f to = new Vector2f(tmpPos.x+getSimpleBoxDimension().x/2f,tmpPos.y+Math.max(minHeight, getSimpleBoxDimension().y)+EXPAND_BOX_HEIGHT-4);
		    if (action.isExpanded())
		    	PlanVisApplet.arrowFromTo(applet, to, from, 6,0);
		    else
		    	PlanVisApplet.arrowFromTo(applet, from, to, 6,0);
		}
		
		tmpPos.x += MAIN_BOX_PADDING;
		tmpPos.y += getNameBoxHeight()+INNER_CONTENT_PADDING;
		
		drawProperties(applet, tmpPos);
		
		// draw inbound and outbound connectors
		applet.fill(backgroundBrightColor.getRed(), backgroundBrightColor.getGreen(), backgroundBrightColor.getBlue(), backgroundBrightColor.getAlpha());
		applet.stroke(currentBorderColor.getRed(), currentBorderColor.getGreen(), currentBorderColor.getBlue(), currentBorderColor.getAlpha());
		applet.rect(getInboundConnectorPos().x-CONNECTOR_DIM.x/2, getInboundConnectorPos().y-CONNECTOR_DIM.y, CONNECTOR_DIM.x, CONNECTOR_DIM.y);
		
		// draw inbound and outbound connectors
		applet.fill(backgroundBrightColor.getRed(), backgroundBrightColor.getGreen(), backgroundBrightColor.getBlue(), backgroundBrightColor.getAlpha());
		applet.stroke(currentBorderColor.getRed(), currentBorderColor.getGreen(), currentBorderColor.getBlue(), currentBorderColor.getAlpha());
		applet.rect(getOutboundConnectorPos().x-CONNECTOR_DIM.x/2, getOutboundConnectorPos().y, CONNECTOR_DIM.x, CONNECTOR_DIM.y);
	}
	
//	/**
//	 * Draw the parent boxes of this action. These are located over the action box
//	 * @param applet Applet to draw on
//	 * @param position start position where to begin to draw
//	 * @return ArrayList of Vector2f with the points at the center bottom for drawing connection arrows
//	 */
//	private ArrayList<Vector2f> drawParentBoxes(PApplet applet, Vector2f position, Vector2f drawOffset)
//	{
//		Vector2f tmpPos = new Vector2f(position);
//		ArrayList<Vector2f> retPoints = new ArrayList<Vector2f>();
//		
//		for (Iterator<Action> i = action.getParentActionsIterator(); i.hasNext(); )
//		{
//			ActionDrawInformation inf = i.next().getDrawInfo();
//			
//			inf.drawSimpleBox(applet, tmpPos, drawOffset, parentsMaxHeight);
//		
//		    retPoints.add(new Vector2f(tmpPos.x + inf.getSimpleBoxDimension().x/2f, tmpPos.y+parentsMaxHeight-5));
//	
//			tmpPos.x += inf.getSimpleBoxDimension().x + MAIN_BOX_PADDING;
//		}
//		return retPoints;
//	}
	
//	/**
//	 * Draw the child boxes of this action. These are located under the extended action box
//	 * @param applet Applet to draw on
//	 * @param position start position where to begin to draw
//	 * @return ArrayList of Vector2f with the points at the center bottom for drawing connection arrows
//	 */
//	private ArrayList<Vector2f> drawChildrenBoxes(PApplet applet, Vector2f position, Vector2f drawOffset)
//	{
//		Vector2f tmpPos = new Vector2f(position);
//		ArrayList<Vector2f> retPoints = new ArrayList<Vector2f>();
//		
//		for (Iterator<Action> i = action.getChildActionsIterator(); i.hasNext(); )
//		{
//			ActionDrawInformation inf = i.next().getDrawInfo();
//			
//			inf.drawSimpleBox(applet, tmpPos, drawOffset, childrenMaxHeight);
//			
//		    retPoints.add(new Vector2f(tmpPos.x + inf.getSimpleBoxDimension().x/2f, tmpPos.y+5));
//			
//			tmpPos.x += inf.getSimpleBoxDimension().x + MAIN_BOX_PADDING;
//		}
//		return retPoints;
//	}
//	
//	/**
//	 * Connect each point of parentPoints with the current action box. Do the same with child points.
//	 * The target points will be calculated by this function with a homogeneous distribution over the
//	 * whole with of the current action box.
//	 * 
//	 * @param applet Applet to draw on
//	 * @param parentPoints Connection points of parent actions
//	 * @param childPoints Connection points of child actions
//	 */
//	private void drawArrows(PApplet applet, ArrayList<Vector2f> parentPoints, ArrayList<Vector2f> childPoints)
//	{
//		applet.stroke(arrowBorderColor.getRed(), arrowBorderColor.getGreen(), arrowBorderColor.getBlue(), arrowBorderColor.getAlpha());
//	    applet.fill(arrowBackgroundColor.getRed(), arrowBackgroundColor.getGreen(), arrowBackgroundColor.getBlue(), arrowBackgroundColor.getAlpha());
//
//	    Vector2f connPointParent = new Vector2f(position.x + localPosOffset.x + globalPosOffset.x, position.y + localPosOffset.y + globalPosOffset.x);
//	    connPointParent.x+=boxOffsetLeft;
//	    Vector2f connPointChildren = new Vector2f(connPointParent);
//	    connPointChildren.y += getExtendedBoxDimension().y-5;
//	    connPointParent.y += 5;
//	    
//	    float diffParent = getExtendedBoxDimension().x / (parentPoints.size()+1);
//	    
//	    for (int i=0; i<parentPoints.size(); i++)
//	    {
//	    	connPointParent.x += diffParent;
//	    	PlanVisApplet.arrowFromTo(applet, parentPoints.get(i), connPointParent, 5,-1);
//	    }
//	    
//	    float diffChild = getExtendedBoxDimension().x / (childPoints.size()+1);
//	    
//	    for (int i=0; i<childPoints.size(); i++)
//	    {
//	    	connPointChildren.x += diffChild;
//	    	PlanVisApplet.arrowFromTo(applet, connPointChildren,childPoints.get(i), 5,-1);
//	    }	    
//	}

	/**
	 * Draw the subsequence boxes of current action.
	 * @param applet Applet to draw on
	 * @param position Start position (upper left corner)
	 */
	private void drawSequence(PApplet applet, Vector2f position, Vector2f drawOffset, boolean isExpandedBox) {
		
		
		List<Action> sub = action.getSubActions();
		
		if (sub.isEmpty())
			return;
		
		synchronized(sub) {

			//Draw outer box of sequence list
			applet.stroke(currentBorderColor.getRed(), currentBorderColor.getGreen(), currentBorderColor.getBlue(), currentBorderColor.getAlpha());
			
			if (isExpandedBox)
				applet.fill(currentBackgroundColor.getRed(), currentBackgroundColor.getGreen(), currentBackgroundColor.getBlue(), currentBackgroundColor.getAlpha());
			else
				applet.fill(currentBackgroundBrightColor.getRed(), currentBackgroundBrightColor.getGreen(), currentBackgroundBrightColor.getBlue(), currentBackgroundBrightColor.getAlpha());
			
			applet.rect(position.x, position.y, sequenceBoxDimension.x, sequenceBoxDimension.y);

			//Calculate start pos of inner boxes
			Vector2f tmpPos = new Vector2f(position);
			tmpPos.y += SEQUENCE_BOX_PADDING;
			tmpPos.x += SEQUENCE_BOX_PADDING;

			//Store the positions of each box to connect them afterwards
			ArrayList<Float> pos = new ArrayList<Float>();

			Vector2f expandedParentCorner1 = null;
			Vector2f expandedParentCorner2 = null;

			Action expSeq = action.getExpandedSequence();


			//Draw inner boxes
			for (Action a : sub) {
				
				ActionDrawInformation inf = a.getDrawInfo();

				inf.drawSimpleBox(applet, tmpPos, drawOffset, maxSubsequenceHeight,a.getSubActionsCount()>0);
				if (a == expSeq) {
					expandedParentCorner1 = new Vector2f(tmpPos.x,tmpPos.y+maxSubsequenceHeight);
					expandedParentCorner2 = new Vector2f(tmpPos.x+inf.getSimpleBoxDimension().x,tmpPos.y+maxSubsequenceHeight);
				}

				tmpPos.x += inf.getSimpleBoxDimension().x + SEQUENCE_BOX_PADDING;
				pos.add(tmpPos.x);
			}

			//Draw connection arrows for each sequence box
			for (int i=0; i<pos.size()-1; i++) {
				
				applet.stroke(arrowBorderColor.getRed(), arrowBorderColor.getGreen(), arrowBorderColor.getBlue(), arrowBorderColor.getAlpha());
				applet.fill(arrowBackgroundColor.getRed(), arrowBackgroundColor.getGreen(), arrowBackgroundColor.getBlue(), arrowBackgroundColor.getAlpha());
				PlanVisApplet.arrow(applet,pos.get(i)-SEQUENCE_BOX_PADDING - 5,tmpPos.y+getNameBoxHeight()/2-10,20,SEQUENCE_BOX_PADDING+11);
			}

			//Draw expanded subsequences if any
			Vector2f startPos = new Vector2f(position);
			startPos.y +=SEQUENCE_BOX_PADDING + maxSubsequenceHeight + EXPAND_BOX_HEIGHT;

			if (expSeq != null && expandedParentCorner1 != null && expandedParentCorner2 != null) {
				
				Vector2f subDim = expSeq.getDrawInfo().sequenceBoxDimension;
				startPos.x = (expandedParentCorner2.x+expandedParentCorner1.x)/2f;
				startPos.x -= subDim.x/2f;

				startPos.x = Math.max(startPos.x, position.x+SEQUENCE_BOX_PADDING);

				startPos.x -= Math.max(0, (startPos.x +subDim.x) - (position.x+sequenceBoxDimension.x-SEQUENCE_BOX_PADDING));


				//applet.stroke(currentBorderColor.getRed(), currentBorderColor.getGreen(), currentBorderColor.getBlue(), currentBorderColor.getAlpha());
				//applet.fill(currentBackgroundColor.getRed(), currentBackgroundColor.getGreen(), currentBackgroundColor.getBlue(), currentBackgroundColor.getAlpha());
				//applet.line(expandedParentCorner1.x, expandedParentCorner1.y, startPos.x, startPos.y);
				//applet.line(expandedParentCorner2.x, expandedParentCorner2.y, startPos.x+subDim.x, startPos.y);

				expSeq.getDrawInfo().drawSequence(applet, startPos, drawOffset, true);
			}
		}
	}
	
	/**
	 * Draw the extended action box which contains properties and subsequences. Also parent and child actions will be drawn.
	 * @param applet Applet to draw on.
	 * @param position Upper left corner where to begin to draw. If the action has parent action(s) it will be the position of
	 * the first parent box. If none parent actions are present, this will be the upper left corner of the extended box.
	 */
	public void drawExtendedBox(PApplet applet, Vector2f position, Vector2f drawOffset)
	{
		this.drawnAsSimple = false;
		recalculateDimensions(applet);
		
		boundingBoxExtended.x = (int)position.x;
		boundingBoxExtended.y = (int)position.y;
		
		Vector2f extendedDim = getExtendedBoxDimension();
		
		//Draw parents
//		ArrayList<Vector2f> parentPoints = drawParentBoxes(applet, new Vector2f(position.x + parentStartX,position.y), drawOffset);
//		
//		position.y += parentsMaxHeight+MAIN_BOX_PADDING;

		//Border & title
		this.position = new Vector2f(position);
		applet.strokeWeight(currentStroke);
		applet.stroke(hoverBorderColor.getRed(), hoverBorderColor.getGreen(), hoverBorderColor.getBlue(), hoverBorderColor.getAlpha());
		position.x += boxOffsetLeft;
		drawBorderAndTitle(applet, position,extendedDim);	
		applet.strokeWeight(defaultStroke);	
		
		position.x += MAIN_BOX_PADDING;
		position.y += getNameBoxHeight()+INNER_CONTENT_PADDING;
		
		//Properties
		drawProperties(applet, position);

		position.y += INNER_CONTENT_PADDING;
		
		//Subsequences
		drawSequence(applet, position, new Vector2f(), false);
		
		position.y += sequenceBoxDimension.y;
		position.x -= MAIN_BOX_PADDING;
		position.x -= boxOffsetLeft;
		
		position.y += MAIN_BOX_PADDING*2;
		//Children
//		ArrayList<Vector2f> childPoints = drawChildrenBoxes(applet, new Vector2f(position.x + childStartX,position.y),  drawOffset);
		
//		drawArrows(applet, parentPoints, childPoints);
	}
	
	/**
	 * Set the correct colors when hovering over action.
	 * If mouse if over action, the hover* colors will be used to draw it.
	 * @param hover If true use hover* colors, otherwise use normal colors
	 */
	private void setHover(boolean hover)
	{
		if (hover)
		{
			currentBorderColor = hoverBorderColor;
			currentBackgroundColor = hoverBackgroundColor;
			currentBackgroundBrightColor = hoverBackgroundBrightColor;
			currentTextColor = hoverTextColor;
		} else 
		{
			currentBorderColor = borderColor;
			currentBackgroundColor = backgroundColor;
			currentBackgroundBrightColor = backgroundBrightColor;
			currentTextColor = textColor;
		}
		isHover = hover;
	}
	
	/**
	 * Set the correct colors when hovering over expand arrow.
	 * If mouse if over expand arrow, the hover* colors will be used to draw it.
	 * @param hover If true use hover* colors, otherwise use normal colors
	 */
	private void setHoverExpand(boolean hover)
	{
		if (hover || action.isExpanded())
		{
			currentExpandBorderColor = hoverArrowExpandBorderColor;
			currentExpandBackgroundColor = hoverArrowExpandBackgroundColor;
		} else 
		{
			currentExpandBorderColor = arrowExpandBorderColor;
			currentExpandBackgroundColor = arrowExpandBackgroundColor;
		}
	}
	
	/**
	 * Check if x|y coordinate is on a currently visible subsequence (also expanded ones)
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param currAction current action to check
	 * @param found set to an action if already one action found and the others only need to be resetted.
	 * @return the action where the coordinate is on
	 */
	private ActionDrawInformation checkHoverExtendedSequence(float x, float y, Action currAction, ActionDrawInformation found) {
		
		if (currAction == null)
			return null;
		
		//over sequences?
		
		List<Action> sub = action.getSubActions();
		
		synchronized(sub) {

			for (Action a : sub) {
				
				ActionDrawInformation inf = a.getDrawInfo();

				if (found != null)
					inf.setHover(false);
				else {
					found = inf.checkHover(x, y,found);	
				}
			}
			return found;
		}
	}
	
	/**
	 * Check if mouse position x|y is over this action
	 * @param x x coordinate of mouse
	 * @param y y coordinate of mouse
	 * @param foundInParent set to true if aleady a matching box was found, so only setHover(false) is needed to call 
	 * @return the Action (its draw info) over which the mouse is hovering or null if none.
	 */
	public ActionDrawInformation checkHover(float x, float y, ActionDrawInformation found)
	{

		if (position == null)
			return found;
			
		//over extended sequences?
		found = checkHoverExtendedSequence(x,y,action.getExpandedSequence(), found);
		
		//Check for extended box
		if (!drawnAsSimple)
		{

			//over sequences?
			List<Action> sub = action.getSubActions();
			
			synchronized(sub) {

				for (Action a : sub) {
					
					ActionDrawInformation inf = a.getDrawInfo();

					if (found != null)
						inf.setHover(false);
					else
						if (inf.checkHover(x, y,found)!=null)
							found = inf;
				}
			}
			
//			//over parent?
//			for (Iterator<Action> i = action.getParentActionsIterator(); i.hasNext(); )
//			{
//				ActionDrawInformation inf = i.next().getDrawInfo();
//				
//				if (found != null)
//					inf.setHover(false);
//				else
//					if (inf.checkHover(x, y,found)!=null)
//						found = inf;
//			}
//			
//			//over child
//			for (Iterator<Action> i = action.getChildActionsIterator(); i.hasNext(); )
//			{
//				ActionDrawInformation inf = i.next().getDrawInfo();
//				
//				if (found != null)
//					inf.setHover(false);
//				else
//					if (inf.checkHover(x, y,found)!=null)
//						found = inf;
//			}
			
			//not hovering, so set correct colors
			setHover(false);
			return found; //will be null
			
		} else if(checkPosOverInboundConnector(x, y)!=null) {
			setHover(true);
			return this;		
			
		} else if(checkPosOverOutboundConnector(x, y)!=null) {
			setHover(true);
			return this;
			
		} else {
			//Check hovering if drawn as simple box
			Vector2f dim = getSimpleBoxDimension();
			float boxX,boxY,boxW,boxH;
			boxX = position.x + localPosOffset.x + globalPosOffset.x;
			boxY = position.y + localPosOffset.y + globalPosOffset.y;
			boxW = dim.x;
			boxH = dim.y;
			if (x>boxX && x<boxX+boxW && y>boxY && y<boxY+boxH)
			{
				setHover(true);
				return this;
			} else {
				setHover(false);
				return found;
			}
		}
		
	}
	
	/**
	 * Check if mouse position x|y is over an expand arrow
	 * @param mouse_x x coordinate of mouse
	 * @param mouse_y y coordinate of mouse
	 * @param sequenceExpandHeight set to 0 for direct call. Only used for recursing
	 * @return the parent Action (it's draw info) over which's expand arrow the mouse is hovering or null if none.
	 */
	private ActionDrawInformation checkHoverExpand(float mouse_x, float mouse_y, boolean hasExpandButton, float sequenceBoxHeight, boolean parentExpanded) {

		if (position == null)
			return null;
		
		ActionDrawInformation found = null;
			
		//over sequences?
		if (parentExpanded)	{
			
			List<Action> sub = action.getSubActions();
			
			synchronized(sub) {
				for (Action a : sub) {
					
					ActionDrawInformation inf = a.getDrawInfo();

					if (found != null)
						inf.setHoverExpand(false);
					else {
						ActionDrawInformation d = inf.checkHoverExpand(mouse_x, mouse_y,this.hasExpandButton,this.maxSubsequenceHeight, a.isExpanded());
						if (d!=null)
							found = d;
					}
				}
			}
		}
		
		if (hasExpandButton) {
			//Check hovering if drawn as simple box
			Vector2f dim = getSimpleBoxDimension();
			float boxX,boxY,boxW,boxH;
			boxX = position.x + localPosOffset.x + globalPosOffset.x;
			boxY = position.y + localPosOffset.y + globalPosOffset.y +sequenceBoxHeight;
			boxW = dim.x;
			boxH = EXPAND_BOX_HEIGHT;
			if (action.getSubActionsCount() > 0 && mouse_x>boxX && mouse_x<boxX+boxW && mouse_y>boxY && mouse_y<boxY+boxH)
			{
				setHoverExpand(true);
				return this;
			} else {
				setHoverExpand(false);
				return found;
			}
		}
		
		return found;
		
	}
	
	
	/**
	 * Check if mouse position is inside inbound connector
	 * 
	 * @param mouse_x x coordinate of mouse
	 * @param mouse_y y coordinate of mouse
	 * @return This action (if connector was clicked on), or null if none.
	 */
	public Action checkPosOverInboundConnector(float mouse_x, float mouse_y) {
		
		float boxX = getInboundConnectorPos().x-CONNECTOR_DIM.x/2;
		float boxY = getInboundConnectorPos().y-CONNECTOR_DIM.y;
		float boxW = CONNECTOR_DIM.x;
		float boxH = CONNECTOR_DIM.y;
		
		if (mouse_x>boxX && mouse_x<boxX+boxW && 
			mouse_y>boxY && mouse_y<boxY+boxH) {
			return this.action;
		} 
		return null;
	}
	
	
	/**
	 * Check if mouse position is inside outbound connector
	 * 
	 * @param mouse_x x coordinate of mouse
	 * @param mouse_y y coordinate of mouse
	 * @return This action (if connector was clicked on), or null if none.
	 */
	public Action checkPosOverOutboundConnector(float mouse_x, float mouse_y) {
		
		float boxX = getOutboundConnectorPos().x-CONNECTOR_DIM.x/2;
		float boxY = getOutboundConnectorPos().y;
		float boxW = CONNECTOR_DIM.x;
		float boxH = CONNECTOR_DIM.y;
		
		if (mouse_x>boxX && mouse_x<boxX+boxW && 
			mouse_y>boxY && mouse_y<boxY+boxH) {
			
			return this.action;
		} 
		return null;
	}
	
	
	/**
	 * Update color of action for current mouse position.
	 * @param x x coordinate of mouse
	 * @param y y coordinate of mouse
	 * @return true if hovering, false if not
	 */
	public boolean updateHover(float x, float y)
	{
		boolean found = checkHover(x,y,null)!=null;
		return (checkHoverExpand(x, y, false,0,true)!=null || found);
	}
	
	/**
	 * Check if mouse click on position x|y was intended for an action.
	 * @param x x coordinate of mouse
	 * @param y y coordinate of mouse
	 * @return Action which is under given coordinate of null if none
	 */
	public Action checkClick(float x, float y)
	{
		ActionDrawInformation a = checkHover(x,y,null);
		if (a==null)
			return null;
		else
			return a.action;
	}
	
	/**
	 * Check if mouse click on position x|y was intended for an expand button.
	 * @param x x coordinate of mouse
	 * @param y y coordinate of mouse
	 * @return Action for which this expand button is
	 */
	public Action checkClickExpand(float x, float y)
	{
		ActionDrawInformation a = checkHoverExpand(x, y, false,0,true);
		if (a==null)
			return null;
		else
			return a.action;
	}
	
	/**
	 * Get height of drawing text
	 * @return text height
	 */
	public float getTextHeight()
	{
		return textHeight;
	}
	
	/**
	 * Get action state if mouse is hovering over this action.
	 * @return true if hovering
	 */
	public boolean IsHover()
	{
		return isHover;
	}
	
	/**
	 * Get the bounding box of currently drawn action.
	 * If drawn as simple, the simple bounding box is returned, otherwise the extended bounding box.
	 * @return bounding box as rectangle
	 */
	public Rectangle getBoundingBox()
	{
		if (drawnAsSimple)
			return boundingBoxSimple;
		else
			return boundingBoxExtended;
	}
	
	/**
	 * Get the highlight status of the action
	 * @return the highlight status
	 */
	public HighlightType getHighlight()
	{
		return highlight;
	}
	
	/**
	 * Set the highlight status of the action
	 * @param h status to set
	 */
	public void setHightlight(HighlightType h)
	{
		this.highlight = h;
	}
	
	/**
	 * Clear the highlight status of the action (set it to NOT_HIGHLIGHTED).
	 * Also clear the hightlight on all the children.
	 */
	public void clearHightlight()
	{
		if (this.highlight == HighlightType.NOT_HIGHTLIGHTED)
			return;
		else if (this.highlight == HighlightType.THIS_HIGHLIGHTED)
		{
			this.highlight = HighlightType.NOT_HIGHTLIGHTED;
			return;
		}
		
		List<Action> sub = action.getSubActions();
		
		synchronized(sub) {
			for (Action a : sub) {

				ActionDrawInformation inf = a.getDrawInfo();

				if (inf.highlight == HighlightType.CHILD_HIGHLIGHTED || inf.highlight == HighlightType.THIS_HIGHLIGHTED)
				{
					inf.clearHightlight();
					this.highlight = HighlightType.NOT_HIGHTLIGHTED;
					return;
				}
			}
		}
	}
	
	/**
	 * Search for a subsequence with the given identifier and highlight it (and also it's parents).
	 * If expand is true, all sequences will be expanded so that the highlighted sequence is visible. 
	 * @param identifier Prolog identifier of the action to highlight
	 * @param expand expand all parent actions
	 * @return true if action found and highlighted
	 */
	public boolean highlightSubsequence(String identifier, boolean expand)
	{
		if (this.action.getIRI().compareTo(identifier)==0) {
			setHightlight(HighlightType.THIS_HIGHLIGHTED);
			notifyModified();
			return true;
		}
		
		//Check sequence of this action
		List<Action> sub = action.getSubActions();
		
		synchronized(sub) {
			for (Action a : sub) {

				ActionDrawInformation inf = a.getDrawInfo();

				if (a.getIRI().compareTo(identifier)==0) {
					inf.setHightlight(HighlightType.THIS_HIGHLIGHTED);
					setHightlight(HighlightType.CHILD_HIGHLIGHTED);
					notifyModified();
					return true;
					
				} else if (inf.highlightSubsequence(identifier, expand)) {
					inf.setHightlight(HighlightType.CHILD_HIGHLIGHTED);
					setHightlight(HighlightType.CHILD_HIGHLIGHTED);
					if (expand)
						action.setExpandedSequence(a);
					notifyModified();
					return true;
				}
			}
		}
		return false;
	}
	
	/**
	 * Search for the subsequence <code>seq</code> and highlight it (and also it's parents).
	 * If expand is true, all sequences will be expanded so that the highlighted sequence is visible. 
	 * @param seq The action to highlight
	 * @param expand expand all parent actions
	 * @return true if action found and highlighted
	 */
	public boolean highlightSubsequence(Action seq, boolean expand)
	{
		if (this.action == seq) {
			setHightlight(HighlightType.THIS_HIGHLIGHTED);
			return true;
		}
		
		//Check sequence of this action
		List<Action> sub = action.getSubActions();
		
		synchronized(sub) {
			for (Action a : sub) {

				ActionDrawInformation inf = a.getDrawInfo();

				if (a == seq)
				{
					inf.setHightlight(HighlightType.THIS_HIGHLIGHTED);
					setHightlight(HighlightType.CHILD_HIGHLIGHTED);
					return true;
				} else if (inf.highlightSubsequence(seq, expand))
				{
					inf.setHightlight(HighlightType.CHILD_HIGHLIGHTED);
					setHightlight(HighlightType.CHILD_HIGHLIGHTED);
					if (expand)
						action.setExpandedSequence(a);
					return true;
				}
			}
		}
		return false;
	}

	
	
	public Vector2f getOutboundConnectorPos() {
		
		// TODO: replace with better computation, taking extended status into account
		return new Vector2f(position.x + localPosOffset.x + globalPosOffset.x + this.getSimpleBoxDimension().x/2, position.y + localPosOffset.y + globalPosOffset.y + this.getSimpleBoxDimension().y);
	}

	public Vector2f getInboundConnectorPos() {
		// TODO: replace with better computation, taking extended status into account
		return new Vector2f(position.x + localPosOffset.x + globalPosOffset.x + this.getSimpleBoxDimension().x/2, position.y + localPosOffset.y + globalPosOffset.y);
	}
}
