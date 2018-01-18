package org.knowrob.vis;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.ros.message.Duration;
import org.ros.message.Time;

import data_vis_msgs.DataVis;
import data_vis_msgs.ValueList;

/**
 * Wrapper class for data_vis messages.
 * Note: JPL has a bug using the getter/setter methods of the message in Prolog
 * (see: https://groups.google.com/forum/#!topic/swi-prolog/0ZH-eTDadh8),
 * using this wrapper class this problem can be avoided.
 * 
 * @author Daniel Beßler
 */
public class DataVisObject {
	private final String identifier;
	private final DataVis dataVisMsg;
	private final DataVisPublisher publisher;
	
	boolean isPublished = false;
	
	public DataVisObject(String identifier, DataVis dataVisMsg, DataVisPublisher publisher) {
		this.dataVisMsg = dataVisMsg;
		this.publisher = publisher;
		this.identifier = identifier;
	}

	public String getIdentifier() {
		return identifier;
	}
	
	public String getTitle() {
		return dataVisMsg.getTitle();
	}
	
	public void setTitle(String value) {
		dataVisMsg.setTitle(value);
		queueRepublish();
	}
	
	public String getXlabel() {
		return dataVisMsg.getXlabel();
	}
	
	public void setXlabel(String value) {
		dataVisMsg.setXlabel(value);
		queueRepublish();
	}
	
	public String getYlabel() {
		return dataVisMsg.getXlabel();
	}
	
	public void setYlabel(String value) {
		dataVisMsg.setYlabel(value);
		queueRepublish();
	}
	
	public int getWidth() {
		return dataVisMsg.getWidth();
	}
	
	public void setWidth(int value) {
		dataVisMsg.setWidth(value);
		queueRepublish();
	}
	
	public int getHeight() {
		return dataVisMsg.getHeight();
	}
	
	public void setHeight(int value) {
		dataVisMsg.setHeight(value);
		queueRepublish();
	}
	
	public String getFontsize() {
		return dataVisMsg.getFontsize();
	}
	
	public void setFontsize(String value) {
		dataVisMsg.setFontsize(value);
		queueRepublish();
	}
	
	public void setData(String[][][] values, String[] labels) {
		if(labels!=null && labels.length != values.length) {
			System.err.println("ERROR: Number of labels and data rows differs.");
		}
		for(int i =0; i< values.length; i++) {
			String[][] val_list = values[i];
			ValueList v = publisher.node.getTopicMessageFactory().newFromType(data_vis_msgs.ValueList._TYPE);
			if(labels != null) v.setLabel(labels[i]);
			v.setValue1(new ArrayList<String>(Arrays.asList(val_list[0])));
			v.setValue2(new ArrayList<String>(Arrays.asList(val_list[1])));
			dataVisMsg.getValues().add(v);
		}
	}
	
	public void setData(String[][][] values) {
		for(int i =0; i< values.length; i++) {
			String[][] val_list = values[i];
			ValueList v = publisher.node.getTopicMessageFactory().newFromType(data_vis_msgs.ValueList._TYPE);
			v.setValue1(new ArrayList<String>(Arrays.asList(val_list[0])));
			v.setValue2(new ArrayList<String>(Arrays.asList(val_list[1])));
			dataVisMsg.getValues().add(v);
		}
	}

	public DataVis getMessage() {
		return dataVisMsg;
	}

	private void queueRepublish() {
		publisher.queueRepublish(this);
	}

	private void unqueueRepublish() {
		publisher.unqueueRepublish(this);
	}
}
