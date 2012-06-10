package edu.tum.cs.ias.knowrob.comp_barcoo;

import java.util.LinkedList;


public class MyNode 
{
	
	private boolean textNode;
	private String value;
	private LinkedList<MyNode> child;
	private int index;
	
	public MyNode(boolean textNode, String value)
	{
		index = 0;
		this.textNode = textNode;
		this.value = value;
		if(!textNode)
			child = new LinkedList<MyNode>();
	}
	
	public void addChild(MyNode node) throws Exception
	{
		if(textNode)
			throw new Exception("Cannot add child to TextNode");
		else
			child.add(node);
	}
	
	public MyNode getNext()
	{
		index++;
		return child.get(index - 1);		
	}
	
	public void resetCursor()
	{
		index = 0;
		if(child != null)
		{
			for(int i = 0;i < child.size(); i++)
			{
				resetChild(child.get(i));
			}
		}
	}
	private void resetChild(MyNode n)
	{
		n.index = 0;
		if(n.child != null)
		{
			for(int i = 0; i < n.child.size(); i++)
				resetChild(n.child.get(i));
		}
	}
	
	public boolean hasNext()
	{
		if(textNode || index == child.size())
			return false;
		return true;
	}
	
	public boolean isTextNode()
	{
		return textNode;
	}
	
	public String getValue()
	{
		return value;
	}
}
