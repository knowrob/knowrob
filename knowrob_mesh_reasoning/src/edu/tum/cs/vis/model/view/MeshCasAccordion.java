/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.DihedralAngleSegmentationAnnotation;
import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.view.control.AnnotationPanel;
import edu.tum.cs.vis.model.view.control.DihedralAngleSegmentationAnnotationPanel;
import edu.tum.cs.vis.model.view.control.FlatSurfaceAnnotationPanel;

/**
 * Accordion like component. Shows a list of buttons where you can expand the content.
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshCasAccordion extends JPanel implements ActionListener {
	/**
	 * Internal class that maintains information about individual Outlook bars; specifically it
	 * maintains the following information:
	 * 
	 * name The name of the bar button The associated JButton for the bar component The component
	 * maintained in the Outlook bar
	 */
	class BarInfo implements ActionListener {

		/**
		 * Cas which holds list of annotations
		 */
		protected MeshCas								cas;

		/**
		 * Type of annotations in this
		 */
		private final Class<? extends MeshAnnotation>	annotationType;

		/**
		 * The JButton that implements the Outlook bar itself
		 */
		private final JPanel							buttonPanel;

		/**
		 * Button for expanding panel
		 */
		private final JButton							button;

		/**
		 * Checkbox at left side of button
		 */
		private final JCheckBox							checkbox;

		/**
		 * The component that is the body of the Outlook bar
		 */
		@SuppressWarnings("rawtypes")
		final AnnotationPanel							component;

		/**
		 * Creates a new BarInfo
		 * 
		 * @param annotationType
		 *            Type of annotations which this bar holds
		 * @param component
		 *            The component that is the body of the Outlook Bar
		 * @param cas
		 *            Main CAS
		 */
		public BarInfo(Class<? extends MeshAnnotation> annotationType,
				@SuppressWarnings("rawtypes") AnnotationPanel component, MeshCas cas) {
			this.component = component;
			this.annotationType = annotationType;
			this.cas = cas;
			String name = annotationType.getName();
			if (name.endsWith("Annotation"))
				name = name.substring(0, name.lastIndexOf("Annotation"));
			if (name.lastIndexOf('.') > 0)
				name = name.substring(name.lastIndexOf('.') + 1);
			button = new JButton(name);
			button.setHorizontalAlignment(SwingConstants.LEFT);
			buttonPanel = new JPanel(new BorderLayout());
			checkbox = new JCheckBox("");
			checkbox.setSelected(true);
			checkbox.addActionListener(this);
			buttonPanel.add(checkbox, BorderLayout.WEST);
			buttonPanel.add(button, BorderLayout.CENTER);
		}

		@Override
		public void actionPerformed(ActionEvent arg0) {
			synchronized (cas.getAnnotations()) {
				for (Annotation m : cas.getAnnotations()) {
					if (m.getClass() != annotationType || !(m instanceof MeshAnnotation))
						continue;
					MeshAnnotation ma = (MeshAnnotation) m;
					ma.setDrawAnnotation(checkbox.isSelected());
				}
			}
		}

		/**
		 * @return the annotationType
		 */
		public Class<? extends MeshAnnotation> getAnnotationType() {
			return annotationType;
		}

		/**
		 * Returns the outlook bar JButton implementation
		 * 
		 * @return The Outlook Bar JButton implementation
		 */
		public JButton getButton() {
			return button;
		}

		/**
		 * Getting panel which holds button and checkbox.
		 * 
		 * @return panel
		 */
		public JPanel getButtonPanel() {
			return buttonPanel;
		}

		/**
		 * Returns the component that implements the body of this Outlook Bar
		 * 
		 * @return The component that implements the body of this Outlook Bar
		 */
		public JComponent getComponent() {
			return component;
		}

	}

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -2608007314705667190L;

	/**
	 * Creates the control panel for given class
	 * 
	 * @param clazz
	 *            AnnotationType for which to create panel
	 * @return the control panel
	 */
	@SuppressWarnings("rawtypes")
	static AnnotationPanel createPanelForAnnotation(Class<? extends MeshAnnotation> clazz,
			MeshCas cas) {
		if (clazz == FlatSurfaceAnnotation.class)
			return new FlatSurfaceAnnotationPanel(cas);
		else if (clazz == DihedralAngleSegmentationAnnotation.class)
			return new DihedralAngleSegmentationAnnotationPanel(cas);

		System.err
				.println("Update createPanelForAnnotation() in MeshCasAccordion.java for creating AnnotationPanel for "
						+ clazz);
		return null;
	}

	/**
	 * The top panel: contains the buttons displayed on the top of the JOutlookBar
	 */
	private final JPanel								topPanel			= new JPanel(
																					new GridLayout(
																							1, 1));

	/**
	 * The bottom panel: contains the buttons displayed on the bottom of the JOutlookBar
	 */
	private final JPanel								bottomPanel			= new JPanel(
																					new GridLayout(
																							1, 1));

	/**
	 * A LinkedHashMap of bars: we use a linked hash map to preserve the order of the bars
	 */
	final Map<Class<? extends MeshAnnotation>, BarInfo>	bars				= new LinkedHashMap<Class<? extends MeshAnnotation>, BarInfo>();

	/**
	 * The currently visible bar (zero-based index)
	 */
	private int											visibleBar			= 0;

	/**
	 * A place-holder for the currently visible component
	 */
	private JComponent									visibleComponent	= null;

	/**
	 * Creates a new JOutlookBar; after which you should make repeated calls to addBar() for each
	 * bar
	 * 
	 * @param cas
	 *            Cas which can be controlled
	 */
	public MeshCasAccordion(final MeshCas cas) {
		setLayout(new BorderLayout());
		this.add(topPanel, BorderLayout.NORTH);
		this.add(bottomPanel, BorderLayout.SOUTH);
		Timer tim = new Timer();
		tim.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
				synchronized (cas.getAnnotations()) {
					synchronized (cas.getAnnotations()) {
						for (Annotation a : cas.getAnnotations()) {
							if (a instanceof MeshAnnotation) {
								MeshAnnotation ma = (MeshAnnotation) a;

								if (bars.containsKey(ma.getClass()))
									continue;

								@SuppressWarnings("rawtypes")
								AnnotationPanel pnl = createPanelForAnnotation(ma.getClass(), cas);

								BarInfo bi = new BarInfo(ma.getClass(), pnl, cas);
								bars.put(ma.getClass(), bi);
							}
						}
					}

				}

				render();

			}
		}, 1000, 2000);
	}

	/**
	 * Invoked when one of our bars is selected
	 */
	@Override
	public void actionPerformed(ActionEvent e) {
		int currentBar = 0;
		for (Iterator<Class<? extends MeshAnnotation>> i = bars.keySet().iterator(); i.hasNext();) {
			Class<? extends MeshAnnotation> clazz = i.next();
			BarInfo barInfo = bars.get(clazz);
			if (barInfo.getButton() == e.getSource()) {
				// Found the selected button
				visibleBar = currentBar;
				render();
				return;
			}
			currentBar++;
		}
	}

	/**
	 * Returns the index of the currently visible bar (zero-based)
	 * 
	 * @return The index of the currently visible bar
	 */
	public int getVisibleBar() {
		return visibleBar;
	}

	/**
	 * Causes the outlook bar component to rebuild itself; this means that it rebuilds the top and
	 * bottom panels of bars as well as making the currently selected bar's panel visible
	 */
	public void render() {
		// Compute how many bars we are going to have where
		int totalBars = bars.size();
		if (totalBars == 0)
			return;

		int topBars = visibleBar + 1;
		int bottomBars = totalBars - topBars;

		// Get an iterator to walk through out bars with
		Iterator<Class<? extends MeshAnnotation>> itr = bars.keySet().iterator();

		// Render the top bars: remove all components, reset the GridLayout to
		// hold to correct number of bars, add the bars, and "validate" it to
		// cause it to re-layout its components
		topPanel.removeAll();
		GridLayout topLayout = (GridLayout) topPanel.getLayout();
		topLayout.setRows(topBars);
		BarInfo barInfo = null;
		for (int i = 0; i < topBars; i++) {
			Class<? extends MeshAnnotation> clazz = itr.next();
			barInfo = bars.get(clazz);
			topPanel.add(barInfo.getButtonPanel());
		}
		topPanel.validate();

		// Render the center component: remove the current component (if there
		// is one) and then put the visible component in the center of this panel
		if (visibleComponent != null) {
			this.remove(visibleComponent);
		}
		visibleComponent = barInfo.getComponent();
		this.add(visibleComponent, BorderLayout.CENTER);

		// Render the bottom bars: remove all components, reset the GridLayout to
		// hold to correct number of bars, add the bars, and "validate" it to
		// cause it to re-layout its components
		bottomPanel.removeAll();
		GridLayout bottomLayout = (GridLayout) bottomPanel.getLayout();
		bottomLayout.setRows(bottomBars);
		for (int i = 0; i < bottomBars; i++) {
			Class<? extends MeshAnnotation> clazz = itr.next();
			barInfo = bars.get(clazz);
			bottomPanel.add(barInfo.getButtonPanel());
		}
		bottomPanel.validate();

		// Validate all of our components: cause this container to re-layout its subcomponents
		validate();
	}

	/**
	 * Opens the panel connected with given annotation and shows the features of this annotation. if
	 * selectedAnnotation is null, feature infos will be reset.
	 * 
	 * @param selectedAnnotation
	 *            annotation to show or null if none
	 */
	@SuppressWarnings("unchecked")
	public void setSelectedAnnotation(final MeshAnnotation selectedAnnotation) {

		BarInfo selBar = null;

		for (Class<? extends MeshAnnotation> cl : bars.keySet()) {
			BarInfo i = bars.get(cl);
			if (selectedAnnotation != null && selectedAnnotation.getClass() == cl) {
				i.component.setSelected(selectedAnnotation);
				selBar = i;
			} else
				i.component.setSelected(null);
		}

		if (selBar != null) {

			int idx = 0;

			for (Iterator<Class<? extends MeshAnnotation>> itr = bars.keySet().iterator(); itr
					.hasNext(); idx++) {
				Class<? extends MeshAnnotation> clazz = itr.next();
				if (bars.get(clazz) == selBar)
					break;
			}

			setVisibleBar(idx);
		}
	}

	/**
	 * Programmatically sets the currently visible bar; the visible bar index must be in the range
	 * of 0 to size() - 1
	 * 
	 * @param visibleBar
	 *            The zero-based index of the component to make visible
	 */
	public void setVisibleBar(int visibleBar) {
		if (visibleBar > 0 && visibleBar < bars.size() - 1) {
			this.visibleBar = visibleBar;
			render();
		}
	}
}
