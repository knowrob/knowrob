package instruction.gui.tab;

import instruction.configuration.ConfigurationManager;

import java.awt.GridBagLayout;
import javax.swing.JPanel;
import javax.swing.JLabel;
import java.awt.GridBagConstraints;
import java.awt.Insets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JButton;
import javax.swing.table.AbstractTableModel;
import javax.swing.table.TableModel;

public class CycMappingTab extends JPanel implements DialogTab {

	private static final long serialVersionUID = 1L;
	private JLabel jLabel = null;
	private JPanel jPanel = null;
	private JScrollPane jScrollPane = null;
	private JTable jTable = null;
	private JButton jButton = null;
	private JButton jButton1 = null;
	private JPanel jPanel1 = null;

	/**
	 * This is the default constructor
	 */
	public CycMappingTab () {

		super();
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {

		GridBagConstraints gridBagConstraints6 = new GridBagConstraints();
		gridBagConstraints6.gridx = 0;
		gridBagConstraints6.insets = new Insets( 5, 5, 5, 5 );
		gridBagConstraints6.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints6.gridy = 3;
		GridBagConstraints gridBagConstraints2 = new GridBagConstraints();
		gridBagConstraints2.fill = GridBagConstraints.BOTH;
		gridBagConstraints2.gridy = 2;
		gridBagConstraints2.weightx = 1.0;
		gridBagConstraints2.weighty = 1.0;
		gridBagConstraints2.gridx = 0;
		gridBagConstraints2.insets = new Insets( 5, 5, 5, 5 );
		GridBagConstraints gridBagConstraints1 = new GridBagConstraints();
		gridBagConstraints1.gridx = 0;
		gridBagConstraints1.gridy = 1;
		GridBagConstraints gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.insets = new Insets( 5, 5, 5, 5 );
		gridBagConstraints.anchor = GridBagConstraints.WEST;
		gridBagConstraints.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints.gridy = 0;
		jLabel = new JLabel();
		jLabel.setText( "The follwing mappings from WordNet Synset-IDs to Cyc concepts are added." );
		this.setSize( 510, 349 );
		this.setLayout( new GridBagLayout() );
		this.add( jLabel, gridBagConstraints );
		this.add( getJPanel(), gridBagConstraints1 );
		this.add( getJScrollPane(), gridBagConstraints2 );
		this.add( getJPanel1(), gridBagConstraints6 );
	}

	public void onCancel() {

	}

	public void onOK() {

		Map<String, List<String>> newMappings = new HashMap<String, List<String>>();
		
		for ( int i = 0; i < getJTable().getRowCount(); i++ ) {
			TableModel model = getJTable().getModel();
			String synset = (String) model.getValueAt( i, 0 );
			String concept = (String) model.getValueAt( i, 1 );
			List<String> concepts = newMappings.get( synset );
			if (concepts == null) {
				concepts = new ArrayList<String>();
				newMappings.put( synset, concepts );
			}
			concepts.add( concept );
			
			ConfigurationManager.setMappings( newMappings );
		}

	}

	/**
	 * This method initializes jPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel() {

		if ( jPanel == null ) {
			jPanel = new JPanel();
			jPanel.setLayout( new GridBagLayout() );

		}
		return jPanel;
	}

	/**
	 * This method initializes jScrollPane
	 * 
	 * @return javax.swing.JScrollPane
	 */
	private JScrollPane getJScrollPane() {

		if ( jScrollPane == null ) {
			jScrollPane = new JScrollPane();
			jScrollPane.setViewportView( getJTable() );
		}
		return jScrollPane;
	}

	/**
	 * This method initializes jTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getJTable() {

		if ( jTable == null ) {

			jTable = new JTable( new AbstractTableModel() {
				private static final long serialVersionUID = -7702568498509726688L;

				String[] columns = {
						"Synset-ID", "Cyc concept"
				};

				List<Object[]> data = new ArrayList<Object[]>();

				@Override
				public String getColumnName( int column ) {

					return columns[column];
				}

				public int getColumnCount() {

					return columns.length;
				}

				public int getRowCount() {

					return data.size();
				}

				public Object getValueAt( int row, int col ) {

					return data.get( row )[col];
				}

				@Override
				public void setValueAt( Object value, int rowIndex, int columnIndex ) {

					if ( rowIndex > getRowCount() - 1 ) {
						Object[] newData = new Object[columns.length];
						newData[columnIndex] = value;
						data.add( newData );
					}
					else {
						if ( value == null )
							data.remove( rowIndex );
						else
							data.get( rowIndex )[columnIndex] = value;
					}
					fireTableDataChanged();
				}

				@Override
				public boolean isCellEditable( int rowIndex, int columnIndex ) {

					return true;
				}

			} );

			Map<String, List<String>> mappings = ConfigurationManager.getMappings();
			Set<String> synsets = mappings.keySet();
			for ( Iterator<String> i = synsets.iterator(); i.hasNext(); ) {
				String synset = i.next();
				List<String> concepts = mappings.get( synset );
				for ( Iterator<String> j = concepts.iterator(); j.hasNext(); ) {
					int row = jTable.getRowCount();
					jTable.setValueAt( synset, row, 0 );
					jTable.setValueAt( j.next(), row, 1 );
				}
			}
		}
		return jTable;
	}

	/**
	 * This method initializes jButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButton() {

		if ( jButton == null ) {
			jButton = new JButton();
			jButton.setText( "Add Mapping" );
			jButton.addActionListener( new java.awt.event.ActionListener() {
				public void actionPerformed( java.awt.event.ActionEvent e ) {

					int row = getJTable().getModel().getRowCount();
					getJTable().getModel().setValueAt( "Synset-ID", row, 0 );
					getJTable().getModel().setValueAt( "Concept", row, 1 );
				}
			} );
		}
		return jButton;
	}

	/**
	 * This method initializes jButton1
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButton1() {

		if ( jButton1 == null ) {
			jButton1 = new JButton();
			jButton1.setText( "Remove" );
			jButton1.addActionListener( new java.awt.event.ActionListener() {
				public void actionPerformed( java.awt.event.ActionEvent e ) {

					int selIndex = getJTable().getSelectedRow();
					if ( selIndex >= 0 )
						getJTable().getModel().setValueAt( null, selIndex, 0 );
				}
			} );

		}
		return jButton1;
	}

	/**
	 * This method initializes jPanel1
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel1() {

		if ( jPanel1 == null ) {
			GridBagConstraints gridBagConstraints4 = new GridBagConstraints();
			gridBagConstraints4.gridx = 1;
			gridBagConstraints4.insets = new Insets( 5, 5, 5, 5 );
			gridBagConstraints4.gridy = 0;
			GridBagConstraints gridBagConstraints3 = new GridBagConstraints();
			gridBagConstraints3.gridx = 0;
			gridBagConstraints3.insets = new Insets( 5, 5, 5, 5 );
			gridBagConstraints3.gridy = 0;
			jPanel1 = new JPanel();
			jPanel1.setLayout( new GridBagLayout() );
			jPanel1.add( getJButton(), gridBagConstraints3 );
			jPanel1.add( getJButton1(), gridBagConstraints4 );
		}
		return jPanel1;
	}

} // @jve:decl-index=0:visual-constraint="10,10"
