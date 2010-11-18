package instruction.gui.tab;

import instruction.gui.dialog.ConfigDlg;
import instruction.configuration.ConfigurationManager;
import java.awt.Dimension;
import javax.swing.JFileChooser;
import javax.swing.JTextField;
import javax.swing.JButton;
import javax.swing.JPanel;
import java.awt.GridBagLayout;
import javax.swing.JLabel;
import java.awt.GridBagConstraints;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

public class SearchViewTab extends InstructionTab {

	private static final long serialVersionUID = -165573500527827064L;

	public static String TITLE = "Load a Howto";

	private JPanel jPanel = null;
	private JLabel jLabel = null;
	private JTextField txtPath = null;
	private JButton btnBrowse = null;

	private JPanel jPanel1 = null;

	private JButton jConfigureButton = null;

	private JLabel jLabel1 = null;

	private JLabel jLabel11 = null;

	private JPanel jPanel2 = null;

	private JPanel jPanel3 = null;

	/**
	 * This method initializes
	 * 
	 */
	public SearchViewTab () {

		super();
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 */
	private void initialize() {

		GridBagConstraints gridBagConstraints15 = new GridBagConstraints();
		gridBagConstraints15.gridx = 4;
		gridBagConstraints15.fill = GridBagConstraints.BOTH;
		gridBagConstraints15.gridheight = 10;
		gridBagConstraints15.gridy = 0;
		GridBagConstraints gridBagConstraints14 = new GridBagConstraints();
		gridBagConstraints14.gridx = 0;
		gridBagConstraints14.gridheight = 10;
		gridBagConstraints14.fill = GridBagConstraints.BOTH;
		gridBagConstraints14.gridy = 0;
		GridBagConstraints gridBagConstraints13 = new GridBagConstraints();
		gridBagConstraints13.gridx = 2;
		gridBagConstraints13.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints13.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints13.anchor = GridBagConstraints.WEST;
		gridBagConstraints13.gridy = 1;
		jLabel11 = new JLabel();
		jLabel11.setText("In the latter case the system will look in the Howto path selected in the \"Settings\" dialog.");
		GridBagConstraints gridBagConstraints121 = new GridBagConstraints();
		gridBagConstraints121.gridx = 2;
		gridBagConstraints121.anchor = GridBagConstraints.WEST;
		gridBagConstraints121.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints121.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints121.gridy = 0;
		jLabel1 = new JLabel();
		jLabel1.setText("Please select either a path to a Howto file or just type the title of a Howto (e.g. \"set a table\").");
		GridBagConstraints gridBagConstraints12 = new GridBagConstraints();
		gridBagConstraints12.gridx = 3;
		gridBagConstraints12.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints12.fill = GridBagConstraints.NONE;
		gridBagConstraints12.gridy = 9;
		GridBagConstraints gridBagConstraints31 = new GridBagConstraints();
		gridBagConstraints31.gridx = 3;
		gridBagConstraints31.gridy = 6;
		GridBagConstraints gridBagConstraints21 = new GridBagConstraints();
		gridBagConstraints21.gridx = 1;
		gridBagConstraints21.gridy = 4;
		GridBagConstraints gridBagConstraints3 = new GridBagConstraints();
		gridBagConstraints3.insets = new Insets( 5, 5, 5, 5 );
		gridBagConstraints3.gridx = 3;
		gridBagConstraints3.gridy = 3;
		gridBagConstraints3.gridwidth = 1;
		gridBagConstraints3.anchor = GridBagConstraints.NORTH;
		gridBagConstraints3.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints3.gridheight = 1;
		GridBagConstraints gridBagConstraints2 = new GridBagConstraints();
		gridBagConstraints2.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints2.gridheight = 2;
		gridBagConstraints2.gridx = 2;
		gridBagConstraints2.gridy = 3;
		gridBagConstraints2.weightx = 1.0;
		gridBagConstraints2.anchor = GridBagConstraints.NORTH;
		gridBagConstraints2.insets = new Insets( 8, 5, 5, 5 );
		GridBagConstraints gridBagConstraints1 = new GridBagConstraints();
		gridBagConstraints1.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints1.gridx = 1;
		gridBagConstraints1.gridy = 3;
		gridBagConstraints1.anchor = GridBagConstraints.WEST;
		gridBagConstraints1.gridheight = 1;
		GridBagConstraints gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridheight = 0;
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 0;
		this.setLayout( new GridBagLayout() );
		this.setSize( new Dimension( 636, 237 ) );
		this.add( getJPanel(), gridBagConstraints );
		this.add(jLabel, gridBagConstraints1);
		this.add(getTxtPath(), gridBagConstraints2);
		this.add(getBtnBrowse(), gridBagConstraints3);
		this.add(getJPanel1(), gridBagConstraints31);
		this.add(getJConfigureButton(), gridBagConstraints12);
		this.add(jLabel1, gridBagConstraints121);
		this.add(jLabel11, gridBagConstraints13);
		this.add(getJPanel2(), gridBagConstraints14);
		this.add(getJPanel3(), gridBagConstraints15);
		
	}

	/**
	 * This method initializes jPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel() {

		if ( jPanel == null ) {
			jLabel = new JLabel();
			jLabel.setText( "Howto:" );
			jPanel = new JPanel();
			jPanel.setLayout( new GridBagLayout() );
		}
		return jPanel;
	}

	/**
	 * This method initializes txtPath
	 * 
	 * @return javax.swing.JTextField
	 */
	private JTextField getTxtPath() {

		if ( txtPath == null ) {
			txtPath = new JTextField();
			txtPath.setText("make pancakes");
			txtPath.setPreferredSize( new Dimension( 200, 20 ) );
		}
		return txtPath;
	}

	/**
	 * This method initializes btnBrowse
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getBtnBrowse() {

		if ( btnBrowse == null ) {
			btnBrowse = new JButton();
			btnBrowse.setText( "Browse..." );

			btnBrowse.addActionListener( new ActionListener() {

				public void actionPerformed( ActionEvent e ) {

					JFileChooser dlg = new JFileChooser();
					dlg.setCurrentDirectory( new File( "." ) );
					int retVal = dlg.showOpenDialog( SearchViewTab.this );
					if ( retVal == JFileChooser.APPROVE_OPTION ) {
						getTxtPath().setText( dlg.getSelectedFile().getAbsolutePath() );

					}
				}

			} );
		}
		return btnBrowse;
	}

	/**
	 * This method initializes jPanel1
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel1() {

		if ( jPanel1 == null ) {
			jPanel1 = new JPanel();
			jPanel1.setLayout( new GridBagLayout() );
		}
		return jPanel1;
	}

	public String getHowtoPath() {

		return getTxtPath().getText();
	}

	/**
	 * This method initializes jConfigureButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJConfigureButton() {
	
		if ( jConfigureButton == null ) {
			jConfigureButton = new JButton();
			jConfigureButton.setText("Settings...");
			jConfigureButton.addActionListener( new java.awt.event.ActionListener() {
				public void actionPerformed( java.awt.event.ActionEvent e ) {
					ConfigDlg dlg = new ConfigDlg();
					dlg.setModal( true );
					dlg.setLocationRelativeTo( SearchViewTab.this );
					dlg.setVisible( true );
					ConfigurationManager.saveSettings();
				}
			} );
			
		}
		return jConfigureButton;
	}

	/**
	 * This method initializes jPanel2	
	 * 	
	 * @return javax.swing.JPanel	
	 */
	private JPanel getJPanel2() {
	
		if ( jPanel2 == null ) {
			jPanel2 = new JPanel();
			jPanel2.setLayout(new GridBagLayout());
			jPanel2.setMinimumSize( new Dimension(40, 10) );
			jPanel2.setMaximumSize( new Dimension(40, 10) );
			jPanel2.setPreferredSize( new Dimension(40, 10) );
		}
		return jPanel2;
	}

	/**
	 * This method initializes jPanel3	
	 * 	
	 * @return javax.swing.JPanel	
	 */
	private JPanel getJPanel3() {
	
		if ( jPanel3 == null ) {
			jPanel3 = new JPanel();
			jPanel3.setLayout(new GridBagLayout());
			jPanel3.setMinimumSize( new Dimension(40, 10) );
			jPanel3.setMaximumSize( new Dimension(40, 10) );
			jPanel3.setPreferredSize( new Dimension(40, 10) );
		}
		return jPanel3;
	}
}  //  @jve:decl-index=0:visual-constraint="97,81"
