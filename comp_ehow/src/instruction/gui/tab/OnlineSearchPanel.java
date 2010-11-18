package instruction.gui.tab;

import java.awt.GridBagLayout;
import javax.swing.JPanel;
import javax.swing.JTextField;
import java.awt.GridBagConstraints;
import javax.swing.JList;
import javax.swing.JButton;
import java.awt.Insets;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JLabel;
import javax.swing.JRadioButton;

import edu.stanford.nlp.web.HTMLParser;

public class OnlineSearchPanel extends JPanel {

	public static String TITLE = "Online Search";
	private static final long serialVersionUID = 1L;
	private JTextField commandLine = null;
	private JList howtos = null;
	private JButton jButton = null;
	private JPanel jPanel = null;
	private JLabel jLabel = null;
	private JLabel jLabel1 = null;
	private JRadioButton jRadioButton = null;
	private JRadioButton jRadioButton1 = null;

	/**
	 * This is the default constructor
	 */
	public OnlineSearchPanel () {

		super();
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {

		GridBagConstraints gridBagConstraints4 = new GridBagConstraints();
		gridBagConstraints4.gridx = 1;
		gridBagConstraints4.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints4.gridy = 1;
		GridBagConstraints gridBagConstraints31 = new GridBagConstraints();
		gridBagConstraints31.gridx = 1;
		gridBagConstraints31.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints31.gridy = 0;
		GridBagConstraints gridBagConstraints21 = new GridBagConstraints();
		gridBagConstraints21.gridx = 0;
		gridBagConstraints21.anchor = GridBagConstraints.WEST;
		gridBagConstraints21.insets = new Insets( 0, 10, 0, 0 );
		gridBagConstraints21.gridwidth = 1;
		gridBagConstraints21.gridy = 1;
		jLabel1 = new JLabel();
		jLabel1.setText( "for executing the task:" );
		GridBagConstraints gridBagConstraints11 = new GridBagConstraints();
		gridBagConstraints11.gridx = 0;
		gridBagConstraints11.insets = new Insets( 10, 10, 0, 10 );
		gridBagConstraints11.gridwidth = 1;
		gridBagConstraints11.anchor = GridBagConstraints.WEST;
		gridBagConstraints11.gridy = 0;
		jLabel = new JLabel();
		jLabel.setText( "Type in a task for the robot. The system will automatically look up an Instruction" );
		GridBagConstraints gridBagConstraints3 = new GridBagConstraints();
		gridBagConstraints3.gridx = 2;
		gridBagConstraints3.gridy = 3;
		GridBagConstraints gridBagConstraints2 = new GridBagConstraints();
		gridBagConstraints2.gridx = 1;
		gridBagConstraints2.insets = new Insets( 10, 10, 10, 10 );
		gridBagConstraints2.gridy = 2;
		GridBagConstraints gridBagConstraints1 = new GridBagConstraints();
		gridBagConstraints1.fill = GridBagConstraints.BOTH;
		gridBagConstraints1.gridy = 3;
		gridBagConstraints1.weightx = 1.0;
		gridBagConstraints1.weighty = 1.0;
		gridBagConstraints1.gridheight = 1;
		gridBagConstraints1.gridwidth = 2;
		gridBagConstraints1.ipadx = 50;
		gridBagConstraints1.ipady = 50;
		gridBagConstraints1.anchor = GridBagConstraints.CENTER;
		gridBagConstraints1.insets = new Insets( 10, 10, 10, 10 );
		gridBagConstraints1.gridx = 0;
		GridBagConstraints gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.weightx = 1.0;
		gridBagConstraints.insets = new Insets( 10, 10, 10, 10 );
		gridBagConstraints.gridx = 0;
		this.setSize( 569, 319 );
		this.setLayout( new GridBagLayout() );
		this.add( getJTextField(), gridBagConstraints );
		this.add( getJList(), gridBagConstraints1 );
		this.add( getJButton(), gridBagConstraints2 );
		this.add( getJPanel(), gridBagConstraints3 );
		this.add( jLabel, gridBagConstraints11 );
		this.add( jLabel1, gridBagConstraints21 );
		this.add( getJRadioButton(), gridBagConstraints31 );
		this.add( getJRadioButton1(), gridBagConstraints4 );
	}

	/**
	 * This method initializes jTextField
	 * 
	 * @return javax.swing.JTextField
	 */
	private JTextField getJTextField() {

		if ( commandLine == null ) {
			commandLine = new JTextField();
			commandLine.setText( "set a table" );
		}
		return commandLine;
	}

	/**
	 * This method initializes jList
	 * 
	 * @return javax.swing.JList
	 */
	private JList getJList() {

		if ( howtos == null ) {
			howtos = new JList();
		}
		return howtos;
	}

	/**
	 * This method initializes jButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButton() {

		if ( jButton == null ) {
			jButton = new JButton();
			jButton.setText( "Search!" );
			jButton.addActionListener( new java.awt.event.ActionListener() {
				public void actionPerformed( java.awt.event.ActionEvent e ) {

					URL url;
					try {
						url = new URL( "http://www.ehow.com/search.aspx?s=" + commandLine.getText().replaceAll( " ", "+" )
								+ "&Options=0" );
						InputStream is = url.openStream();
						Reader reader = new BufferedReader( new InputStreamReader( is ) );
						StringBuilder answer = new StringBuilder();
						char[] buf = new char[1024];
						int read;
						while ( ( read = reader.read( buf ) ) > 0 ) {
							answer.append( buf, 0, read );
						}

						HTMLParser parser = new HTMLParser();
						
						
						List<String> inst = new ArrayList<String>();
						String[] tokens = parser.parse( answer.toString() ).split( "\\|" );
						for (int i = 0; i < tokens.length; i++) {
							System.out.println(tokens[i]);
							int start = -1;
							int end = -1;
							start = tokens[i].indexOf( "How to" );
							if (start == -1)
								continue;
							else {
								end = tokens[i].indexOf( ".", start + 1 );
								if (end == -1)
									continue;
								inst.add( tokens[i].substring( start, end ) );
							}
							
						}
						howtos.setListData( inst.toArray(new String[0]) );

					}
					catch ( MalformedURLException e1 ) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					catch ( IOException e1 ) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					

				}
			} );
		}
		return jButton;
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
			jPanel.setSize( 50, 0 );
		}
		return jPanel;
	}

	/**
	 * This method initializes jRadioButton
	 * 
	 * @return javax.swing.JRadioButton
	 */
	private JRadioButton getJRadioButton() {

		if ( jRadioButton == null ) {
			jRadioButton = new JRadioButton();
			jRadioButton.setText( "ehow.com" );
		}
		return jRadioButton;
	}

	/**
	 * This method initializes jRadioButton1
	 * 
	 * @return javax.swing.JRadioButton
	 */
	private JRadioButton getJRadioButton1() {

		if ( jRadioButton1 == null ) {
			jRadioButton1 = new JRadioButton();
			jRadioButton1.setText( "wikihow.com" );
		}
		return jRadioButton1;
	}

} // @jve:decl-index=0:visual-constraint="10,10"
