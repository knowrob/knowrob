package instruction.gui.tab;

import instruction.disambiguator.Disambiguator;
import instruction.configuration.ConfigurationManager;

import java.awt.GridBagLayout;
import javax.swing.JPanel;
import javax.swing.JTextField;
import java.awt.GridBagConstraints;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JRadioButton;
import java.awt.Insets;

public class ConfigurationTab extends JPanel implements DialogTab {

	private static final long serialVersionUID = 1L;
	private JTextField txtDisambiguator = null;
	private JButton btnBrowse2 = null;
	private JLabel jLabel3 = null;
	private JLabel jLabel31 = null;
	private JLabel jLabel6 = null;
	private JRadioButton jRadioButtonApply = null;
	private JRadioButton jRadioButtonTrain = null;
	private JLabel jLabel5 = null;
	private JTextField txtCycCache = null;
	private JButton btnBrowse3 = null;
	private JButton jButton = null;
	private JTextField jTextField = null;
	private JLabel jLabel = null;

	private ButtonGroup disambMode = new ButtonGroup();

	/**
	 * This is the default constructor
	 */
	public ConfigurationTab() {

		super();
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {

		GridBagConstraints gridBagConstraints11 = new GridBagConstraints();
		gridBagConstraints11.gridx = 0;
		gridBagConstraints11.anchor = GridBagConstraints.WEST;
		gridBagConstraints11.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints11.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints11.gridy = 0;
		jLabel = new JLabel();
		jLabel.setText("Path to local Howtos:");
		GridBagConstraints gridBagConstraints10 = new GridBagConstraints();
		gridBagConstraints10.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints10.gridy = 0;
		gridBagConstraints10.weightx = 1.0;
		gridBagConstraints10.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints10.gridx = 1;
		GridBagConstraints gridBagConstraints91 = new GridBagConstraints();
		gridBagConstraints91.gridx = 2;
		gridBagConstraints91.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints91.gridy = 0;
		GridBagConstraints gridBagConstraints9 = new GridBagConstraints();
		gridBagConstraints9.gridx = 2;
		gridBagConstraints9.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints9.gridy = 5;
		GridBagConstraints gridBagConstraints8 = new GridBagConstraints();
		gridBagConstraints8.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints8.gridy = 5;
		gridBagConstraints8.weightx = 1.0;
		gridBagConstraints8.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints8.gridx = 1;
		GridBagConstraints gridBagConstraints7 = new GridBagConstraints();
		gridBagConstraints7.gridx = 0;
		gridBagConstraints7.anchor = GridBagConstraints.WEST;
		gridBagConstraints7.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints7.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints7.gridy = 5;
		jLabel5 = new JLabel();
		jLabel5.setText("Cyc Cache (optional):");
		GridBagConstraints gridBagConstraints6 = new GridBagConstraints();
		gridBagConstraints6.gridx = 1;
		gridBagConstraints6.anchor = GridBagConstraints.WEST;
		gridBagConstraints6.gridy = 4;
		GridBagConstraints gridBagConstraints5 = new GridBagConstraints();
		gridBagConstraints5.gridx = 1;
		gridBagConstraints5.anchor = GridBagConstraints.WEST;
		gridBagConstraints5.gridy = 3;
		GridBagConstraints gridBagConstraints4 = new GridBagConstraints();
		gridBagConstraints4.gridx = 0;
		gridBagConstraints4.anchor = GridBagConstraints.WEST;
		gridBagConstraints4.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints4.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints4.gridy = 3;
		jLabel6 = new JLabel();
		jLabel6.setText("Disambiguator Mode:");
		GridBagConstraints gridBagConstraints3 = new GridBagConstraints();
		gridBagConstraints3.gridx = 1;
		gridBagConstraints3.gridy = 2;
		jLabel31 = new JLabel();
		jLabel31.setText("");
		GridBagConstraints gridBagConstraints2 = new GridBagConstraints();
		gridBagConstraints2.gridx = 0;
		gridBagConstraints2.anchor = GridBagConstraints.WEST;
		gridBagConstraints2.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints2.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints2.gridy = 1;
		jLabel3 = new JLabel();
		jLabel3.setText("Disambiguator:");
		GridBagConstraints gridBagConstraints1 = new GridBagConstraints();
		gridBagConstraints1.gridx = 2;
		gridBagConstraints1.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints1.gridy = 1;
		GridBagConstraints gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.fill = GridBagConstraints.HORIZONTAL;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.weightx = 1.0;
		gridBagConstraints.insets = new Insets(5, 5, 5, 5);
		gridBagConstraints.gridx = 1;
		this.setSize(693, 295);
		this.setLayout(new GridBagLayout());
		this.add(getTxtDisambiguator(), gridBagConstraints);
		this.add(getBtnBrowse2(), gridBagConstraints1);
		this.add(jLabel3, gridBagConstraints2);
		this.add(jLabel31, gridBagConstraints3);
		this.add(jLabel6, gridBagConstraints4);
		this.add(getJRadioButton(), gridBagConstraints5);
		this.add(getJRadioButton1(), gridBagConstraints6);
		disambMode.add(getJRadioButton());
		disambMode.add(getJRadioButton1());
		if (ConfigurationManager.getDisambiguatorMode() == Disambiguator.MODE_APPLY)
			getJRadioButton().setSelected(true);
		else
			getJRadioButton1().setSelected(true);
		this.add(jLabel5, gridBagConstraints7);
		this.add(getTxtCycCache(), gridBagConstraints8);
		this.add(getBtnBrowse3(), gridBagConstraints9);
		this.add(getJButton(), gridBagConstraints91);
		this.add(getJTextField(), gridBagConstraints10);
		this.add(jLabel, gridBagConstraints11);
	}

	/**
	 * This method initializes txtDisambiguator
	 * 
	 * @return javax.swing.JTextField
	 */
	private JTextField getTxtDisambiguator() {

		if (txtDisambiguator == null) {
			txtDisambiguator = new JTextField();
			txtDisambiguator.setText(ConfigurationManager
					.getPathDisambiguator() == null ? "" : ConfigurationManager
					.getPathDisambiguator());
		}
		return txtDisambiguator;
	}

	/**
	 * This method initializes btnBrowse2
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getBtnBrowse2() {

		if (btnBrowse2 == null) {
			btnBrowse2 = new JButton();
			btnBrowse2.setText("Browse...");
			btnBrowse2.addActionListener(new java.awt.event.ActionListener() {
				public void actionPerformed(java.awt.event.ActionEvent e) {
					JFileChooser dlg = new JFileChooser();
					if (dlg.showOpenDialog(ConfigurationTab.this) == JFileChooser.APPROVE_OPTION) {
						getTxtDisambiguator().setText(
								dlg.getSelectedFile().getAbsolutePath());
					}
				}
			});
		}
		return btnBrowse2;
	}

	/**
	 * This method initializes jRadioButton
	 * 
	 * @return javax.swing.JRadioButton
	 */
	private JRadioButton getJRadioButton() {

		if (jRadioButtonApply == null) {
			jRadioButtonApply = new JRadioButton();
			jRadioButtonApply.setText("Apply");
		}
		return jRadioButtonApply;
	}

	/**
	 * This method initializes jRadioButton1
	 * 
	 * @return javax.swing.JRadioButton
	 */
	private JRadioButton getJRadioButton1() {

		if (jRadioButtonTrain == null) {
			jRadioButtonTrain = new JRadioButton();
			jRadioButtonTrain.setText("Train");
		}
		return jRadioButtonTrain;
	}

	/**
	 * This method initializes txtCycCache
	 * 
	 * @return javax.swing.JTextField
	 */
	private JTextField getTxtCycCache() {

		if (txtCycCache == null) {
			txtCycCache = new JTextField();
			txtCycCache
					.setText(ConfigurationManager.getPathCycCache() == null ? ""
							: ConfigurationManager.getPathCycCache());
		}
		return txtCycCache;
	}

	/**
	 * This method initializes btnBrowse3
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getBtnBrowse3() {

		if (btnBrowse3 == null) {
			btnBrowse3 = new JButton();
			btnBrowse3.setText("Browse...");
		}
		return btnBrowse3;
	}

	public void onCancel() {

		// TODO Auto-generated method stub

	}

	public void onOK() {

		ConfigurationManager.setPathDisambiguator(txtDisambiguator.getText());
		ConfigurationManager.setPathCycCache(txtCycCache.getText());
		ConfigurationManager.setPathHowtos(jTextField.getText());
		ConfigurationManager.setDisambiguatorMode(getJRadioButton()
				.isSelected() ? Disambiguator.MODE_APPLY
				: Disambiguator.MODE_TRAIN);
	}

	/**
	 * This method initializes jButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButton() {

		if (jButton == null) {
			jButton = new JButton();
			jButton.setText("Browse...");
			jButton.addActionListener(new java.awt.event.ActionListener() {
				public void actionPerformed(java.awt.event.ActionEvent e) {
					JFileChooser dlg = new JFileChooser();
					dlg.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
					if (dlg.showOpenDialog(ConfigurationTab.this) == JFileChooser.APPROVE_OPTION) {
						getJTextField().setText(
								dlg.getSelectedFile().getAbsolutePath());
					}
				}
			});

		}
		return jButton;
	}

	/**
	 * This method initializes jTextField
	 * 
	 * @return javax.swing.JTextField
	 */
	private JTextField getJTextField() {

		if (jTextField == null) {
			jTextField = new JTextField();
			jTextField
					.setText(ConfigurationManager.getPathHowtos() == null ? ""
							: ConfigurationManager.getPathHowtos());
		}
		return jTextField;
	}

} // @jve:decl-index=0:visual-constraint="162,44"
