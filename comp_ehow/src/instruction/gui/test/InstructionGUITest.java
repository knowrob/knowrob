package instruction.gui.test;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import instruction.gui.EHowInstructionPanel;

import javax.swing.JFrame;

public class InstructionGUITest {

	private static EHowInstructionPanel panel = null;

	/**
	 * @param args
	 */
	public static void main( String[] args ) {
		
		JFrame window = new JFrame();
		window.setVisible( true );
		window.setSize( 1100, 800 );
		window.setTitle( "Plan Importer GUI" );
		window.setLocation( 400, 300 );
		window.setBackground(new Color(20, 20, 20));
		
		window.addWindowListener( new WindowAdapter() {
      public void windowClosing( WindowEvent we ) {
        System.exit( 0 );
      }
    } );

		panel = new EHowInstructionPanel();
		window.add( panel );
		panel.init();
		panel.revalidate();
	}

}
