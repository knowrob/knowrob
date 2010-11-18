package instruction.gui.tab;

import instruction.gui.internal.PlanImporterWrapper;

import java.awt.BorderLayout;
import java.awt.Font;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.util.Iterator;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.SwingUtilities;

public class CycViewTab extends JPanel {

	private static final long serialVersionUID = 6023046063791085728L;

	public static String TITLE = "Cyc Assertions";

	JScrollPane scroll = null;
	JTextPane text = null;

	public CycViewTab () {

		initialize();
	}

	public void initialize() {

		setLayout( new BorderLayout() );

		text = new JTextPane();
		text.setEditable( false );
		text.setBorder( BorderFactory.createLoweredBevelBorder() );
		text.setFont( new Font( "DialogInput", Font.PLAIN, 14 ) );
	//	text.setTabSize( 4 );

		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent e ) {
			}

			public void componentMoved( ComponentEvent e ) {
			}

			public void componentResized( ComponentEvent e ) {
			}

			public void componentShown( ComponentEvent e ) {

				List<String> assertions = PlanImporterWrapper.getImporter().getAssertions();
				text.setText( "" );
				if ( assertions == null || assertions.isEmpty() ) {
					text.setText( "No assertions to display." );
				}
				else {
					for ( Iterator<String> i = assertions.iterator(); i.hasNext(); ) {
						text.setText( text.getText() + (text.getText().isEmpty() ? "" : "\n") + i.next() );
					}
				}
				
				SwingUtilities.invokeLater( new Runnable() {
             public void run() {
            	 scroll.getVerticalScrollBar().setValue(0);
             }
         } );
			}

		} );

		scroll = new JScrollPane( text );
		add( scroll, BorderLayout.CENTER );
	}

}
