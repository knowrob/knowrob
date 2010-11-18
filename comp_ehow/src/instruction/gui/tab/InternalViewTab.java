package instruction.gui.tab;

import instruction.gui.internal.PlanImporterWrapper;
import java.awt.BorderLayout;
import java.awt.ScrollPane;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextPane;

public class InternalViewTab extends InstructionTab {

	private static final long serialVersionUID = 3245495709892820570L;

	public static String TITLE = "Internal Data Structures";
	
	ScrollPane scroll = null;
	DataStructurePanel dsView = null;
	JTextPane text = null;
	JPanel buttonPanel = null;
	JButton next = null;
	JButton prev = null;
	JLabel instCount = null;

	public InternalViewTab() {
		initialize();
	}
	
	public void initialize() {

		setLayout( new BorderLayout() );

		dsView = new DataStructurePanel();
		dsView.init();
	//	text = new JTextPane();
		
	

		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent e ) {

				// TODO Auto-generated method stub

			}

			public void componentMoved( ComponentEvent e ) {

				// TODO Auto-generated method stub

			}

			public void componentResized( ComponentEvent e ) {

				// TODO Auto-generated method stub

			}

			public void componentShown( ComponentEvent e ) {

				dsView.setInstructions( PlanImporterWrapper.getImporter().getInstructions() );
				dsView.setActiveInstruction( 0 );
				dsView.redraw();
				instCount.setText( "Instruction No. " + (dsView.getActiveInstruction()+1) + " of " + dsView.getInstructionCount());
				
//				text.setText( "" );
//				List<Instruction> in = PlanImporterWrapper.getImporter().getInstructions();
//				for (Iterator<Instruction> i = in.iterator(); i.hasNext();) {
//					text.setText( text.getText() + "\n\n" + i.next().toString() );
//				}
			}

		} );
		
		buttonPanel = new JPanel();
		prev = new JButton("< Prev");
		prev.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				dsView.setActiveInstruction( (dsView.getActiveInstruction() - 1 < 0 ? dsView.getInstructionCount() - 1 : dsView.getActiveInstruction() - 1) % dsView.getInstructionCount() );
				dsView.redraw();
				instCount.setText( "Instruction No. " + (dsView.getActiveInstruction()+1) + " of " + dsView.getInstructionCount());
			}
			
		});
		next = new JButton("Next >");
		next.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				dsView.setActiveInstruction( (dsView.getActiveInstruction() + 1) % dsView.getInstructionCount());
				dsView.redraw();
				instCount.setText( "Instruction No. " + (dsView.getActiveInstruction()+1)  + " of " + dsView.getInstructionCount());
			}
			
		});
		
		instCount = new JLabel("");
		
		buttonPanel.add( prev );
		buttonPanel.add( instCount );
		buttonPanel.add( next );
		
		add(buttonPanel, BorderLayout.NORTH);

		scroll = new ScrollPane(  );
		scroll.add( dsView );
		
		add( scroll, BorderLayout.CENTER );

	}
	
}
