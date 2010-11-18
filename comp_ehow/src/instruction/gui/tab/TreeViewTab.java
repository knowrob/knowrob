package instruction.gui.tab;

import instruction.gui.internal.PlanImporterWrapper;
import instruction.syntaxparser.SyntaxTree;

import java.awt.BorderLayout;
import java.awt.ScrollPane;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.util.List;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;


public class TreeViewTab extends JPanel {

	private static final long serialVersionUID = 5838455112176213755L;

	public static String TITLE = "Syntax Tree View";

	ScrollPane scroll = null;
	SyntaxTreePanel treeView = null;
	JPanel buttonPanel = null;
	JButton next = null;
	JButton prev = null;
	JLabel instCount = null;

	public TreeViewTab () {

		initialize();
	}

	public void initialize() {

		setLayout( new BorderLayout() );

		treeView = new SyntaxTreePanel();
		treeView.init();
		
		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent e ) {
			}

			public void componentMoved( ComponentEvent e ) {
			}

			public void componentResized( ComponentEvent e ) {
			}
			

			public void componentShown( ComponentEvent e ) {

				List<SyntaxTree> trees = PlanImporterWrapper.getImporter().getSyntaxTrees();
				
				if (trees == null || trees.isEmpty())
					return;
				
			
				treeView.setSyntaxTree( trees );
				treeView.setActiveTree( 0 );
				treeView.redraw();
				instCount.setText( "Parse No. " + (treeView.getActiveTree()+1) + " of " + treeView.getTreeCount() );

			}

		} );

		scroll = new ScrollPane(  );
		
		scroll.add( treeView );
		
		buttonPanel = new JPanel();
		prev = new JButton("< Prev");
		prev.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				treeView.setActiveTree( (treeView.getActiveTree() - 1 < 0 ? treeView.getTreeCount() - 1 : treeView.getActiveTree() - 1) % treeView.getTreeCount() );
				treeView.redraw();
				instCount.setText( "Parse No. " + (treeView.getActiveTree()+1) + " of " + treeView.getTreeCount());
			}
			
		});
		next = new JButton("Next >");
		next.addActionListener( new ActionListener() {

			public void actionPerformed( ActionEvent e ) {
				treeView.setActiveTree( (treeView.getActiveTree() + 1) % treeView.getTreeCount());
				treeView.redraw();
				instCount.setText( "Parse No. " + (treeView.getActiveTree()+1)  + " of " + treeView.getTreeCount());
			}
			
		});
		
		instCount = new JLabel("");
		
		buttonPanel.add( prev );
		buttonPanel.add( instCount );
		buttonPanel.add( next );
		
		add(buttonPanel, BorderLayout.NORTH);
		
		add( scroll, BorderLayout.CENTER );
	}

}
