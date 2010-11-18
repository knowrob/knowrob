package instruction.gui.tab;

import instruction.gui.internal.PlanImporterWrapper;
import instruction.wrapper.IHowtoWebsiteWrapper;

import java.awt.BorderLayout;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.io.File;
import javax.swing.JEditorPane;
import javax.swing.JScrollPane;

public class BrowserTab extends InstructionTab {

	private static final long serialVersionUID = 2094236267788921978L;

	public static String TITLE = "HTML View";

	private static String htmlContent = "<html><head></head><body><table width=\"100%\"><tr><td align=\"center\"><img src=\"file:///IMGSRC\"></td></tr></table></body></html>";
	private static String htmlContentError = "<html><head></head><body><table><tr><td align=\"center\">No Screenshot found for this Howto.</td></tr></table></body></html>";

	JEditorPane browser = null;
	JScrollPane scroll = null;

	public BrowserTab () {

		init();
	}

	@Override
	public void init() {

		setLayout( new BorderLayout() );

		browser = new JEditorPane();
		browser.setContentType( "text/html" );
		browser.setEditable( false );

		addComponentListener( new ComponentListener() {

			public void componentHidden( ComponentEvent arg0 ) {

			}

			public void componentMoved( ComponentEvent arg0 ) {

			}

			public void componentResized( ComponentEvent arg0 ) {

			}

			public void componentShown( ComponentEvent arg0 ) {

				IHowtoWebsiteWrapper wrapper = PlanImporterWrapper.getImporter().getWrapper();

				if ( wrapper == null )
					browser.setText( htmlContentError );
				else {
					String url = wrapper.getUrl();

					if ( url != null ) {

						File dummy = new File( url );
						if ( dummy.exists() ) {
							String screenshotFile = dummy.getAbsolutePath().replaceAll( "\\\\", "/" );
							screenshotFile += ".jpg";

							File screenshotDummy = new File( screenshotFile );
							if ( screenshotDummy.exists() ) {
					//			System.out.println( htmlContent.replaceAll( "IMGSRC", screenshotFile ) );
								browser.setText( htmlContent.replaceAll( "IMGSRC", screenshotFile ) );
							}
							else {
								browser.setText( htmlContentError );
							}
						}
					}

					else
						browser.setText( htmlContentError );
				}
			}

		} );

		scroll = new JScrollPane( browser );
		add( scroll, BorderLayout.CENTER );
	}
}
