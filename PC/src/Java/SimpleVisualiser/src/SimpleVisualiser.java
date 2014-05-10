import org.lwjgl.LWJGLException;
import org.lwjgl.Sys;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import static org.lwjgl.opengl.GL11.*;


public class SimpleVisualiser {
	private static int Width = 100;
	private static int Height = 100;
	private static int deep = 10;
	private long lastFPS;
	private int fps;
	
	public void updateFPS() {
		if (getTime() - lastFPS > 1000) {
			Display.setTitle("FPS: " + fps);
			fps = 0; //reset the FPS counter
			lastFPS += 1000; //add one second
		}
		fps++;
	}
	
	public void start() {
		try {
			Display.setDisplayMode(new DisplayMode(800,600));
			Display.create();
		} catch (LWJGLException e) {
			e.printStackTrace();
			System.exit(0);
		}
		
		lastFPS = getTime();
		
		// init OpenGL
		glMatrixMode(GL_PROJECTION);
		glOrtho(-1, 1, -1, 1, 1, -1);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glEnable(GL_DEPTH_TEST);
		
		while (!Display.isCloseRequested()) {
			//Lock fps to max 60. 
			Display.sync(60);
		    // Clear the screen and depth buffer
		    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
			
		    // set the color of the quad (R,G,B,A)
		    glColor3f(0.5f,0.5f,1.0f);
		    	
		    // draw quad
		    glBegin(GL_QUADS);
			
			
			/*
			GL11.glVertex3f(-Width/2,Height/2, 100 - deep/2);
			GL11.glVertex3f(Width/2,Height/2, 100 - deep/2);
			GL11.glVertex3f(-Width/2,-Height/2, 100 + deep/2);
			GL11.glVertex3f(Width/2,-Height/2, 100 + deep/2);
			GL11.glVertex3f(-Width/2,-Height/2, 100 - deep/2);
			GL11.glVertex3f(Width/2,-Height/2, 100 - deep/2);*/
		    glEnd();
	 
		    Display.update();
		    updateFPS();
		}		
		Display.destroy();
	}
	
	public long getTime() {
		return (Sys.getTime() * 1000) / Sys.getTimerResolution();
	}
	
	public static void main(String[] argv) {
		SimpleVisualiser displayExample = new SimpleVisualiser();
		displayExample.start();
	}

}
