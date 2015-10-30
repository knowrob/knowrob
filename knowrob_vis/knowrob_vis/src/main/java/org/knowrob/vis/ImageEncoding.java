package org.knowrob.vis;

import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.apache.commons.codec.binary.Base64;

public class ImageEncoding {
	public static String encodeBase64(String imagePath) {
        try {
    		BufferedImage img = ImageIO.read(new File(imagePath));
    		return encodeBase64(img, "png");
		}
        catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        return null;
	}
	
	public static String encodeBase64(BufferedImage image, String type) throws IOException {
		String imageString = null;
		ByteArrayOutputStream bos = new ByteArrayOutputStream();
		ImageIO.write(image, type, bos);
		byte[] imageBytes = bos.toByteArray();
		imageString = new String(Base64.encodeBase64(imageBytes));
		bos.close();
		return imageString;
	}
}
