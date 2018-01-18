package org.knowrob.utils.ros;

import java.awt.image.BufferedImage;
import java.awt.image.ComponentSampleModel;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferByte;
import java.awt.image.Raster;
import java.awt.image.SampleModel;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.apache.commons.codec.binary.Base64;

/**
 * Constants and Macros related to encoding of ROS Image messages.
 * @author Daniel Beßler
 */
public class ImageEncoding {
    public static final String RGB8 = "rgb8";
    public static final String RGBA8 = "rgba8";
    public static final String RGB16 = "rgb16";
    public static final String RGBA16 = "rgba16";
    public static final String BGR8 = "bgr8";
    public static final String BGRA8 = "bgra8";
    public static final String BGR16 = "bgr16";
    public static final String BGRA16 = "bgra16";
    public static final String MONO8="mono8";
    public static final String MONO16="mono16";

    // OpenCV CvMat types
    public static final String TYPE_8UC1="8UC1";
    public static final String TYPE_8UC2="8UC2";
    public static final String TYPE_8UC3="8UC3";
    public static final String TYPE_8UC4="8UC4";
    public static final String TYPE_8SC1="8SC1";
    public static final String TYPE_8SC2="8SC2";
    public static final String TYPE_8SC3="8SC3";
    public static final String TYPE_8SC4="8SC4";
    public static final String TYPE_16UC1="16UC1";
    public static final String TYPE_16UC2="16UC2";
    public static final String TYPE_16UC3="16UC3";
    public static final String TYPE_16UC4="16UC4";
    public static final String TYPE_16SC1="16SC1";
    public static final String TYPE_16SC2="16SC2";
    public static final String TYPE_16SC3="16SC3";
    public static final String TYPE_16SC4="16SC4";
    public static final String TYPE_32SC1="32SC1";
    public static final String TYPE_32SC2="32SC2";
    public static final String TYPE_32SC3="32SC3";
    public static final String TYPE_32SC4="32SC4";
    public static final String TYPE_32FC1="32FC1";
    public static final String TYPE_32FC2="32FC2";
    public static final String TYPE_32FC3="32FC3";
    public static final String TYPE_32FC4="32FC4";
    public static final String TYPE_64FC1="64FC1";
    public static final String TYPE_64FC2="64FC2";
    public static final String TYPE_64FC3="64FC3";
    public static final String TYPE_64FC4="64FC4";

    // Bayer encodings
    public static final String BAYER_RGGB8="bayer_rggb8";
    public static final String BAYER_BGGR8="bayer_bggr8";
    public static final String BAYER_GBRG8="bayer_gbrg8";
    public static final String BAYER_GRBG8="bayer_grbg8";
    public static final String BAYER_RGGB16="bayer_rggb16";
    public static final String BAYER_BGGR16="bayer_bggr16";
    public static final String BAYER_GBRG16="bayer_gbrg16";
    public static final String BAYER_GRBG16="bayer_grbg16";

    // Miscellaneous
    public static final String YUV422="yuv422";

    // Utility functions for inspecting an encoding string
    public static boolean isColor(final String encoding)
    {
      return encoding.equals(RGB8)  || encoding.equals(BGR8) ||
             encoding.equals(RGBA8) || encoding.equals(BGRA8) ||
             encoding.equals(RGB16) || encoding.equals(BGR16) ||
             encoding.equals(RGBA16) || encoding.equals(BGRA16);
    }

    public static boolean isMono(final String encoding)
    {
      return encoding.equals(MONO8) || encoding.equals(MONO16);
    }

    public static boolean isBayer(final String encoding)
    {
      return encoding.equals(BAYER_RGGB8) || encoding.equals(BAYER_BGGR8) ||
             encoding.equals(BAYER_GBRG8) || encoding.equals(BAYER_GRBG8) ||
             encoding.equals(BAYER_RGGB16) || encoding.equals(BAYER_BGGR16) ||
             encoding.equals(BAYER_GBRG16) || encoding.equals(BAYER_GRBG16);
    }

    public static boolean isScalar(final String encoding)
    {
      return encoding.equals(TYPE_32FC1) || encoding.equals(TYPE_32FC2) ||
    		  encoding.equals(TYPE_32FC3) || encoding.equals(TYPE_32FC4) ||
    		  encoding.equals(TYPE_64FC1) || encoding.equals(TYPE_64FC2) ||
    		  encoding.equals(TYPE_64FC3) || encoding.equals(TYPE_64FC4);
    }

    public static boolean hasAlpha(final String encoding)
    {
      return encoding.equals(RGBA8) || encoding.equals(BGRA8) ||
             encoding.equals(RGBA16) || encoding.equals(BGRA16);
    }

    public static int numChannels(final String encoding)
    {
      // First do the common-case encodings
      if (encoding.equals(MONO8) ||
          encoding.equals(MONO16))
        return 1;
      if (encoding.equals(BGR8) ||
          encoding.equals(RGB8) ||
          encoding.equals(BGR16) ||
          encoding.equals(RGB16))
        return 3;
      if (encoding.equals(BGRA8) ||
          encoding.equals(RGBA8) ||
          encoding.equals(BGRA16) ||
          encoding.equals(RGBA16))
        return 4;
      if (encoding.equals(BAYER_RGGB8) ||
          encoding.equals(BAYER_BGGR8) ||
          encoding.equals(BAYER_GBRG8) ||
          encoding.equals(BAYER_GRBG8) ||
          encoding.equals(BAYER_RGGB16) ||
          encoding.equals(BAYER_BGGR16) ||
          encoding.equals(BAYER_GBRG16) ||
          encoding.equals(BAYER_GRBG16))
        return 1;
      return -1;
    }

    public static int bitDepth(final String encoding)
    {
      if (encoding.equals(MONO16))
        return 16;
      if (encoding.equals(MONO8)       ||
          encoding.equals(BGR8)        ||
          encoding.equals(RGB8)        ||
          encoding.equals(BGRA8)       ||
          encoding.equals(RGBA8)       ||
          encoding.equals(BAYER_RGGB8) ||
          encoding.equals(BAYER_BGGR8) ||
          encoding.equals(BAYER_GBRG8) ||
          encoding.equals(BAYER_GRBG8))
        return 8;

      if (encoding.equals(MONO16)       ||
          encoding.equals(BGR16)        ||
          encoding.equals(RGB16)        ||
          encoding.equals(BGRA16)       ||
          encoding.equals(RGBA16)       ||
          encoding.equals(BAYER_RGGB16) ||
          encoding.equals(BAYER_BGGR16) ||
          encoding.equals(BAYER_GBRG16) ||
          encoding.equals(BAYER_GRBG16))
        return 16;
      return -1;
    }
	
	public static String encodeBase64(BufferedImage image, String type) {
		try {
			String imageString = null;
			ByteArrayOutputStream bos = new ByteArrayOutputStream();
			ImageIO.write(image, type, bos);
			byte[] imageBytes = bos.toByteArray();
			imageString = new String(Base64.encodeBase64(imageBytes));
			bos.close();
			return "data:image/"+type+";base64,"+imageString;
		}
		catch(Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	public static String encodeBase64(String imagePath) {
		try {
			return encodeBase64(ImageIO.read(new File(imagePath)), "png");
		}
		catch(Exception e) {
			e.printStackTrace();
			return null;
		}
	}
	
	public static String encodeBase64(sensor_msgs.Image rosImgMsg) {
		try {
			return encodeBase64(messageToBufferedImage(rosImgMsg), "png");
		}
		catch(Exception e) {
			e.printStackTrace();
			return null;
		}
	}
	
	public static String encodeBase64(sensor_msgs.CompressedImage rosImgMsg) {
		try {
			String imageString = new String(Base64.encodeBase64(rosImgMsg.getData().array()));
			return "data:image/"+rosImgMsg.getFormat()+";base64,"+imageString;
		}
		catch(Exception e) {
			e.printStackTrace();
			return null;
		}
	}
	
    public static BufferedImage messageToBufferedImage(sensor_msgs.Image imgMsg) {
    	// TODO(daniel): support other message formats
    	DataBufferByte buffer = new DataBufferByte(imgMsg.getData().array(), imgMsg.getWidth()*imgMsg.getHeight());
        SampleModel sampleModel = new ComponentSampleModel(DataBuffer.TYPE_BYTE,
        		imgMsg.getWidth(), imgMsg.getHeight(), 3, imgMsg.getWidth()*3, new int[]{2,1,0});
        Raster raster = Raster.createRaster(sampleModel, buffer, null);
        BufferedImage image = new BufferedImage(imgMsg.getWidth(),
        		imgMsg.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
        image.setData(raster);
        return image;
    }
}
