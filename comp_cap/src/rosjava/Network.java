package rosjava;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;

public class Network {

	public static String[] getServiceHeader(String serviceHost,
			int servicePort, String serviceName) {
		String[] headerRequest;
		String[] headerResponse;
		byte[] encode_header;
		byte[] header_recived;

		SocketAddress addr;
		Socket s;

		DataInputStream in;
		DataOutputStream out;

		headerRequest = new String[] { "service=" + serviceName, "probe=1",
				"callerid=/rosservice", "md5sum=*" };
		encode_header = rosjava.Network
				.encode_ros_handshake_header(headerRequest);
		headerResponse = new String[4];

		do {
			try {
				addr = new InetSocketAddress(serviceHost, servicePort);
				s = new Socket();
				s.connect(addr, 100);

				in = new DataInputStream(s.getInputStream());
				out = new DataOutputStream(s.getOutputStream());

				do {
					out.write(encode_header);
					out.flush();
					header_recived = new byte[in.available()];
					in.read(header_recived);
				} while (!rosjava.Network.decode_ros_hanshake_header(
						header_recived, headerResponse));

			} catch (UnknownHostException e) {
				System.out
						.println("Error: Error in rosjava.Network.getServiceHeader() ");
				System.out.println("ServiceName: " + serviceName);
				System.out.println(e.getMessage());
			} catch (IOException e) {
				System.out
						.println("Error: Error in rosjava.Network.getServiceHeader() ");
				System.out.println("ServiceName: " + serviceName);
				System.out.println(e.getMessage());
			}

		} while (headerResponse[0] == null || headerResponse[1] == null
				|| headerResponse[2] == null || headerResponse[3] == null
				|| headerResponse.length != 4);

		return headerResponse;
	}

	// Fehlerbehandlung muss noch eingefügt werden, code restrukturieren
	public static byte[] encode_ros_handshake_header(String[] header) {
		byte[] encode_header;
		ArrayList<byte[]> header_byte = new ArrayList<byte[]>();
		for (String s : header) {
			header_byte.add(convertStringToByteArray(s));
		}
		header_byte.trimToSize();
		for (int j = 0; j < header_byte.size(); j++) {
			byte[] b = header_byte.get(j);
			byte[] length = convertIntToByteArray(b.length);
			byte[] header_part = new byte[4 + b.length];
			for (int i = 0; i < length.length; i++) {
				header_part[i] = length[i];
			}
			for (int i = 4; i < header_part.length; i++) {
				header_part[i] = b[i - 4];
			}
			header_byte.remove(j);
			header_byte.add(j, header_part);
		}
		header_byte.trimToSize();

		int size = 0;
		for (byte[] b : header_byte) {
			size = size + b.length;
		}
		encode_header = new byte[4 + size];

		int j = 0;
		byte[] length = convertIntToByteArray((size));
		for (int i = 0; i < length.length; i++) {
			encode_header[i] = length[i];
			j++;
		}
		
		for (int k = 0; k < 4; k++) {
			for (int i = 0; i < header_byte.get(k).length; i++) {
				encode_header[j] = header_byte.get(k)[i];
				j++;
			}
		}

		return encode_header;
	}

	public static boolean decode_ros_hanshake_header(byte[] header,
			String[] result) {
		byte[] length_all;
		byte[] length_first;
		byte[] length_second;
		byte[] length_third;
		byte[] length_fourth;

		int len_all;
		int len_first;
		int len_second;
		int len_third;
		int len_fourth;

		if (header.length >= 4) {

			if (result == null) {
				result = new String[4];
			}
			length_all = subArray(0, 3, header);
			len_all = convertByteArrayToInt(length_all);
			if (header.length == (len_all + 4)) {
				length_first = subArray(4, 7, header);
				len_first = convertByteArrayToInt(length_first);

				length_second = subArray(8 + len_first, 12 + len_first - 1,
						header);
				len_second = convertByteArrayToInt(length_second);

				length_third = subArray(12 + len_first + len_second, 16
						+ len_first + len_second - 1, header);
				len_third = convertByteArrayToInt(length_third);

				length_fourth = subArray(16 + len_first + len_second
						+ len_third, 20 + len_first + len_second + len_third
						- 1, header);
				len_fourth = convertByteArrayToInt(length_fourth);

				result[0] = new String(subArray(8, 8 + len_first - 1, header));
				result[1] = new String(subArray(12 + len_first, 12 + len_first
						+ len_second - 1, header));
				result[2] = new String(subArray(16 + len_first + len_second, 16
						+ len_first + len_second + len_third - 1, header));
				result[3] = new String(subArray(20 + len_first + len_second
						+ len_third, 20 + len_first + len_second + len_third
						+ len_fourth - 1, header));

				return true;
			}

			return false;
		}

		return false;
	}

	/**
	 * return a byte[] containing a string in little endian byte order
	 * 
	 * @param s
	 * @return
	 */
	public static byte[] convertStringToByteArray(String s) {
		return s.getBytes();
	}

	public static byte[] convertIntToByteArray(int i) {
		byte[] bytes = new byte[4];
		bytes[3] = (byte) (i >>> 24);
		bytes[2] = (byte) (i >>> 16);
		bytes[1] = (byte) (i >>> 8);
		bytes[0] = (byte) (i);
		return bytes;
	}

	public static int unsignedByteToInt(byte b) {
		return (int) b & 0xFF;
	}

	public static int convertByteArrayToInt(byte[] b) {
		int val = 0;
		val += unsignedByteToInt(b[3]) << 24;
		val += unsignedByteToInt(b[2]) << 16;
		val += unsignedByteToInt(b[1]) << 8;
		val += unsignedByteToInt(b[0]) << 0;
		return val;
	}

	public static byte[] subArray(int begin, int end, byte[] b) {
		byte[] result;
		int j;
		if ((end > begin) && (b.length > (end - begin + 1))) {
			result = new byte[end - (begin - 1)];
			j = begin;
			for (int i = 0; i < result.length; i++) {
				result[i] = b[j];
				j++;
			}
			return result;
		}
		return null;
	}
}
