package com.GCS.xbee;

import ros.*;
import ros.communication.*;
import ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate;


import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import com.GCS.xbee.Coordinate;
import com.rapplogic.xbee.api.ApiId;
import com.rapplogic.xbee.api.PacketListener;
import com.rapplogic.xbee.api.XBee;
import com.rapplogic.xbee.api.XBeeAddress64;
import com.rapplogic.xbee.api.XBeeException;
import com.rapplogic.xbee.api.XBeeResponse;
import com.rapplogic.xbee.api.zigbee.ZNetRxResponse;
import com.rapplogic.xbee.api.zigbee.ZNetTxRequest;
import com.rapplogic.xbee.util.ByteUtils;

public class XBeeGCS {

	private final static Logger log = Logger.getLogger(XBeeGCS.class);
	private static XBee xbee;
	
	static long next;
	static long prev;
	public static Publisher<ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate> pub;
	
	
	public static void main(String[] args)
	{
		Ros ros = Ros.getInstance();
		ros.init("xbee_bridge");
		PropertyConfigurator.configure("log4j.properties");
		xbee = new XBee();
		
		NodeHandle n = ros.createNodeHandle();
		//create robotmodel datatype and pull info from planedata into robotmodel then publish
		
		
		try {
			xbee.open("/dev/ttyUSB0", 115200);
			xbee.addPacketListener(new GCSPacketListener());
			
			try {
				pub = n.advertise("/pub", new ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate(), 100);
				//ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate m = new ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate();
				//use robotmodel data type right here, set equal current planedata values
			
				while (true);
			}
			catch (ros.RosException re) {
				
			}
			
		}
		catch (XBeeException e) {
			e.printStackTrace();
		}
		finally {
			xbee.close();
			pub.shutdown();
		}
		
		
		
	}
	

	public void transmit(XBeeAddress64 addr, Coordinate wp) 
	{
		int waypoint[] = new int[3];
		int payload[] = new int[16];	//4 int32's
		
		// ArduPilot asks for these multipliers
		waypoint[0] = (int) (wp.x * 10); 
		waypoint[1] = (int) (wp.y * 10);
		waypoint[2] = (int) (wp.z * 100);
		
		// flip endian for payload (Arduino is Big Endian)
		for (int i = 0; i < waypoint.length; i++) 
		{
			ByteBuffer bb = ByteBuffer.allocate(32);
			bb.order(ByteOrder.LITTLE_ENDIAN);
			bb.putInt(waypoint[i]);
			bb.order(ByteOrder.BIG_ENDIAN);
			for (int j = 0; j < 4; j++) 
			{
				payload[4*i+j] = bb.array()[j];
			}
		}
		
		// put checksum (sum of lat and alt) in packet payload
		ByteBuffer bb = ByteBuffer.allocate(32);
		bb.order(ByteOrder.LITTLE_ENDIAN);
		bb.putInt(waypoint[0] + waypoint[2]);
		bb.order(ByteOrder.BIG_ENDIAN);
		for (int j = 0; j < 4; j++) {
			payload[12+j] = bb.array()[j];
		}
		
		// send packet
		ZNetTxRequest request = new ZNetTxRequest(addr, payload);
		try {
			xbee.sendSynchronous(request, 5000);
		} catch (XBeeException e) {
			e.printStackTrace();
		}
	
		// log output
		//log.debug("zb request is " + request.getXBeePacket().getPacket());
		// undo the multipliers to make logging prettier
		waypoint[0] /= 10; waypoint[1] /= 10; waypoint[2] /= 100;
		//log.info("sent " + Arrays.toString(waypoint) + " to Plane " + getDataMap().get(addr).planeID);
	}
	
	private static class GCSPacketListener implements PacketListener {

		@Override
		public void processResponse(XBeeResponse response) {
			if (response.getApiId() == ApiId.ZNET_RX_RESPONSE) {
				ZNetRxResponse rx = (ZNetRxResponse) response;
				
				log.info("Received RX packet, option is " + rx.getOption() + 
						", sender 64 address is " + ByteUtils.toBase16(rx.getRemoteAddress64().getAddress()) + 
						", remote 16-bit address is " + ByteUtils.toBase16(rx.getRemoteAddress16().getAddress()) + 
						", data is " + ByteUtils.toBase16(rx.getData()));
			
			}
		}
		
		private static PlaneData packetParser(ZNetRxResponse response) {
			
			if (response.getData().length != 44) return null;
			
			int planeDataArray[] = new int[11];
			byte data[] = new byte[response.getData().length];
			
			for (int i = 0; i < data.length; i++)
				data[i] = (byte) response.getData()[i];
			
			for (int i = 0; i < data.length; i += 4) {
				ByteBuffer bb = ByteBuffer.allocate(32);
				bb.order(ByteOrder.BIG_ENDIAN);
				bb.put(data, i, 4);
				bb.order(ByteOrder.LITTLE_ENDIAN);
				planeDataArray[i / 4] = bb.getInt(0);
			}
			//could just parse new data into robotobject datatype and return it
			PlaneData pd = new PlaneData();
			ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate m = new ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate();
			
			pd.currLat = planeDataArray[0];
			pd.currLng = planeDataArray[1];
			pd.currAlt = planeDataArray[2];
			pd.nextLat = planeDataArray[3];
			pd.nextLng = planeDataArray[4];
			pd.nextAlt = planeDataArray[5];
			pd.ground_speed = planeDataArray[6];
			pd.target_bearing = planeDataArray[7];
			pd.currWP = planeDataArray[8];
			pd.WPdistance = planeDataArray[9];
			
			//i dunno what this is???
			//pd.battV = planeDataArray[10];
			
			//check where planeID should be set
		    m.currentLatitude = planeDataArray[0];
		    m.currentLongitude = planeDataArray[1];
		    m.currentAltitude = planeDataArray[2];
		    m.destLatitude = planeDataArray[3];
		    m.destLongitude = planeDataArray[4];
		    m.destAltitude = planeDataArray[5];
		    m.groundSpeed = planeDataArray[6];
		    m.targetBearing = planeDataArray[7];
		    m.currentWaypointIndex = planeDataArray[8];
		    m.distanceToDestination = planeDataArray[9];
			
		    pub.publish(m);
		    
			next = System.currentTimeMillis();
			/*
			System.out.println("Address: " + ByteUtils.toBase16(response.getRemoteAddress16().getAddress())
					+ " Data: " + pd + " Time Elapsed: " + (next-prev));
					*/
			System.out.println(pd + "," + (next-prev));
			prev = next;
			
			return pd;
		}
	}
}


