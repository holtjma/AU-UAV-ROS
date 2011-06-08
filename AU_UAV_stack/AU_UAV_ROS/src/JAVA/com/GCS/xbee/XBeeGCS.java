package com.GCS.xbee;

import ros.*;
import ros.communication.*;
import ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate;
import ros.pkg.AU_UAV_ROS.msg.Command;
import ros.pkg.AU_UAV_ROS.srv.RequestPlaneID;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.Iterator;
import java.util.InputMismatchException;
import java.util.NoSuchElementException;
import java.util.Scanner;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import com.rapplogic.xbee.api.ApiId;
import com.rapplogic.xbee.api.PacketListener;
import com.rapplogic.xbee.api.XBee;
import com.rapplogic.xbee.api.XBeeAddress64;
import com.rapplogic.xbee.api.XBeeException;
import com.rapplogic.xbee.api.XBeeResponse;
import com.rapplogic.xbee.api.zigbee.ZNetRxResponse;
import com.rapplogic.xbee.api.zigbee.ZNetTxRequest;
import com.rapplogic.xbee.util.ByteUtils;

/**
 * The main class for the XBeeGCS.
 * 
 * This class controls XBee communication, and can send and receive packets with XBee-equipped ArduPilots.
 * It maintains a GUI for loading an arbitrary waypoint in air, and can work with a collision avoidance thread for multiple planes.
 * 
 * @author Varun Sampath <vsampath@seas.upenn.edu>
 */
public class XBeeGCS {
	
	private final static Logger log = Logger.getLogger(XBeeGCS.class);	// logger for the log4j system
	
	// instance variables
	static private XBee xbee;		// the XBee hooked up to this GCS
	static private HashMap<XBeeAddress64, PlaneData> dataMap;		// collection of all telemetry data indexed by IEEE address
	private XBeeAddress64 latest;							// latest plane to transmit valid telemetry data
	private int planeCounter;								// the number of planes

	//ROS Variables
	static private Ros ros;
	static private NodeHandle n;
	private Publisher<ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate> telemetryPub;
	private ServiceClient<RequestPlaneID.Request, RequestPlaneID.Response, RequestPlaneID> requestPlaneIDService;
	static private Subscriber.QueueingCallback<ros.pkg.AU_UAV_ROS.msg.Command> callback;
	private Subscriber<ros.pkg.AU_UAV_ROS.msg.Command> sub;
				
	/**
	 * Initializes XBee serial communication, packet listening thread, GUI, and if wanted, collision avoidance thread.
	 * @throws XBeeException if failing to initialize XBee serial link.
	 */
	public XBeeGCS () throws XBeeException {
		PropertyConfigurator.configure("log4j.properties");
		
		//ROS init
		ros = Ros.getInstance();
		ros.init("XBee_IO");
		n = ros.createNodeHandle();
		try
		{
			telemetryPub = n.advertise("/telemetry", new ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate(), 100);
			requestPlaneIDService = n.serviceClient("request_plane_ID", new RequestPlaneID(), false);
			callback = new Subscriber.QueueingCallback<ros.pkg.AU_UAV_ROS.msg.Command>();
			sub = n.subscribe("/commands", new ros.pkg.AU_UAV_ROS.msg.Command(), callback, 100);
		}
		catch (ros.RosException re)
		{
		
		}
		
		// set up the coordinator XBee serial communication
		xbee = new XBee();
		xbee.open("/dev/ttyUSB0", 115200);
		setDataMap(new HashMap<XBeeAddress64, PlaneData>());
		planeCounter = 0;
		
		System.out.println("Starting packet listener...");
		xbee.addPacketListener(new GCSPacketListener());	// packet listener for plane data
		System.out.println("Waiting for packets...");
	}
	
	private void setDataMap(HashMap<XBeeAddress64, PlaneData> dataMap) {
		this.dataMap = dataMap;
	}

	/**
	 * Returns the internal hash map of plane telemetry data indexed by each plane's XBee's 64-bit IEEE address.
	 * @return the internal hash map of plane telemetry data
	 */
	public static HashMap<XBeeAddress64, PlaneData> getDataMap() {
		return dataMap;
	}
	
	private void setLatest(XBeeAddress64 latest) {
		this.latest = latest;
	}

	/**
	 * Returns the 64-bit IEEE address of the XBee radio belonging to the plane that last transmitted valid telemetry data.
	 * @return the 64-bit IEEE address of the last plane radio to send valid telemetry data
	 */
	public XBeeAddress64 getLatest() {
		return latest;
	}
	
	/**
	 * Updates the internal hash map with new plane data and informs the collision avoidance thread of it.
	 * 
	 * Note: this method should be thread-safe with regards to the collision avoidance algorithm.
	 * 
	 * @param addr	The 64-bit IEEE address of the sending node
	 * @param data	The telemetry data sent by the node
	 */
	public void addData(XBeeAddress64 addr, PlaneData data) {
		if (data != null) {
			if (!getDataMap().containsKey(addr))
			{
				try
				{
					RequestPlaneID.Request req = new RequestPlaneID.Request();
					req.requestedID = -1;
					data.planeID = requestPlaneIDService.call(req).planeID;
					data.seq = 0;
					log.info("New plane with ID #"+data.planeID+" created.");
					planeCounter++;
				}
				catch (RosException re)
				{
					log.error("Request Plane ID failed!");
					return;
				}
			}
			else
			{
				data.planeID = getDataMap().get(addr).planeID;
				data.seq++;
			}	
				
			// update hash map
			setLatest(addr);
			log.info(data);
			synchronized (getDataMap()) {
				getDataMap().put(addr, data);
				getDataMap().notify();
			}
			
			//forward the information on to ros
			ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate tUpdate = new ros.pkg.AU_UAV_ROS.msg.TelemetryUpdate();
			tUpdate.currentLatitude = data.currLat / 1000000.0; //change to real lat
			tUpdate.currentLongitude = data.currLng / 1000000.0; //change to real long
			tUpdate.currentAltitude = data.currAlt;
			tUpdate.destLatitude = data.nextLat / 1000000.0;
			tUpdate.destLongitude = data.nextLng / 1000000.0;
			tUpdate.destAltitude = data.nextAlt;
			//TODO: not sure what this ground speed is in, but we have it as meters/sec, so might need to change it
			tUpdate.groundSpeed = data.ground_speed;
			tUpdate.targetBearing = data.target_bearing;
			tUpdate.currentWaypointIndex = data.currWP;
			//TODO: we need to change this to meters
			tUpdate.distanceToDestination = data.WPdistance;//not sure what unit this is anymore, supposedly meters but it doesn't work out right
			tUpdate.planeID = data.planeID;
			//tUpdate.telemetryHeader.stamp = ros.communication.Time.now();
			
			log.info("Forwarding telemetry data to ROS topic.");
			telemetryPub.publish(tUpdate);
			
		}
	}
	
	/**
	 * Transmits a new waypoint for a plane to follow.
	 * 
	 * This function transmits a waypoint in a big endian GCS_packet_t format specified in the modified
	 * ArduPilot code.  The packet is transmitted synchronously, with an ACK timeout of 5 seconds.
	 * Note: while Coordinate fields are doubles, they should be large enough numbers (i.e. > 1 million)
	 * so the ArduPilot can use them.  This method casts the Coordinate fields as 32-bit integers
	 * and only multiplies the latitude and longitude by 10 and the altitude by 100 for the ArduPilot.
	 * 
	 * @param addr	The 64-bit IEEE address of the node to send the waypoint to
	 * @param wp	The waypoint to send to the node
	 */
	public static void transmit(XBeeAddress64 addr, Coordinate wp) {
		int waypoint[] = new int[3];
		int payload[] = new int[16];	//4 int32's
		
		// ArduPilot asks for these multipliers
		waypoint[0] = (int) (wp.x * 10); 
		waypoint[1] = (int) (wp.y * 10);
		waypoint[2] = (int) (wp.z * 100);
		
		// flip endian for payload (Arduino is Big Endian)
		for (int i = 0; i < waypoint.length; i++) {
			ByteBuffer bb = ByteBuffer.allocate(32);
			bb.order(ByteOrder.LITTLE_ENDIAN);
			bb.putInt(waypoint[i]);
			bb.order(ByteOrder.BIG_ENDIAN);
			for (int j = 0; j < 4; j++) {
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
		log.debug("zb request is " + request.getXBeePacket().getPacket());
		// undo the multipliers to make logging prettier
		waypoint[0] /= 10; waypoint[1] /= 10; waypoint[2] /= 100;
		log.info("sent " + Arrays.toString(waypoint) + " to Plane " + getDataMap().get(addr).planeID);
	}
	
	/**
	 * Closes the XBee serial link
	 */
	public void exit() {
		n.shutdown();
		xbee.close();
	}

	/**
	 * Creates a new XBeeGCS.
	 * Exits with a fatal error if the XBee serial link cannot be made.
	 */
	public static void main(String[] args) {
		try {
			new XBeeGCS();
			
			//if you say while true, you can't kill it in a normal manner
			while(n.ok())
			{
				ros.spinOnce();
				while(!callback.isEmpty())
				{
					//TODO:forward the command
					try
					{
						//pop our command and store it
						ros.pkg.AU_UAV_ROS.msg.Command command = callback.pop();
						
						//we need to see if the plane ID in the command is in our list
						Set entrySet = getDataMap().entrySet();
						Iterator ii = entrySet.iterator();
						while(ii.hasNext())
						{
							Map.Entry entry = (Map.Entry) ii.next();
							PlaneData myPD = (PlaneData)(entry.getValue());
							if(myPD.planeID == command.planeID)
							{
								//we found a matching address and ID
								log.info("Message forwarding");
								
								//get our address from the entry
								XBeeAddress64 myTarget = (XBeeAddress64)(entry.getKey());
								
								//fill out the rest of our info
								Coordinate myCoor = new Coordinate();
								myCoor.x = command.latitude*1000000.0;
								myCoor.y = command.longitude*1000000.0;
								myCoor.z = command.altitude;
								
								//fire away
								transmit(myTarget, myCoor);
								break;
							}
							//else this isn't the right ID
						}
						
						//when we get here, we've either forwarded the message or we didn't find the ID	
						//transmit(XBeeAddress64 addr, Coordinate wp)
					}
					catch (InterruptedException ie)
					{
						log.error("InterruptedException while popping Command callback");
					}
				}
				try
				{
					Thread.sleep(50);
				}
				catch (InterruptedException ie)
				{
					//nothing?
				}
			}
		} 
		catch (XBeeException e) {
			log.fatal("XBeeGCS: Failed to initialize XBee serial link, exiting.");
			System.exit(-1);
		}
		
		finally {
			xbee.close();
			n.shutdown();
		}
	}

	/**
	 * Packet Listener class for receiving data from XBees on ArduPilots.
	 * Loads new telemetry data into the GCS internal hash map if provided.  Prints all other received packets.
	 */
	private class GCSPacketListener implements PacketListener {

		// keep track of time between packets
		private long next;
		private long prev;

		/**
		 * Receives packet and adds the parsed data from it to the Collision Avoidance object's
		 * internal hash map of plane telemetry data.
		 */
		@Override
		public void processResponse(XBeeResponse response) {
			if (response.getApiId() == ApiId.ZNET_RX_RESPONSE) {
				ZNetRxResponse rx = (ZNetRxResponse) response;

				log.debug("Received RX packet, option is " + rx.getOption() + 
						", sender 64 address is " + ByteUtils.toBase16(rx.getRemoteAddress64().getAddress()) + 
						", remote 16-bit address is " + ByteUtils.toBase16(rx.getRemoteAddress16().getAddress()) + 
						", data is " + ByteUtils.toBase16(rx.getData()));

				addData(rx.getRemoteAddress64(), packetParser(rx));
			}
		}

		/**
		 * Parses the received packet's payload.
		 * 
		 * If the data is not of the specific XBeeGCS payload format, the data is interpreted as a string and logged.
		 * The XBeeGCS format is in the print_position() function of GCS_XBee.pde, and is composed of 10 32-bit big endian integers
		 * 
		 * @param response The ZigBee Receive Packet Frame
		 * @return the PlaneData object containing the new telemetry data.  Returns null if the received packet's payload
		 * is not of XBeeGCS form.
		 */
		private PlaneData packetParser(ZNetRxResponse response) {
			next = System.currentTimeMillis();
			log.debug("Time elapsed is " + (next-prev)+"ms, response length is "+response.getData().length);
			prev = next;	

			// If response isn't a telemetry packet that we sent, just print it out
			if (response.getData().length != 40) {
				StringBuffer sb = new StringBuffer();
				for (int i = 0; i < response.getData().length; i++)
					sb.append((char)response.getData()[i]);
				log.info(sb.toString());
				return null;
			}

			// flip data in packet to little endian and store it in an int array
			int planeDataArray[] = new int[10];
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

			// store data in new PlaneData object
			PlaneData pd = new PlaneData();
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
			
			return pd;
		}
	}
}
