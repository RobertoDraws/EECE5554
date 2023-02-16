#!/usr/bin/env
import utm
import numpy
from datetime import datetime
from gps_driver.msg import gps_msg
from std_msgs.msg import Float64
import rospy
import math
import serial

ser = serial.Serial('/dev/pts/2', 4800)

def talker():
    i = 0
    pub = rospy.Publisher('gps_data', gps_msg, queue_size=10)
    rospy.init_node('gps_data_pub')
    rate= rospy.Rate(10)
    while not rospy.is_shutdown():
        arraydata = ser.readline().decode('utf-8')
        data = arraydata.split(',')
        print(data[0])
        if '$GPGGA' in data[0]:
            if data[2] == ' ':
                print('Oof - Invailid Reading')
                rospy.loginfo('Oof - Invailid Reading')
            else:
                rospy.loginfo(data)
                print(data)

                #UTC
                utc = data[1]
                #LAT
                lat = data[2]
                #print(lat)
                #LAT Direction
                lat_dir = data[3]
                #Long
                lon = data[4]
                #print(lon)
                #LongDir
                lon_dir = data[5]
                #quality
                quality = data[6]
                #hdop
                hdop = data[8]
                #altitude
                alt = data[9]
                print(alt)
                
                #Convert from minutes to degress
                dec_lat = minuteSec2degrees(lat, lat_dir)
                dec_lon = minuteSec2degrees(lon, lon_dir)

                #print(dec_lat)
                #print(dec_lon)


                pub_msg = gps_msg()
                #Header
                pub_msg.Header.stamp = rospy.Time.now()
                pub_msg.Header.seq = i=i+1
                pub_msg.Header.frame_id = 'GPS1_Frame'

                #Populate
                pub_msg.UTC = numpy.float64(utc)
                pub_msg.HDOP = numpy.float64(hdop)
                pub_msg.Latitude = dec_lat
                pub_msg.Longitude = dec_lon
                pub_msg.Altitude = numpy.float64(alt)
                pub_msg.UTM_easting, pub_msg.UTM_northing, pub_msg.Zone, pub_msg.Letter = utm.from_latlon(pub_msg.Latitude, pub_msg.Longitude)
                
                rospy.loginfo(pub_msg)
                pub.publish(pub_msg)

        else: 
            print('oof- err') 
                
                #rate.sleep()
                
#        data = serial.Serial(serial_port, serial_baud, timeout=3)
#        print(data)
        # I need to split the string here
        # Then I need messages
        # Then I collect seperatly into a rosbag
        # Write roslaunch
        #rospy.loginfo(data)

def minuteSec2degrees(lat_long_mins, dir):
    lat_long_mins = float(lat_long_mins)
    x = (lat_long_mins/100)
    minutes = math.trunc((lat_long_mins/100-x)*100)/60
    #minutes = ((lat_long_mins/100-x)*100)
    seconds = (lat_long_mins-lat_long_mins)*100/3600
    #seconds = (lat_long_mins - (lat_long_mins)*100)
    lat_long_deg = x + (minutes/60) + (seconds/3600)
    if dir == 'W':
        lat_long_deg *= -1
    if dir == 'S': 
        lat_long_deg *= -1
    return lat_long_deg



if __name__ == '__main__':
    try:
#   Do I pull data like this
#        SENSOR_NAME = 'gps_data'
#        rospy.init_node('gps_data_pub')
#        serial_port = rospy.get_param('~port','/dev/ttyS1')
#        serial_baud = rospy.get_param('~baudrate','4800')
#        sampling_rate = rospy.get_param('~sampling_rate',10.0)
#        offset = rospy.get_param('~atm_offset', 0)
#        port = serial.Serial(serial_port, serial_baud, timeout=3)
#        rospy.logdebung("Using GPS sensor on port " +serial_port+" at "+str(serial_baud))
        talker()    

#        utm_Easting = rospy.Publisher(SENSOR_NAME+'/easting', Float64, queue_size=5)
#        utm_northing = rospy.Publisher(SENSOR_NAME+'/northing', Float64, queue_size=5)
#        zone = rospy.Publisher(SENSOR_NAME+'/zone', Float64, queue_size=5)
#        letter = rospy.Publisher(SENSOR_NAME+'/letter', Float64, queue_size=5)
#   Or Should I Split it up like this
#    data = ser.readline().decode('utf-8')
#    print(data)
#    truncated = data.split(',')
    except rospy.ROSInterruptException:
        pass
    