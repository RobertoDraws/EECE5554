import re
import rospy
import serial
import numpy as np
from lab3_package.msg import IMU_msg
from std_msgs.msg import Header

ser = serial.Serial('/dev/ttyUSB0',115200)

def euler_2_quart(roll,pitch,yaw):
    yaw = yaw*np.pi/180
    pitch = pitch*np.pi/180
    roll = roll*np.pi/180
    
    x = (np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2))-(np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2))
    y = (np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2))+(np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2))
    z = (np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2))-(np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2))
    w = (np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2))+(np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2))
    return x,y,z,w

def imu():
    topic_pub = rospy.Publisher('imu', IMU_msg, queue_size=10)
    rospy.init_node('imu_publisher')
    rospy.logdebug('IMU initalized')
    rate = rospy.Rate(30) #og 40
    while not rospy.is_shutdown():
        string_data = ser.readline().decode('utf-8')
        #whole_data
        #rospy.logwarn(string_data)
        split_data = string_data.split(',')
        #for x in split_data:
        #    rospy.logwarn(x)
        if split_data[0] != '$VNYMR':
            rospy.logwarn('OOf: $VNYMR detected.')
        elif len(split_data) != 13:
            rospy.logwarn('OOf: to big/small')
        elif split_data[2] == '':
            rospy.logwarn('OOf: empty')
        #checks if '/x' is in result; if it is skipps to avoid errors
#        elif "\\x" in string_data:
#            rospy.logwarn(Skipping data due to /x in result')
#        elif "\\x" in (split_data for split_data in range(1,12)):
#            rospy.logwarn('Manual Warning')
        else:
            #genrating & populating msg
            msg = IMU_msg()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()    
            yaw = float(re.sub("\x00", "",split_data[1]))
            pitch = float(re.sub("\x00", "", split_data[2]))
            roll = float(re.sub("\x00", "",split_data[3]))
            msg.magnetic_field.x = float(re.sub("\x00", "",split_data[4]))*0.0001
            msg.magnetic_field.y = float(re.sub("\x00", "",split_data[5]))*0.0001
            msg.magnetic_field.z = float(re.sub("\x00", "",split_data[6]))*0.0001
            msg.linear_acceleration.x = float(re.sub("\x00", "",split_data[7]))
            msg.linear_acceleration.y = float(re.sub("\x00", "",split_data[8]))
            msg.linear_acceleration.z = float(re.sub("\x00", "",split_data[9]))
            msg.angular_velocity.x = float(re.sub("\x00", "",split_data[10]))
            msg.angular_velocity.y = float(re.sub("\x00", "",split_data[11]))
            # removing the shit at the end of the message 
            msg.angular_velocity.z = float(re.sub("\x00", "", split_data[12].split('*')[0]))
            split_gyro_z = list(split_data[12])
            new_gyro_z = "".join(split_gyro_z[:len(split_gyro_z)-6])
            msg.angular_velocity.z = float(re.sub("\x00", "",new_gyro_z))

            #eueler2quart
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = euler_2_quart(roll, pitch, yaw)
        
            #publish message and print in concole. 
            rospy.loginfo(msg)
            topic_pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        pass
