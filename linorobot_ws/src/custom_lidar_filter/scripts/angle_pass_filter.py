#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from math import degrees

pub = None

def scan_listener(data):
	global pub

	# transform tuples to lists (that are editable)
	ranges      = list(data.ranges)
	intensities = list(data.intensities)

	# recalculate min/max ranges after filtering
	range_min = float('inf')
	range_max = float('-inf')

	# iterate over each individual scan
	for i in xrange(len(data.ranges)):
		# obtain angle of scan in degrees
		angle = degrees(data.angle_min + i * data.angle_increment)

		# filter the posterior half plan (0 deg is forward)
		if angle > -90.0 and angle < 90.0:
			# scrape registered distance
			ranges[i] = float('inf')
			intensities[i] = 0.0
		# use acceptable scans to recalculate min/max ranges
		else:
			range_min = min(range_min, ranges[i])
			range_max = max(range_max, ranges[i])    \
 				    if ranges[i] != float('inf') \
				    else range_max

	# republish filtered scan on appropriate topic with altered values
	data.ranges      = tuple(ranges)
	data.intensities = tuple(intensities)
	data.range_min   = range_min
	data.range_max   = range_max

	pub.publish(data)

def main():
	global pub

	rospy.init_node('angle_pass_filter', anonymous=True)

	pub = rospy.Publisher('scan', LaserScan, queue_size=20)
	rospy.Subscriber('raw_scan', LaserScan, scan_listener)

	rospy.spin()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
