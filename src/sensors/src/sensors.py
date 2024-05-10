#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import psutil
import time
import subprocess

prev_bytes_sent = 0
prev_bytes_recv = 0
prev_time = time.time()

try:
    import pynvml
    pynvml.nvmlInit()
    gpu_available = True
except ImportError:
    gpu_available = False

def get_cpu_utilization():
    return psutil.cpu_percent()

def get_ram_usage():
    return psutil.virtual_memory().percent

def get_disk_usage():
    return psutil.disk_usage('/').percent

def get_network_traffic():
    global prev_bytes_sent, prev_bytes_recv, prev_time

    # Get current network stats
    network_stats = psutil.net_io_counters()
    current_bytes_sent = network_stats.bytes_sent
    current_bytes_recv = network_stats.bytes_recv
    current_time = time.time()

    # Calculate time elapsed and bytes transferred since last reading
    time_elapsed = current_time - prev_time
    bytes_sent = current_bytes_sent - prev_bytes_sent
    bytes_recv = current_bytes_recv - prev_bytes_recv

    # Convert bytes to kilobytes and calculate kilobytes per second
    kb_sent_per_sec = round((bytes_sent / 1024) / time_elapsed, 2)
    kb_recv_per_sec = round((bytes_recv / 1024) / time_elapsed, 2)

    # Update previous values for next iteration
    prev_bytes_sent = current_bytes_sent
    prev_bytes_recv = current_bytes_recv
    prev_time = current_time

    return kb_sent_per_sec, kb_recv_per_sec

# def get_system_temperature():
#     try:
#         result = subprocess.run(["sensors"], capture_output=True, text=True)
#         return result.stdout.strip()  # Return the output if successful
#     except Exception as e:
#         print("Error retrieving system temperature:", e)
#         return "N/A"

# def get_battery_level():
#     try:
#         result = subprocess.run(["acpi"], capture_output=True, text=True)
#         # Parse the result and extract battery level
#         return "N/A"  # Example return statement
#     except Exception as e:
#         # print("Error retrieving battery status:", e)
#         return "N/A"


def get_wifi_signal_strength():
    try:
        result = subprocess.run(["iwconfig", "wlan0"], capture_output=True, text=True)
        if result.returncode == 0:
            return "N/A" if not result.stdout else result.stdout.strip()  # Return output if available, otherwise "N/A"
        else:
            return "N/A"
    except Exception as e:
        print("Error retrieving WiFi signal strength:", e)
        return "N/A"

    
def get_gpu_utilization():
    if gpu_available:
        try:
            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            utilization = pynvml.nvmlDeviceGetUtilizationRates(handle)
            return utilization.gpu
        except Exception as e:
            print("Error retrieving GPU utilization:", e)
            return "N/A"
    else:
        return "N/A"

def publish_sensor_readings():
    rospy.init_node('sensors_node', anonymous=True)
    pub = rospy.Publisher('/sensor_topic', String, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        sensor_readings = get_sensor_readings()
        pub.publish(sensor_readings)
        rate.sleep()

def get_sensor_readings():
    cpu_utilization = str(get_cpu_utilization())
    gpu_utilization = str(get_gpu_utilization())
    ram_usage = str(get_ram_usage())
    disk_usage = str(get_disk_usage())
    network_traffic_sent, network_traffic_received = get_network_traffic()
    wifi_signal_strength = get_wifi_signal_strength()

    ram_usage_str = ram_usage + "%" if ram_usage != "N/A" else "N/A"
    disk_usage_str = disk_usage + "%" if disk_usage != "N/A" else "N/A"

    if cpu_utilization != "N/A":
        cpu_utilization += "%"
    if gpu_utilization != "N/A":
        gpu_utilization += "%"
    if network_traffic_sent != "N/A":
        network_traffic_sent = f"{network_traffic_sent:.2f} KB/s"
    if network_traffic_received != "N/A":
        network_traffic_received = f"{network_traffic_received:.2f} KB/s"

    sensor_data_str = "{cpu:%s,gpu:%s,ram:%s,disk:%s,network_sent:%s,network_received:%s,wifi:%s}" % (
        cpu_utilization, gpu_utilization, ram_usage_str, disk_usage_str, network_traffic_sent, network_traffic_received,
        wifi_signal_strength)
    return sensor_data_str




if __name__ == "__main__":
    try:
        publish_sensor_readings()
    except rospy.ROSInterruptException:
        pass
