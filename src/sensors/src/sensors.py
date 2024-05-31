#!/usr/bin/env python3

import rospy
import psutil
from std_msgs.msg import String
import time
import os
import json
# import pynvml

battery_level = None

def get_kernel_info():
    return {
        "kernel_version": os.uname().release,
        "system_name": os.uname().sysname,
        "node_name": os.uname().nodename,
        "machine": os.uname().machine
    }

def get_memory_info():
    return {
        "total_memory": round(psutil.virtual_memory().total / (1024.0 ** 3), 2),
        "available_memory": round(psutil.virtual_memory().available / (1024.0 ** 3), 2),
        "used_memory": round(psutil.virtual_memory().used / (1024.0 ** 3), 2),
        "memory_percentage": psutil.virtual_memory().percent
    }

def get_cpu_info():
    return {
        "physical_cores": psutil.cpu_count(logical=False),
        "total_cores": psutil.cpu_count(logical=True),
        "processor_speed": psutil.cpu_freq().current,
        "cpu_usage_per_core": dict(enumerate(psutil.cpu_percent(percpu=True))),
        "total_cpu_usage": psutil.cpu_percent()
    }

def get_disk_info():
    total_usage = psutil.disk_usage('/')
    return {
        "total_space": round(total_usage.total / (1024.0 ** 3), 2),
        "used_space": round(total_usage.used / (1024.0 ** 3), 2),
        "free_space": round(total_usage.free / (1024.0 ** 3), 2),
        "usage_percentage": total_usage.percent
    }

def get_system_uptime():
    boot_time_timestamp = psutil.boot_time()
    current_time_timestamp = time.time()
    uptime_seconds = current_time_timestamp - boot_time_timestamp
    uptime_minutes = uptime_seconds // 60
    uptime_hours = uptime_minutes // 60
    uptime_days = uptime_hours // 24
    uptime_str = f"{int(uptime_days)} days, {int(uptime_hours % 24)} hours, {int(uptime_minutes % 60)} minutes, {int(uptime_seconds % 60)} seconds"
    return {"uptime": uptime_str}

def get_network_usage():
    # wifi_interface = "wlp5s0"  # Change this to your Wi-Fi interface name
    wifi_interface = "wlan0"
    net_stat = psutil.net_io_counters(pernic=True, nowrap=True)[wifi_interface]
    net_in_1 = net_stat.bytes_recv
    net_out_1 = net_stat.bytes_sent
    time.sleep(1)
    net_stat = psutil.net_io_counters(pernic=True, nowrap=True)[wifi_interface]
    net_in_2 = net_stat.bytes_recv
    net_out_2 = net_stat.bytes_sent

    megabytes_received_per_second = round((net_in_2 - net_in_1) / 1024 / 1024, 3)
    megabytes_sent_per_second = round((net_out_2 - net_out_1) / 1024 / 1024, 3)

    return {
        "megabytes_sent_per_second": megabytes_sent_per_second,
        "megabytes_received_per_second": megabytes_received_per_second
    }

# def get_gpu_info():
#     pynvml.nvmlInit()
#     gpu_count = pynvml.nvmlDeviceGetCount()
#     gpu_info = []

#     for i in range(gpu_count):
#         handle = pynvml.nvmlDeviceGetHandleByIndex(i)
#         name = pynvml.nvmlDeviceGetName(handle).decode("utf-8")
#         memory_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
#         utilization_info = pynvml.nvmlDeviceGetUtilizationRates(handle)
#         temperature = pynvml.nvmlDeviceGetTemperature(handle, 0)

#         gpu_info.append({
#             "index": i,
#             "name": name,
#             "memory_total": memory_info.total / 1024**2,  # Convert to MiB
#             "memory_used": memory_info.used / 1024**2,  # Convert to MiB
#             "memory_free": memory_info.free / 1024**2,  # Convert to MiB
#             "utilization_gpu": utilization_info.gpu,
#             "utilization_memory": utilization_info.memory,
#             "temperature": temperature,  # in Celsius
#         })

#     pynvml.nvmlShutdown()
#     return gpu_info

def get_battery_level(_message):
    global battery_level
    battery_level = _message.data.split(" ")[-1][:-1]

def publish_sensor_readings():
    global battery_level

    rospy.init_node('sensors_node', anonymous=True)
    pub = rospy.Publisher('/sensors_topic', String, queue_size = 10)
    rospy.Subscriber("/battery_level", String, get_battery_level)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        data = {
            "kernel_info": get_kernel_info(),
            "cpu_info": get_cpu_info(),
            "battery_level": battery_level,
            "memory_info": get_memory_info(),
            "disk_info": get_disk_info(),
            "internet_speed": get_network_usage(),
            "system_uptime": get_system_uptime(),
        }

        sensor_readings = json.dumps(data, indent=4)
        pub.publish(sensor_readings)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_sensor_readings()
    except rospy.ROSInterruptException:
        pass
