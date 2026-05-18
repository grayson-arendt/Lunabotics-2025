#!/usr/bin/env python3
import os
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from lunabot_logger import Logger


class BandwidthMonitor(Node):
    def __init__(self):
        super().__init__("bandwidth_monitor")
        self.log = Logger(self)

        self.declare_parameter("interface", "auto")
        self.declare_parameter("update_rate", 0.5)
        self.declare_parameter("verbose", True)
        self.declare_parameter("log_interval", 5.0)
        self.declare_parameter("ping_target", "192.168.0.250")
        self.declare_parameter("ping_interval", 5.0)
        self.declare_parameter("latency_threshold", 100.0)
        self.declare_parameter("enable_latency_monitoring", True)

        interface = self.get_parameter("interface").value
        update_rate = self.get_parameter("update_rate").value
        self.verbose = self.get_parameter("verbose").value
        self.log_interval = self.get_parameter("log_interval").value
        self.ping_target = self.get_parameter("ping_target").value
        self.ping_interval = self.get_parameter("ping_interval").value
        self.latency_threshold = self.get_parameter("latency_threshold").value
        self.enable_latency_monitoring = self.get_parameter("enable_latency_monitoring").value

        self.interface = self.detect_interface() if interface == "auto" else interface

        if not self.interface:
            self.log.failure("No network interface found!")
            return

        self.log.info(f"Monitoring interface: {self.interface}")

        self.rx_pub       = self.create_publisher(Float32, "bandwidth/rx_mbps",       10)
        self.tx_pub       = self.create_publisher(Float32, "bandwidth/tx_mbps",       10)
        self.total_pub    = self.create_publisher(Float32, "bandwidth/total_mbps",    10)
        self.avg_rx_pub   = self.create_publisher(Float32, "bandwidth/avg_rx_mbps",   10)
        self.avg_tx_pub   = self.create_publisher(Float32, "bandwidth/avg_tx_mbps",   10)
        self.avg_total_pub = self.create_publisher(Float32, "bandwidth/avg_total_mbps", 10)
        
        self.latency_pub     = self.create_publisher(Float32, "connection/latency_ms",     10)
        self.avg_latency_pub = self.create_publisher(Float32, "connection/avg_latency_ms", 10)
        self.packet_loss_pub = self.create_publisher(Float32, "connection/packet_loss_pct", 10)

        self.last_rx_bytes = self.get_bytes("rx")
        self.last_tx_bytes = self.get_bytes("tx")
        self.last_time = time.time()
        self.start_rx_bytes = self.last_rx_bytes
        self.start_tx_bytes = self.last_tx_bytes
        self.start_time = self.last_time

        self.min_total_mbps = float('inf')
        self.max_total_mbps = 0.0
        self.last_log_time = self.last_time
        self.samples_since_log = 0
        self.sum_total_mbps = 0.0

        self.latency_samples = []
        self.ping_count = 0
        self.ping_failures = 0
        self.min_latency = float('inf')
        self.max_latency = 0.0

        self.timer = self.create_timer(1.0 / update_rate, self.update)
        
        if self.enable_latency_monitoring:
            self.ping_timer = self.create_timer(self.ping_interval, self.measure_latency)
            self.log.info(f"Latency monitoring enabled - pinging {self.ping_target} every {self.ping_interval}s")
        else:
            self.log.info("Latency monitoring disabled")

        self.log.success("Bandwidth monitor started")

    def detect_interface(self):
        try:
            interfaces = os.listdir("/sys/class/net/")
            self.log.info(f"Available interfaces: {', '.join(interfaces)}")
            
            for iface in interfaces:
                if iface.startswith(("eth", "enp", "eno", "ens")):
                    self.log.info(f"Selected wired interface: {iface}")
                    return iface
            
            for iface in interfaces:
                if iface.startswith(("wlan", "wlp", "wlo")):
                    self.log.info(f"Selected wireless interface: {iface}")
                    return iface
        except Exception as e:
            self.log.failure(f"Failed to detect interface: {e}")
        return None

    def get_bytes(self, direction):
        try:
            path = f"/sys/class/net/{self.interface}/statistics/{direction}_bytes"
            with open(path, "r") as f:
                return int(f.read().strip())
        except Exception as e:
            self.log.failure(f"Failed to read {direction} bytes: {e}")
            return 0

    def measure_latency(self):
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', self.ping_target],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            self.ping_count += 1
            
            if result.returncode == 0:
                match = re.search(r'time=(\d+\.?\d*)', result.stdout)
                if match:
                    latency = float(match.group(1))
                    self.latency_samples.append(latency)
                    
                    self.min_latency = min(self.min_latency, latency)
                    self.max_latency = max(self.max_latency, latency)
                    
                    self.latency_pub.publish(Float32(data=latency))
                    
                    avg_latency = sum(self.latency_samples) / len(self.latency_samples)
                    self.avg_latency_pub.publish(Float32(data=avg_latency))
                    
                    if latency > self.latency_threshold:
                        self.log.warning(f"High latency detected: {latency:.1f} ms to {self.ping_target}")
            else:
                self.ping_failures += 1
                self.log.warning(f"Ping failed to {self.ping_target}")
            
            packet_loss = (self.ping_failures / self.ping_count) * 100.0
            self.packet_loss_pub.publish(Float32(data=packet_loss))
            
        except (subprocess.TimeoutExpired, Exception) as e:
            self.ping_failures += 1
            self.log.failure(f"Latency measurement error: {e}")

    def update(self):
        current_rx = self.get_bytes("rx")
        current_tx = self.get_bytes("tx")
        current_time = time.time()

        delta_time = current_time - self.last_time
        delta_rx = current_rx - self.last_rx_bytes
        delta_tx = current_tx - self.last_tx_bytes

        rx_mbps    = (delta_rx * 8) / (delta_time * 1_000_000)
        tx_mbps    = (delta_tx * 8) / (delta_time * 1_000_000)
        total_mbps = rx_mbps + tx_mbps

        total_time = current_time - self.start_time
        if total_time > 0:
            total_rx_bytes = current_rx - self.start_rx_bytes
            total_tx_bytes = current_tx - self.start_tx_bytes

            avg_rx_mbps = (total_rx_bytes * 8) / (total_time * 1_000_000)
            avg_tx_mbps = (total_tx_bytes * 8) / (total_time * 1_000_000)
            avg_total_mbps = avg_rx_mbps + avg_tx_mbps
        else:
            avg_rx_mbps = 0.0
            avg_tx_mbps = 0.0
            avg_total_mbps = 0.0

        self.rx_pub.publish(Float32(data=rx_mbps))
        self.tx_pub.publish(Float32(data=tx_mbps))
        self.total_pub.publish(Float32(data=total_mbps))

        self.avg_rx_pub.publish(Float32(data=avg_rx_mbps))
        self.avg_tx_pub.publish(Float32(data=avg_tx_mbps))
        self.avg_total_pub.publish(Float32(data=avg_total_mbps))

        self.min_total_mbps = min(self.min_total_mbps, total_mbps)
        self.max_total_mbps = max(self.max_total_mbps, total_mbps)

        self.samples_since_log += 1
        self.sum_total_mbps += total_mbps

        if self.verbose and (current_time - self.last_log_time) >= self.log_interval:
            avg_recent = self.sum_total_mbps / self.samples_since_log if self.samples_since_log > 0 else 0.0
            
            latency_str = ""
            if self.latency_samples:
                current_latency = self.latency_samples[-1]
                avg_latency = sum(self.latency_samples) / len(self.latency_samples)
                packet_loss = (self.ping_failures / self.ping_count) * 100.0 if self.ping_count > 0 else 0.0
                latency_str = f" | Latency: {current_latency:.1f} ms (avg: {avg_latency:.1f}, min: {self.min_latency:.1f}, max: {self.max_latency:.1f}) | Packet loss: {packet_loss:.1f}%"
            
            self.log.info(
                f"Bandwidth - RX: {rx_mbps:.2f} Mbps, TX: {tx_mbps:.2f} Mbps, "
                f"Total: {total_mbps:.2f} Mbps | Recent avg: {avg_recent:.2f} Mbps | "
                f"Min: {self.min_total_mbps:.2f}, Max: {self.max_total_mbps:.2f} Mbps{latency_str}"
            )
            self.last_log_time = current_time
            self.samples_since_log = 0
            self.sum_total_mbps = 0.0

        self.last_rx_bytes = current_rx
        self.last_tx_bytes = current_tx
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = BandwidthMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
