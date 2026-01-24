'use client';

import { 
  Terminal, Battery, Cpu, Server, Play, Check, ChevronRight, 
  BookOpen, Layout, Settings, Code, ArrowRight, AlertTriangle, 
  Network, Search, Award, Users, Wrench, Package, FolderTree,
  Lightbulb, Zap, Target, Layers, HelpCircle, Bot, Cog,
  FileCode, GitBranch, Box, Rocket, Radio, Send, Inbox,
  MessageSquare, RefreshCw, Eye, Activity, Filter, Gauge,
  CircuitBoard, Workflow, Combine, BarChart3, Sparkles
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture3() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Lecture 3: Data Processing', icon: BookOpen },
    { id: 'slide-2', title: 'Homework Review', icon: Check },
    { id: 'slide-3', title: 'Speed Sensor Solution', icon: Code },
    { id: 'slide-4', title: 'Multi-Topic Dashboard', icon: Code },
    { id: 'slide-5', title: 'Safety Monitor Solution', icon: Code },
    { id: 'slide-6', title: 'Review: Topics So Far', icon: RefreshCw },
    { id: 'slide-7', title: 'The New Challenge', icon: HelpCircle },
    { id: 'slide-8', title: 'Data Processing Pipelines', icon: Workflow },
    { id: 'slide-9', title: 'The Filter Node Concept', icon: Filter },
    { id: 'slide-10', title: 'Moving Average Filter', icon: BarChart3 },
    { id: 'slide-11', title: 'Sensor Filter Code', icon: Code },
    { id: 'slide-12', title: 'Understanding the Filter', icon: Search },
    { id: 'slide-13', title: 'Running the Pipeline', icon: Play },
    { id: 'slide-14', title: 'The Aggregator Concept', icon: Combine },
    { id: 'slide-15', title: 'Custom Message Types', icon: MessageSquare },
    { id: 'slide-16', title: 'Status Aggregator Code', icon: Code },
    { id: 'slide-17', title: 'Updated Dashboard', icon: Gauge },
    { id: 'slide-18', title: 'The Complete System', icon: Network },
    { id: 'slide-19', title: 'Debugging with CLI', icon: Terminal },
    { id: 'slide-20', title: 'Key Patterns', icon: Lightbulb },
    { id: 'slide-21', title: 'Summary', icon: Layout },
    { id: 'slide-22', title: 'Homework', icon: Battery },
    { id: 'slide-23', title: 'Next Lecture', icon: ArrowRight },
  ];

  const speedSensorCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SpeedSensor(Node):
    def __init__(self):
        super().__init__('speed_sensor')
        
        self.publisher = self.create_publisher(Float32, 'robot/speed', 10)
        self.timer = self.create_timer(0.5, self.publish_speed)
        
        self.base_speed = 1.0  # meters per second
        
        self.get_logger().info("Speed Sensor started!")
    
    def publish_speed(self):
        msg = Float32()
        # Add some noise to simulate real sensor
        noise = random.uniform(-0.3, 0.3)
        msg.data = self.base_speed + noise
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Speed: {msg.data:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = SpeedSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const multiTopicDashboardCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard')
        
        # Store latest values
        self.battery_level = 0.0
        self.speed = 0.0
        
        # Subscribe to BOTH topics
        self.battery_sub = self.create_subscription(
            Float32, 'battery_level', self.battery_callback, 10
        )
        
        self.speed_sub = self.create_subscription(
            Float32, 'robot/speed', self.speed_callback, 10
        )
        
        self.get_logger().info("Dashboard started! Waiting for data...")
    
    def battery_callback(self, msg):
        self.battery_level = msg.data
        self.display_status()
    
    def speed_callback(self, msg):
        self.speed = msg.data
        self.display_status()
    
    def display_status(self):
        # Simple display - just print both values
        self.get_logger().info(
            f"Battery: {self.battery_level:.1f}% | Speed: {self.speed:.2f} m/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const safetyMonitorCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        self.subscription = self.create_subscription(
            Float32, 'battery_level', self.check_battery, 10
        )
        
        self.get_logger().info("Safety Monitor active!")
    
    def check_battery(self, msg):
        if msg.data < 20.0:
            self.get_logger().warn(f"LOW BATTERY: {msg.data:.1f}%")
        elif msg.data < 10.0:
            self.get_logger().error(f"CRITICAL BATTERY: {msg.data:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const sensorFilterCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from collections import deque

class SensorFilter(Node):
    def __init__(self):
        super().__init__('sensor_filter')
        
        # Moving average window
        self.window_size = 5
        self.readings = deque(maxlen=self.window_size)
        
        # Subscribe to raw speed
        self.subscription = self.create_subscription(
            Float32, 'robot/speed', self.filter_callback, 10
        )
        
        # Publish filtered speed
        self.publisher = self.create_publisher(
            Float32, 'robot/speed_filtered', 10
        )
        
        self.get_logger().info("Sensor Filter started!")
    
    def filter_callback(self, msg):
        # Add new reading to window
        self.readings.append(msg.data)
        
        # Calculate moving average
        if len(self.readings) > 0:
            filtered_value = sum(self.readings) / len(self.readings)
            
            # Publish filtered value
            filtered_msg = Float32()
            filtered_msg.data = filtered_value
            self.publisher.publish(filtered_msg)
            
            self.get_logger().info(
                f"Raw: {msg.data:.2f} -> Filtered: {filtered_value:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = SensorFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const statusAggregatorCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class StatusAggregator(Node):
    def __init__(self):
        super().__init__('status_aggregator')
        
        # Store latest values
        self.battery = None
        self.speed = None
        
        # Subscribe to multiple topics
        self.battery_sub = self.create_subscription(
            Float32, 'battery_level', self.battery_callback, 10
        )
        
        self.speed_sub = self.create_subscription(
            Float32, 'robot/speed_filtered', self.speed_callback, 10
        )
        
        # Publish combined status
        self.status_pub = self.create_publisher(String, 'robot/status', 10)
        
        # Publish at regular interval
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Status Aggregator started!")
    
    def battery_callback(self, msg):
        self.battery = msg.data
    
    def speed_callback(self, msg):
        self.speed = msg.data
    
    def publish_status(self):
        if self.battery is None or self.speed is None:
            return
        
        # Determine overall status
        if self.battery < 20:
            status = "CRITICAL"
        elif self.battery < 50:
            status = "WARNING"
        else:
            status = "OK"
        
        # Create combined message
        msg = String()
        msg.data = f"[{status}] Battery: {self.battery:.1f}% | Speed: {self.speed:.2f} m/s"
        
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = StatusAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const updatedDashboardCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard')
        
        self.battery = None
        self.raw_speed = None
        self.filtered_speed = None
        
        # Subscribe to all relevant topics
        self.create_subscription(Float32, 'battery_level', self.battery_cb, 10)
        self.create_subscription(Float32, 'robot/speed', self.raw_speed_cb, 10)
        self.create_subscription(Float32, 'robot/speed_filtered', self.filtered_speed_cb, 10)
        self.create_subscription(String, 'robot/status', self.status_cb, 10)
        
        self.get_logger().info("Dashboard v2 started!")
    
    def battery_cb(self, msg):
        self.battery = msg.data
    
    def raw_speed_cb(self, msg):
        self.raw_speed = msg.data
    
    def filtered_speed_cb(self, msg):
        self.filtered_speed = msg.data
        self.display()
    
    def status_cb(self, msg):
        self.get_logger().info(f"Status: {msg.data}")
    
    def display(self):
        self.get_logger().info(
            f"Raw: {self.raw_speed:.2f} | Filtered: {self.filtered_speed:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={3} 
        lectureTitle="Data Processing" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Title */}
        <LectureSlide id="slide-1" title="Multiple Subscribers & Data Processing" subtitle="Building Complex Topic-Based Systems" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Lecture</h4>
                <p className="font-medium text-zinc-900">3 of 12</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Focus</h4>
                <p className="font-medium text-zinc-900">Data Pipelines &amp; Filtering</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Goal</h4>
                <p className="font-medium text-zinc-900">Process and combine sensor data</p>
              </div>
            </div>
            <div className="mt-8 pt-8 border-t border-zinc-200">
              <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-4">Today&apos;s Agenda</h4>
              <div className="grid md:grid-cols-4 gap-4">
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">✅</div>
                  <div className="text-sm font-medium">Homework Review</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">🔧</div>
                  <div className="text-sm font-medium">Filtering Data</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">🔗</div>
                  <div className="text-sm font-medium">Aggregating Data</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">🌐</div>
                  <div className="text-sm font-medium">Building Pipelines</div>
                </div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 2: Homework Review */}
        <LectureSlide id="slide-2" title="Homework Review" subtitle="What You Should Have Built" icon={Check}>
          <div className="mb-8">
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h3 className="font-bold text-green-900 mb-4">📋 The Assignments Were:</h3>
              <div className="grid md:grid-cols-3 gap-4">
                <div className="p-4 bg-white rounded-lg">
                  <div className="font-bold text-green-900">1. speed_sensor</div>
                  <p className="text-green-800 text-sm">Publishes speed on <code className="bg-green-100 px-1 rounded">/robot/speed</code></p>
                </div>
                <div className="p-4 bg-white rounded-lg">
                  <div className="font-bold text-green-900">2. Updated dashboard</div>
                  <p className="text-green-800 text-sm">Subscribes to both battery AND speed</p>
                </div>
                <div className="p-4 bg-white rounded-lg">
                  <div className="font-bold text-green-900">3. safety_monitor</div>
                  <p className="text-green-800 text-sm">Warns if battery &lt; 20%</p>
                </div>
              </div>
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">🔑 Key Learning: One Node, Multiple Subscribers</h4>
            <p className="text-blue-800">
              A single node can subscribe to as many topics as it needs. The dashboard subscribes to both <code className="bg-white px-1 rounded">battery_level</code> and <code className="bg-white px-1 rounded">robot/speed</code>. Each subscription has its own callback function.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 3: Speed Sensor Solution */}
        <LectureSlide id="slide-3" title="Speed Sensor Solution" subtitle="Publishing Speed Data" icon={Code}>
          <CodeBlock 
            filename="speed_sensor.py"
            code={speedSensorCode}
          />
          
          <div className="mt-6">
            <TerminalBlock 
              command="ros2 run my_robot_pkg speed_sensor"
              output={`[INFO] [speed_sensor]: Speed Sensor started!
[INFO] [speed_sensor]: Speed: 1.23 m/s
[INFO] [speed_sensor]: Speed: 0.87 m/s
[INFO] [speed_sensor]: Speed: 1.15 m/s`}
              title="Terminal Output"
            />
          </div>
          
          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">💡 Notice the Noise</h4>
            <p className="text-amber-800 text-sm">
              We added random noise to simulate a real sensor. Real sensors are never perfectly accurate. This noise will become important later when we learn to filter it out.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 4: Multi-Topic Dashboard */}
        <LectureSlide id="slide-4" title="Dashboard with Multiple Subscriptions" subtitle="Listening to Multiple Topics" icon={Code}>
          <CodeBlock 
            filename="dashboard.py"
            code={multiTopicDashboardCode}
          />
          
          <div className="mt-6">
            <TerminalBlock 
              command="ros2 run my_robot_pkg dashboard"
              output={`[INFO] [dashboard]: Dashboard started! Waiting for data...
[INFO] [dashboard]: Battery: 100.0% | Speed: 0.00 m/s
[INFO] [dashboard]: Battery: 100.0% | Speed: 1.23 m/s
[INFO] [dashboard]: Battery: 99.5% | Speed: 1.23 m/s
[INFO] [dashboard]: Battery: 99.5% | Speed: 0.87 m/s`}
              title="Terminal Output"
            />
          </div>
          
          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">✅ Two Subscribers</h4>
              <p className="text-green-800 text-sm">Each with its own callback function</p>
            </div>
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">✅ Shared State</h4>
              <p className="text-blue-800 text-sm">Both callbacks can access <code>self.battery_level</code> and <code>self.speed</code></p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 5: Safety Monitor Solution */}
        <LectureSlide id="slide-5" title="Safety Monitor Solution" subtitle="Reacting to Conditions" icon={Code}>
          <CodeBlock 
            filename="safety_monitor.py"
            code={safetyMonitorCode}
          />
          
          <div className="mt-6">
            <TerminalBlock 
              command="ros2 run my_robot_pkg safety_monitor"
              output={`[INFO] [safety_monitor]: Safety Monitor active!
[WARN] [safety_monitor]: LOW BATTERY: 18.5%
[WARN] [safety_monitor]: LOW BATTERY: 18.0%
[WARN] [safety_monitor]: LOW BATTERY: 17.5%`}
              title="Terminal Output (when battery drops below 20%)"
            />
          </div>
          
          <div className="mt-6 p-4 bg-blue-50 border border-blue-100 rounded-lg">
            <h4 className="font-bold text-blue-900 mb-2">🔍 Different Log Levels</h4>
            <div className="grid md:grid-cols-3 gap-4 mt-4 text-sm">
              <div className="p-3 bg-white rounded">
                <code className="text-blue-600">get_logger().info()</code>
                <p className="text-zinc-600 mt-1">Normal information</p>
              </div>
              <div className="p-3 bg-white rounded">
                <code className="text-yellow-600">get_logger().warn()</code>
                <p className="text-zinc-600 mt-1">Warning, needs attention</p>
              </div>
              <div className="p-3 bg-white rounded">
                <code className="text-red-600">get_logger().error()</code>
                <p className="text-zinc-600 mt-1">Error, something is wrong</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 6: Review Topics */}
        <LectureSlide id="slide-6" title="Review: What We Know About Topics" subtitle="Quick Recap" icon={RefreshCw}>
          <div className="grid md:grid-cols-2 gap-8">
            <div className="space-y-4">
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h4 className="font-bold text-green-900">✅ Publishers</h4>
                <p className="text-green-800 text-sm">Send data to a topic using <code>create_publisher()</code> and <code>publish()</code></p>
              </div>
              <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
                <h4 className="font-bold text-blue-900">✅ Subscribers</h4>
                <p className="text-blue-800 text-sm">Receive data via callback with <code>create_subscription()</code></p>
              </div>
              <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg">
                <h4 className="font-bold text-purple-900">✅ Message Types</h4>
                <p className="text-purple-800 text-sm">Both sides must use the same interface (Float32, String, etc.)</p>
              </div>
            </div>
            <div className="space-y-4">
              <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
                <h4 className="font-bold text-yellow-900">✅ Multiple Subscribers</h4>
                <p className="text-yellow-800 text-sm">Many nodes can subscribe to the same topic</p>
              </div>
              <div className="p-4 bg-orange-50 border border-orange-200 rounded-lg">
                <h4 className="font-bold text-orange-900">✅ Anonymous</h4>
                <p className="text-orange-800 text-sm">Publishers and subscribers don&apos;t know about each other</p>
              </div>
              <div className="p-4 bg-zinc-100 border border-zinc-200 rounded-lg">
                <h4 className="font-bold text-zinc-900">✅ One Node, Many Topics</h4>
                <p className="text-zinc-700 text-sm">A node can publish AND subscribe to multiple topics</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 7: The New Challenge */}
        <LectureSlide id="slide-7" title="The New Challenge" subtitle="Real Sensors Are Noisy" icon={HelpCircle}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Look at Our Speed Sensor Output:</h3>
            <TerminalBlock 
              command="ros2 run my_robot_pkg speed_sensor"
              output={`[INFO] [speed_sensor]: Speed: 1.23 m/s
[INFO] [speed_sensor]: Speed: 0.87 m/s
[INFO] [speed_sensor]: Speed: 1.15 m/s
[INFO] [speed_sensor]: Speed: 0.72 m/s
[INFO] [speed_sensor]: Speed: 1.28 m/s`}
              title="Terminal"
            />
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <h4 className="font-bold text-red-900 mb-4">❌ The Problem</h4>
              <ul className="space-y-2 text-red-800">
                <li>• The robot is moving at ~1.0 m/s</li>
                <li>• But readings jump between 0.7 and 1.3</li>
                <li>• This is sensor <strong>noise</strong></li>
                <li>• If we use this directly, decisions will be erratic</li>
              </ul>
            </div>
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-4">✅ The Solution</h4>
              <ul className="space-y-2 text-green-800">
                <li>• Create a <strong>filter node</strong></li>
                <li>• It subscribes to raw data</li>
                <li>• Applies smoothing algorithm</li>
                <li>• Publishes cleaned data on new topic</li>
              </ul>
            </div>
          </div>

          <div className="text-center p-6 bg-zinc-900 text-white rounded-xl">
            <p className="text-xl">We need a <span className="text-yellow-400 font-bold">data processing pipeline</span>.</p>
          </div>
        </LectureSlide>

        {/* Slide 8: Data Processing Pipelines */}
        <LectureSlide id="slide-8" title="Data Processing Pipelines" subtitle="Nodes as an Assembly Line" icon={Workflow}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">The Assembly Line Pattern</h3>
            <p className="text-lg text-zinc-600">Each node does <strong>one job well</strong>, then passes data to the next.</p>
          </div>

          <div className="p-8 bg-zinc-50 rounded-xl border border-zinc-200 mb-8">
            <div className="flex flex-col md:flex-row items-center justify-between gap-4">
              <div className="p-4 bg-white border-2 border-blue-300 rounded-lg text-center">
                <div className="text-2xl mb-2">📡</div>
                <div className="font-bold">speed_sensor</div>
                <div className="text-xs text-zinc-500">Raw data</div>
              </div>
              <div className="text-zinc-400">
                <ArrowRight size={24} />
              </div>
              <div className="p-2 bg-zinc-200 rounded text-xs font-mono">/robot/speed</div>
              <div className="text-zinc-400">
                <ArrowRight size={24} />
              </div>
              <div className="p-4 bg-white border-2 border-green-300 rounded-lg text-center">
                <div className="text-2xl mb-2">🔧</div>
                <div className="font-bold">sensor_filter</div>
                <div className="text-xs text-zinc-500">Smooths data</div>
              </div>
              <div className="text-zinc-400">
                <ArrowRight size={24} />
              </div>
              <div className="p-2 bg-zinc-200 rounded text-xs font-mono">/robot/speed_filtered</div>
              <div className="text-zinc-400">
                <ArrowRight size={24} />
              </div>
              <div className="p-4 bg-white border-2 border-purple-300 rounded-lg text-center">
                <div className="text-2xl mb-2">📊</div>
                <div className="font-bold">dashboard</div>
                <div className="text-xs text-zinc-500">Displays</div>
              </div>
            </div>
          </div>

          <div className="grid md:grid-cols-3 gap-6">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg text-center">
              <h4 className="font-bold text-blue-900">Stage 1: Collect</h4>
              <p className="text-blue-800 text-sm">Sensor reads raw data</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg text-center">
              <h4 className="font-bold text-green-900">Stage 2: Process</h4>
              <p className="text-green-800 text-sm">Filter cleans the data</p>
            </div>
            <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg text-center">
              <h4 className="font-bold text-purple-900">Stage 3: Use</h4>
              <p className="text-purple-800 text-sm">Other nodes consume clean data</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 9: Filter Node Concept */}
        <LectureSlide id="slide-9" title="The Filter Node" subtitle="Subscribe, Process, Publish" icon={Filter}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl">
            <h3 className="text-xl font-bold text-yellow-400 mb-4">🔑 The Key Pattern</h3>
            <p className="text-lg">A processing node is both a <span className="text-blue-400">subscriber</span> AND a <span className="text-green-400">publisher</span>.</p>
          </div>

          <div className="grid md:grid-cols-3 gap-8">
            <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl">
              <div className="w-12 h-12 bg-blue-100 rounded-full flex items-center justify-center mb-4">
                <Inbox size={24} className="text-blue-600" />
              </div>
              <h4 className="font-bold text-zinc-900 mb-2">1. Subscribe</h4>
              <p className="text-zinc-600 text-sm">Receive raw data from upstream topic</p>
              <code className="text-xs bg-white px-2 py-1 rounded block mt-2">/robot/speed</code>
            </div>
            
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <div className="w-12 h-12 bg-green-100 rounded-full flex items-center justify-center mb-4">
                <Cog size={24} className="text-green-600" />
              </div>
              <h4 className="font-bold text-zinc-900 mb-2">2. Process</h4>
              <p className="text-zinc-600 text-sm">Apply algorithm in callback</p>
              <code className="text-xs bg-white px-2 py-1 rounded block mt-2">moving_average()</code>
            </div>
            
            <div className="p-6 bg-purple-50 border border-purple-200 rounded-xl">
              <div className="w-12 h-12 bg-purple-100 rounded-full flex items-center justify-center mb-4">
                <Send size={24} className="text-purple-600" />
              </div>
              <h4 className="font-bold text-zinc-900 mb-2">3. Publish</h4>
              <p className="text-zinc-600 text-sm">Send processed data downstream</p>
              <code className="text-xs bg-white px-2 py-1 rounded block mt-2">/robot/speed_filtered</code>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 10: Moving Average */}
        <LectureSlide id="slide-10" title="Moving Average Filter" subtitle="A Simple Smoothing Algorithm" icon={BarChart3}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">How It Works</h3>
            <p className="text-lg text-zinc-600">Instead of using the latest value, use the <strong>average of the last N values</strong>.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">Example (window size = 3):</h4>
              <div className="space-y-2 font-mono text-sm">
                <div className="flex justify-between p-2 bg-white rounded">
                  <span>Reading 1:</span>
                  <span>1.2</span>
                  <span className="text-green-600">→ avg: 1.20</span>
                </div>
                <div className="flex justify-between p-2 bg-white rounded">
                  <span>Reading 2:</span>
                  <span>0.8</span>
                  <span className="text-green-600">→ avg: 1.00</span>
                </div>
                <div className="flex justify-between p-2 bg-white rounded">
                  <span>Reading 3:</span>
                  <span>1.1</span>
                  <span className="text-green-600">→ avg: 1.03</span>
                </div>
                <div className="flex justify-between p-2 bg-white rounded">
                  <span>Reading 4:</span>
                  <span>0.7</span>
                  <span className="text-green-600">→ avg: 0.87</span>
                </div>
                <div className="flex justify-between p-2 bg-white rounded">
                  <span>Reading 5:</span>
                  <span>1.3</span>
                  <span className="text-green-600">→ avg: 1.03</span>
                </div>
              </div>
            </div>
            
            <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-4">Why This Works</h4>
              <ul className="space-y-3 text-blue-800">
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-2 mt-1" />
                  <span>Random noise tends to cancel out</span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-2 mt-1" />
                  <span>True signal remains stable</span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-2 mt-1" />
                  <span>Larger window = smoother but slower</span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-2 mt-1" />
                  <span>Smaller window = faster but noisier</span>
                </li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 11: Sensor Filter Code */}
        <LectureSlide id="slide-11" title="Sensor Filter Code" subtitle="The Complete Implementation" icon={Code}>
          <CodeBlock 
            filename="sensor_filter.py"
            code={sensorFilterCode}
          />
        </LectureSlide>

        {/* Slide 12: Understanding the Filter */}
        <LectureSlide id="slide-12" title="Understanding the Code" subtitle="Key Parts Explained" icon={Search}>
          <div className="space-y-6">
            <div className="p-4 bg-purple-50 border-l-4 border-purple-500 rounded-r-lg">
              <h4 className="font-bold text-purple-900 mb-2">from collections import deque</h4>
              <p className="text-purple-800 text-sm">A deque with <code>maxlen</code> automatically removes old values when new ones are added. Perfect for a sliding window.</p>
            </div>

            <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
              <h4 className="font-bold text-blue-900 mb-2">self.readings = deque(maxlen=5)</h4>
              <p className="text-blue-800 text-sm">Keep only the last 5 readings. When a 6th arrives, the oldest is automatically dropped.</p>
            </div>

            <div className="p-4 bg-green-50 border-l-4 border-green-500 rounded-r-lg">
              <h4 className="font-bold text-green-900 mb-2">Subscriber + Publisher in one node</h4>
              <p className="text-green-800 text-sm">We have <code>create_subscription()</code> for input AND <code>create_publisher()</code> for output.</p>
            </div>

            <div className="p-4 bg-yellow-50 border-l-4 border-yellow-500 rounded-r-lg">
              <h4 className="font-bold text-yellow-900 mb-2">Processing in the callback</h4>
              <p className="text-yellow-800 text-sm">The callback receives raw data, calculates the average, and immediately publishes the result.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 13: Running the Pipeline */}
        <LectureSlide id="slide-13" title="Running the Pipeline" subtitle="See the Filtering in Action" icon={Play}>
          <div className="mb-6">
            <h4 className="font-bold text-zinc-900 mb-3">Run three terminals:</h4>
          </div>

          <div className="space-y-4 mb-8">
            <TerminalBlock 
              command="ros2 run my_robot_pkg speed_sensor"
              output={`[INFO] [speed_sensor]: Speed Sensor started!
[INFO] [speed_sensor]: Speed: 1.23 m/s
[INFO] [speed_sensor]: Speed: 0.87 m/s
[INFO] [speed_sensor]: Speed: 1.15 m/s
[INFO] [speed_sensor]: Speed: 0.72 m/s
[INFO] [speed_sensor]: Speed: 1.28 m/s
[INFO] [speed_sensor]: Speed: 0.95 m/s`}
              title="Terminal 1: Speed Sensor"
            />

            <TerminalBlock 
              command="ros2 run my_robot_pkg sensor_filter"
              output={`[INFO] [sensor_filter]: Sensor Filter started!
[INFO] [sensor_filter]: Raw: 1.23 -> Filtered: 1.23
[INFO] [sensor_filter]: Raw: 0.87 -> Filtered: 1.05
[INFO] [sensor_filter]: Raw: 1.15 -> Filtered: 1.08
[INFO] [sensor_filter]: Raw: 0.72 -> Filtered: 0.99
[INFO] [sensor_filter]: Raw: 1.28 -> Filtered: 1.05
[INFO] [sensor_filter]: Raw: 0.95 -> Filtered: 0.99`}
              title="Terminal 2: Filter"
            />

            <TerminalBlock 
              command="ros2 topic echo /robot/speed_filtered"
              output={`data: 1.05
---
data: 1.08
---
data: 0.99
---`}
              title="Terminal 3: Verify Output"
            />
          </div>

          <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800">
              <strong>🎉 Notice:</strong> The filtered values are much more stable than the raw readings!
            </p>
          </div>
        </LectureSlide>

        {/* Slide 14: Aggregator Concept */}
        <LectureSlide id="slide-14" title="The Aggregator Pattern" subtitle="Combining Multiple Data Sources" icon={Combine}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Another Common Pattern</h3>
            <p className="text-lg text-zinc-600">Combine data from <strong>multiple topics</strong> into a <strong>single topic</strong>.</p>
          </div>

          <div className="p-8 bg-zinc-50 rounded-xl border border-zinc-200 mb-8">
            <div className="flex flex-col items-center gap-4">
              <div className="flex gap-8">
                <div className="p-4 bg-white border-2 border-blue-300 rounded-lg text-center">
                  <div className="font-mono text-xs">/battery_level</div>
                </div>
                <div className="p-4 bg-white border-2 border-green-300 rounded-lg text-center">
                  <div className="font-mono text-xs">/robot/speed_filtered</div>
                </div>
              </div>
              <div className="text-zinc-400">
                <ArrowRight size={24} className="rotate-90" />
              </div>
              <div className="p-4 bg-purple-100 border-2 border-purple-400 rounded-lg text-center">
                <div className="text-2xl mb-2">🔗</div>
                <div className="font-bold">status_aggregator</div>
              </div>
              <div className="text-zinc-400">
                <ArrowRight size={24} className="rotate-90" />
              </div>
              <div className="p-4 bg-white border-2 border-orange-300 rounded-lg text-center">
                <div className="font-mono text-xs">/robot/status</div>
              </div>
            </div>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">Why Aggregate?</h4>
              <ul className="text-blue-800 text-sm space-y-1">
                <li>• Single source of truth for robot status</li>
                <li>• Easier for other nodes to consume</li>
                <li>• Can add logic (determine overall health)</li>
              </ul>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">The Pattern</h4>
              <ul className="text-green-800 text-sm space-y-1">
                <li>• Subscribe to multiple input topics</li>
                <li>• Store latest values in class variables</li>
                <li>• Publish combined output periodically</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 15: Custom Message Types */}
        <LectureSlide id="slide-15" title="Using String for Combined Data" subtitle="A Simple Approach" icon={MessageSquare}>
          <div className="mb-8">
            <div className="p-6 bg-amber-50 border border-amber-200 rounded-xl">
              <h4 className="font-bold text-amber-900 mb-2">📝 For Now: String Messages</h4>
              <p className="text-amber-800">
                We&apos;ll use <code className="bg-white px-2 py-1 rounded">std_msgs/msg/String</code> to publish combined status. Later in the course, you&apos;ll learn to create <strong>custom message types</strong> with proper fields.
              </p>
            </div>
          </div>

          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">String Approach (Today)</h4>
              <div className="p-4 bg-white rounded-lg font-mono text-sm">
                <div className="text-zinc-500"># Message content:</div>
                <div className="text-green-600">&quot;[OK] Battery: 85.0% | Speed: 1.02 m/s&quot;</div>
              </div>
              <p className="text-zinc-600 text-sm mt-4">Simple but hard to parse programmatically</p>
            </div>
            
            <div className="p-6 bg-zinc-50 rounded-xl opacity-60">
              <h4 className="font-bold text-zinc-900 mb-4">Custom Message (Later)</h4>
              <div className="p-4 bg-white rounded-lg font-mono text-sm">
                <div className="text-zinc-500"># Custom RobotStatus.msg:</div>
                <div>float32 battery</div>
                <div>float32 speed</div>
                <div>string status</div>
              </div>
              <p className="text-zinc-600 text-sm mt-4">Structured and easy to use</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 16: Status Aggregator Code */}
        <LectureSlide id="slide-16" title="Status Aggregator Code" subtitle="Combining Battery and Speed" icon={Code}>
          <CodeBlock 
            filename="status_aggregator.py"
            code={statusAggregatorCode}
          />
          
          <div className="mt-6">
            <TerminalBlock 
              command="ros2 run my_robot_pkg status_aggregator"
              output={`[INFO] [status_aggregator]: Status Aggregator started!
[INFO] [status_aggregator]: [OK] Battery: 85.0% | Speed: 1.02 m/s
[INFO] [status_aggregator]: [OK] Battery: 84.5% | Speed: 0.98 m/s
[INFO] [status_aggregator]: [WARNING] Battery: 45.0% | Speed: 1.05 m/s
[INFO] [status_aggregator]: [CRITICAL] Battery: 18.0% | Speed: 0.95 m/s`}
              title="Terminal Output"
            />
          </div>
        </LectureSlide>

        {/* Slide 17: Updated Dashboard */}
        <LectureSlide id="slide-17" title="Updated Dashboard" subtitle="Subscribing to Everything" icon={Gauge}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">Now our dashboard can subscribe to raw speed, filtered speed, battery, AND the combined status.</p>
          </div>

          <CodeBlock 
            filename="dashboard_v2.py"
            code={updatedDashboardCode}
          />

          <div className="mt-6">
            <TerminalBlock 
              command="ros2 run my_robot_pkg dashboard"
              output={`[INFO] [dashboard]: Dashboard v2 started!
[INFO] [dashboard]: Raw: 1.23 | Filtered: 1.08
[INFO] [dashboard]: Raw: 0.87 | Filtered: 1.02
[INFO] [dashboard]: Status: [OK] Battery: 85.0% | Speed: 1.02 m/s
[INFO] [dashboard]: Raw: 1.15 | Filtered: 0.99`}
              title="Terminal Output"
            />
          </div>
          
          <div className="mt-6 p-4 bg-blue-50 border border-blue-100 rounded-lg">
            <h4 className="font-bold text-blue-900 mb-2">💡 Notice</h4>
            <p className="text-blue-800 text-sm">
              Four subscribers in one node! Each has its own callback, and they all share access to the node&apos;s state.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 18: Complete System */}
        <LectureSlide id="slide-18" title="The Complete System" subtitle="All Nodes Working Together" icon={Network}>
          <div className="mb-6">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Our Robot Now Has:</h3>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl font-mono text-sm mb-8">
            <div className="grid md:grid-cols-2 gap-8">
              <div>
                <div className="text-zinc-400 mb-2"># Data Sources (Publishers)</div>
                <div className="text-green-400">battery_monitor</div>
                <div className="text-zinc-500 pl-4">→ /battery_level</div>
                <div className="text-green-400 mt-2">speed_sensor</div>
                <div className="text-zinc-500 pl-4">→ /robot/speed</div>
              </div>
              <div>
                <div className="text-zinc-400 mb-2"># Processing (Sub + Pub)</div>
                <div className="text-blue-400">sensor_filter</div>
                <div className="text-zinc-500 pl-4">← /robot/speed</div>
                <div className="text-zinc-500 pl-4">→ /robot/speed_filtered</div>
                <div className="text-blue-400 mt-2">status_aggregator</div>
                <div className="text-zinc-500 pl-4">← /battery_level, /robot/speed_filtered</div>
                <div className="text-zinc-500 pl-4">→ /robot/status</div>
              </div>
            </div>
            <div className="mt-4 pt-4 border-t border-zinc-700">
              <div className="text-zinc-400 mb-2"># Consumers (Subscribers)</div>
              <div className="text-purple-400">dashboard, safety_monitor</div>
            </div>
          </div>

          <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
            <h4 className="font-bold text-green-900 mb-2">🎉 This is a Real Robot Architecture!</h4>
            <p className="text-green-800">
              Sensors publish raw data → Processing nodes clean and combine → Consumers display and react. This is how industrial robots, drones, and self-driving cars work.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 19: Debugging with CLI */}
        <LectureSlide id="slide-19" title="Debugging Your System" subtitle="Useful CLI Commands" icon={Terminal}>
          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">See all topics in the system</h4>
              <TerminalBlock command="ros2 topic list" output={`/battery_level
/robot/speed
/robot/speed_filtered
/robot/status`} />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Check who is publishing/subscribing to a topic</h4>
              <TerminalBlock command="ros2 topic info /robot/speed -v" output={`Type: std_msgs/msg/Float32

Publisher count: 1
Node name: speed_sensor

Subscription count: 2
Node name: sensor_filter
Node name: dashboard`} />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Check publishing rate</h4>
              <TerminalBlock command="ros2 topic hz /robot/speed" output={`average rate: 2.001
min: 0.499s max: 0.501s`} />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 20: Key Patterns */}
        <LectureSlide id="slide-20" title="Key Patterns You Learned" subtitle="Remember These" icon={Lightbulb}>
          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 border-2 border-blue-200 bg-blue-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <Filter size={24} className="text-blue-600 mr-3" />
                <h3 className="font-bold text-zinc-900">Filter Pattern</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-4">Subscribe to raw data, process it, publish cleaned data.</p>
              <div className="p-3 bg-white rounded font-mono text-xs">
                raw_topic → [filter] → filtered_topic
              </div>
            </div>
            
            <div className="p-6 border-2 border-green-200 bg-green-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <Combine size={24} className="text-green-600 mr-3" />
                <h3 className="font-bold text-zinc-900">Aggregator Pattern</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-4">Subscribe to multiple topics, combine into one output.</p>
              <div className="p-3 bg-white rounded font-mono text-xs">
                topic_a + topic_b → [aggregate] → combined
              </div>
            </div>
            
            <div className="p-6 border-2 border-purple-200 bg-purple-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <Workflow size={24} className="text-purple-600 mr-3" />
                <h3 className="font-bold text-zinc-900">Pipeline Pattern</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-4">Chain nodes together, each doing one transformation.</p>
              <div className="p-3 bg-white rounded font-mono text-xs">
                sensor → filter → aggregator → consumer
              </div>
            </div>
            
            <div className="p-6 border-2 border-orange-200 bg-orange-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <Users size={24} className="text-orange-600 mr-3" />
                <h3 className="font-bold text-zinc-900">Multi-Subscriber</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-4">One node can subscribe to many topics simultaneously.</p>
              <div className="p-3 bg-white rounded font-mono text-xs">
                node ← topic_a, topic_b, topic_c
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 21: Summary */}
        <LectureSlide id="slide-21" title="What You Learned Today" subtitle="Lecture 3 Summary" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-8">
            <div>
              <h3 className="text-xl font-bold text-zinc-900 mb-6">Concepts Covered:</h3>
              <ul className="space-y-4">
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Processing nodes</strong> are both subscribers AND publishers</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Moving average</strong> filters out sensor noise</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Aggregators</strong> combine multiple data sources</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Pipelines</strong> chain nodes for complex processing</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>CLI tools</strong> help debug topic connections</span>
                </li>
              </ul>
            </div>
            <div className="flex flex-col justify-center p-8 bg-zinc-50 rounded-2xl border border-zinc-200">
              <h4 className="text-zinc-400 font-bold uppercase tracking-widest text-sm mb-4">Next Lecture</h4>
              <div className="text-2xl font-bold text-zinc-900 mb-2">
                Services
              </div>
              <p className="text-zinc-600">
                Request-response communication. When you need an answer, not a stream.
              </p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 22: Homework */}
        <LectureSlide id="slide-22" title="Homework" subtitle="Build a Performance Monitor" icon={Battery}>
          <div className="space-y-6">
            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">1</span>
                Create a performance_monitor node
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Subscribe to <code className="bg-zinc-100 px-2 py-1 rounded">/battery_level</code></li>
                <li>• Subscribe to <code className="bg-zinc-100 px-2 py-1 rounded">/robot/speed_filtered</code></li>
                <li>• Publish alerts on <code className="bg-zinc-100 px-2 py-1 rounded">/robot/alerts</code> (use String)</li>
              </ul>
            </div>

            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">2</span>
                Implement the logic
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• If speed &gt; 1.5 m/s AND battery &lt; 30%, publish a warning</li>
                <li>• The warning should explain the problem</li>
                <li>• Use <code className="bg-zinc-100 px-2 py-1 rounded">get_logger().warn()</code> too</li>
              </ul>
            </div>

            <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-2">🎯 Expected Output</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg performance_monitor"
                output={`[INFO] [performance_monitor]: Performance Monitor started!
[WARN] [performance_monitor]: HIGH SPEED + LOW BATTERY: 1.8m/s @ 25.0%`}
                title="When conditions are met"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 23: Next Lecture */}
        <LectureSlide id="slide-23" title="Next Lecture" subtitle="A Different Kind of Communication" icon={ArrowRight}>
          <div className="min-h-[40vh] flex flex-col items-center justify-center text-center">
            <p className="text-3xl md:text-4xl font-bold text-zinc-900 mb-8 leading-tight">
              Topics are for <span className="text-blue-600">streams</span>.<br/>
              But sometimes you need an <span className="text-green-600">answer</span>.
            </p>
            <div className="w-16 h-1 bg-zinc-200 mb-8"></div>
            <p className="text-xl text-zinc-600 mb-8">
              &quot;Where is the robot right now?&quot;<br/>
              &quot;Is this task valid?&quot;<br/>
              &quot;Reset the system.&quot;
            </p>
            
            <div className="mt-8 inline-flex items-center px-6 py-3 bg-zinc-900 text-white rounded-full text-sm font-bold tracking-wide">
              NEXT: Services - Request &amp; Response <ArrowRight size={16} className="ml-2" />
            </div>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
