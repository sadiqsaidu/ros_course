'use client';

import { 
  Terminal, Battery, Cpu, Server, Play, Check, ChevronRight, 
  BookOpen, Layout, Settings, Code, ArrowRight, AlertTriangle, 
  Network, Search, Award, Users, Wrench, Package, FolderTree,
  Lightbulb, Zap, Target, Layers, HelpCircle, Bot, Cog,
  FileCode, GitBranch, Box, Rocket, Radio, Send, Inbox,
  MessageSquare, RefreshCw, Eye, Activity, Filter, Gauge,
  CircuitBoard, Workflow, Combine, BarChart3, Sparkles,
  Phone, PhoneCall, MapPin, Navigation, FileQuestion,
  ArrowLeftRight, CheckCircle, XCircle, FolderOpen, Hammer
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture4() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Lecture 4: Services', icon: BookOpen },
    { id: 'slide-2', title: 'Homework Review', icon: Check },
    { id: 'slide-3', title: 'Review: Topics', icon: RefreshCw },
    { id: 'slide-4', title: 'The Problem', icon: HelpCircle },
    { id: 'slide-5', title: 'What is a Service?', icon: PhoneCall },
    { id: 'slide-6', title: 'Topics vs Services', icon: ArrowLeftRight },
    { id: 'slide-7', title: 'Service Structure', icon: MessageSquare },
    { id: 'slide-8', title: 'What We\'re Building', icon: Target },
    { id: 'slide-9', title: 'Creating Interfaces Package', icon: Package },
    { id: 'slide-10', title: 'Package Configuration', icon: Settings },
    { id: 'slide-11', title: 'Custom Service Interface', icon: FileCode },
    { id: 'slide-12', title: 'Building the Interface', icon: Hammer },
    { id: 'slide-13', title: 'Service Server Code', icon: Code },
    { id: 'slide-14', title: 'Understanding the Server', icon: Search },
    { id: 'slide-15', title: 'Setup and Build', icon: Wrench },
    { id: 'slide-16', title: 'Testing the Server', icon: Play },
    { id: 'slide-17', title: 'Service Client Code', icon: Code },
    { id: 'slide-18', title: 'Understanding the Client', icon: Search },
    { id: 'slide-19', title: 'Running Both Nodes', icon: Activity },
    { id: 'slide-20', title: 'Speed Control Service', icon: Bot },
    { id: 'slide-21', title: 'CLI Tools', icon: Terminal },
    { id: 'slide-22', title: 'Key Takeaways', icon: Lightbulb },
    { id: 'slide-23', title: 'Summary', icon: Layout },
    { id: 'slide-24', title: 'Homework', icon: Target },
    { id: 'slide-25', title: 'Next Lecture', icon: ArrowRight },
  ];

  const setTargetSpeedSrvCode = `# SetTargetSpeed.srv
# Request (what the client sends)
float64 target_speed  # Desired speed in m/s
---
# Response (what the server returns)
bool success          # True if speed was set
string message        # Feedback message
float64 actual_speed  # The speed that was actually set`;

  const batteryMonitorServerCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetTargetSpeed
import random

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Battery state
        self.battery_level = 100.0
        self.current_speed = 0.0
        
        # Create the service server
        self.service = self.create_service(
            SetTargetSpeed,
            'set_target_speed',
            self.set_speed_callback
        )
        
        # Simulate battery drain
        self.timer = self.create_timer(1.0, self.update_battery)
        
        self.get_logger().info("Battery Monitor with speed control ready!")
    
    def update_battery(self):
        # Drain battery faster when moving faster
        drain_rate = 0.5 + (self.current_speed * 0.3)
        self.battery_level -= drain_rate
        
        if self.battery_level < 0:
            self.battery_level = 0.0
        
        self.get_logger().info(
            f"Battery: {self.battery_level:.1f}% | Speed: {self.current_speed:.2f} m/s"
        )
    
    def set_speed_callback(self, request, response):
        # Validate the requested speed
        requested = request.target_speed
        
        # Check if battery is too low for high speed
        if self.battery_level < 20 and requested > 0.5:
            response.success = False
            response.message = f"Battery too low ({self.battery_level:.1f}%) for speed {requested:.2f} m/s"
            response.actual_speed = self.current_speed
            self.get_logger().warn(response.message)
        
        # Check for valid speed range
        elif requested < 0 or requested > 2.0:
            response.success = False
            response.message = f"Speed {requested:.2f} m/s out of range [0.0, 2.0]"
            response.actual_speed = self.current_speed
            self.get_logger().warn(response.message)
        
        # All checks passed - set the speed
        else:
            self.current_speed = requested
            response.success = True
            response.message = f"Speed set to {requested:.2f} m/s"
            response.actual_speed = self.current_speed
            self.get_logger().info(response.message)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const speedControllerClientCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetTargetSpeed
import sys

class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')
        
        # Create service client
        self.client = self.create_client(SetTargetSpeed, 'set_target_speed')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for battery monitor service...')
        
        self.get_logger().info("Speed Controller ready!")
    
    def set_speed(self, target_speed):
        # Create request with data
        request = SetTargetSpeed.Request()
        request.target_speed = target_speed
        
        self.get_logger().info(f"Requesting speed: {target_speed:.2f} m/s")
        
        # Call the service and wait for response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        # Get the response
        response = future.result()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SpeedController()
    
    # Try different speeds
    speeds_to_test = [1.0, 1.5, 0.5]
    
    for speed in speeds_to_test:
        response = node.set_speed(speed)
        
        if response.success:
            node.get_logger().info(f"✓ {response.message}")
        else:
            node.get_logger().warn(f"✗ {response.message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const getSpeedLimitSrvCode = `# GetSpeedLimit.srv
# Request - ask for safe speed based on battery level
float64 battery_level
---
# Response - recommended speed limit
float64 max_speed
string recommendation
bool emergency_stop`;

  const speedLimitServerCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import GetSpeedLimit
from std_msgs.msg import Float32

class SafetyAdvisor(Node):
    def __init__(self):
        super().__init__('safety_advisor')
        
        # Subscribe to battery level (from Lecture 3)
        self.battery = 100.0
        self.create_subscription(
            Float32, 'battery_level', self.battery_callback, 10
        )
        
        # Create service
        self.service = self.create_service(
            GetSpeedLimit,
            'get_speed_limit',
            self.speed_limit_callback
        )
        
        self.get_logger().info("Safety Advisor Service ready!")
    
    def battery_callback(self, msg):
        self.battery = msg.data
    
    def speed_limit_callback(self, request, response):
        # Use either current battery or requested battery level
        battery = request.battery_level if request.battery_level > 0 else self.battery
        
        # Determine safe speed based on battery
        if battery < 10:
            response.max_speed = 0.0
            response.recommendation = "Battery critical! Return to base immediately"
            response.emergency_stop = True
        elif battery < 20:
            response.max_speed = 0.3
            response.recommendation = "Low battery - reduce speed significantly"
            response.emergency_stop = False
        elif battery < 50:
            response.max_speed = 1.0
            response.recommendation = "Moderate battery - medium speed advised"
            response.emergency_stop = False
        else:
            response.max_speed = 2.0
            response.recommendation = "Battery healthy - full speed available"
            response.emergency_stop = False
        
        self.get_logger().info(
            f"Speed limit requested for {battery:.1f}% battery: {response.max_speed:.2f} m/s"
        )
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SafetyAdvisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const packageXmlCode = `<?xml version="1.0"?>
<package format="3">
  <name>my_robot_interfaces</name>
  <version>0.0.0</version>
  <description>Custom interfaces for delivery robot</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

</package>`;

  const cmakeListsCode = `cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "srv/SetTargetSpeed.srv"
  "srv/GetSpeedLimit.srv"
)

ament_package()`;

  const setupPyAdditionCode = `entry_points={
    'console_scripts': [
        'battery_monitor = my_robot_pkg.battery_monitor:main',
        'speed_sensor = my_robot_pkg.speed_sensor:main',
        'dashboard = my_robot_pkg.dashboard:main',
        'sensor_filter = my_robot_pkg.sensor_filter:main',
        'speed_controller = my_robot_pkg.speed_controller:main',
        'safety_advisor = my_robot_pkg.safety_advisor:main',
    ],
},`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={4} 
        lectureTitle="Services" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Title */}
        <LectureSlide id="slide-1" title="Services" subtitle="Request-Response Communication" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Lecture</h4>
                <p className="font-medium text-zinc-900">4 of 12</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Focus</h4>
                <p className="font-medium text-zinc-900">Client-Server Communication</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Goal</h4>
                <p className="font-medium text-zinc-900">Ask questions, get answers</p>
              </div>
            </div>
            <div className="mt-8 pt-8 border-t border-zinc-200">
              <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-4">Today&apos;s Agenda</h4>
              <div className="grid md:grid-cols-4 gap-4">
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">❓</div>
                  <div className="text-sm font-medium">Why Services?</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">📦</div>
                  <div className="text-sm font-medium">Custom Interfaces</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">🖥️</div>
                  <div className="text-sm font-medium">Server &amp; Client</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">🔧</div>
                  <div className="text-sm font-medium">CLI Tools</div>
                </div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 2: Homework Review */}
        <LectureSlide id="slide-2" title="Homework Review" subtitle="Performance Monitor" icon={Check}>
          <div className="mb-8">
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h3 className="font-bold text-green-900 mb-4">📋 The Assignment Was:</h3>
              <p className="text-green-800 mb-4">Create a <code className="bg-white px-2 py-1 rounded">performance_monitor</code> node that:</p>
              <ul className="space-y-2 text-green-800">
                <li>• Subscribes to <code className="bg-white px-1 rounded">/battery_level</code> and <code className="bg-white px-1 rounded">/robot/speed_filtered</code></li>
                <li>• Publishes alerts on <code className="bg-white px-1 rounded">/robot/alerts</code></li>
                <li>• Warns if speed &gt; 1.5 m/s AND battery &lt; 30%</li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">🔑 Key Learning: Combining Multiple Data Sources</h4>
            <p className="text-blue-800">
              Your node subscribed to two topics and made decisions based on both values. This pattern of combining data is very common in robotics.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 3: Review Topics */}
        <LectureSlide id="slide-3" title="Review: What Topics Do" subtitle="Continuous Data Streams" icon={RefreshCw}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-2xl font-medium">
              Topics are for <span className="text-green-400">continuous streams</span> of data.
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 mb-8">
            <div className="p-4 bg-zinc-50 rounded-lg text-center">
              <div className="text-3xl mb-2">📡</div>
              <h4 className="font-bold text-zinc-900">Publisher</h4>
              <p className="text-zinc-600 text-sm">Sends data continuously</p>
            </div>
            <div className="p-4 bg-zinc-50 rounded-lg text-center">
              <div className="text-3xl mb-2">📻</div>
              <h4 className="font-bold text-zinc-900">Topic</h4>
              <p className="text-zinc-600 text-sm">Named channel</p>
            </div>
            <div className="p-4 bg-zinc-50 rounded-lg text-center">
              <div className="text-3xl mb-2">📥</div>
              <h4 className="font-bold text-zinc-900">Subscriber</h4>
              <p className="text-zinc-600 text-sm">Receives when available</p>
            </div>
          </div>

          <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
            <h4 className="font-bold text-green-900 mb-2">✅ Topics are great for:</h4>
            <ul className="text-green-800 text-sm space-y-1">
              <li>• Sensor data (battery, speed, camera images)</li>
              <li>• Status updates sent regularly</li>
              <li>• One-to-many communication</li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 4: The Problem */}
        <LectureSlide id="slide-4" title="The Problem" subtitle="When Topics Aren&apos;t Enough" icon={HelpCircle}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Scenario: Task Planner Needs Robot Location</h3>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <h4 className="font-bold text-red-900 mb-4">❌ Using a Topic</h4>
              <ul className="space-y-3 text-red-800 text-sm">
                <li>• Location published every 1 second</li>
                <li>• Task planner subscribes...</li>
                <li>• But might get data from 0.9 seconds ago</li>
                <li>• No guarantee of freshness</li>
                <li>• What if we only need it once?</li>
              </ul>
            </div>
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-4">✅ What We Actually Need</h4>
              <ul className="space-y-3 text-green-800 text-sm">
                <li>• Ask: &quot;Where is the robot NOW?&quot;</li>
                <li>• Get an immediate response</li>
                <li>• Guaranteed fresh data</li>
                <li>• One-time request, one response</li>
                <li>• Like making a phone call</li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-amber-50 border border-amber-200 rounded-xl">
            <h4 className="font-bold text-amber-900 mb-2">🤔 More Examples Where Topics Don&apos;t Fit:</h4>
            <div className="grid md:grid-cols-3 gap-4 mt-4 text-sm">
              <div className="p-3 bg-white rounded">
                <p className="text-amber-800">&quot;What&apos;s the current task?&quot;</p>
              </div>
              <div className="p-3 bg-white rounded">
                <p className="text-amber-800">&quot;Is this delivery valid?&quot;</p>
              </div>
              <div className="p-3 bg-white rounded">
                <p className="text-amber-800">&quot;Reset the robot position&quot;</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 5: What is a Service? */}
        <LectureSlide id="slide-5" title="What is a Service?" subtitle="Request-Response Pattern" icon={PhoneCall}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-2xl font-medium">
              A <span className="text-yellow-400">Service</span> is a request-response communication.
            </p>
            <p className="text-zinc-400 mt-2">You ask a question, you get an answer.</p>
          </div>

          <div className="p-8 bg-zinc-50 rounded-xl border border-zinc-200 mb-8">
            <div className="flex flex-col md:flex-row items-center justify-between gap-6">
              <div className="p-4 bg-white border-2 border-blue-300 rounded-lg text-center">
                <div className="text-2xl mb-2">📞</div>
                <div className="font-bold">Client</div>
                <div className="text-xs text-zinc-500">Sends request</div>
              </div>
              
              <div className="flex flex-col items-center gap-2">
                <div className="flex items-center gap-2">
                  <span className="text-sm font-mono bg-blue-100 px-2 py-1 rounded">Request</span>
                  <ArrowRight size={20} className="text-blue-500" />
                </div>
                <div className="p-2 bg-zinc-200 rounded font-mono text-sm">/get_location</div>
                <div className="flex items-center gap-2">
                  <ArrowRight size={20} className="text-green-500 rotate-180" />
                  <span className="text-sm font-mono bg-green-100 px-2 py-1 rounded">Response</span>
                </div>
              </div>
              
              <div className="p-4 bg-white border-2 border-green-300 rounded-lg text-center">
                <div className="text-2xl mb-2">🖥️</div>
                <div className="font-bold">Server</div>
                <div className="text-xs text-zinc-500">Sends response</div>
              </div>
            </div>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">Client (Asks)</h4>
              <p className="text-blue-800 text-sm">Sends a request and waits for a response. Can call the service whenever needed.</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">Server (Answers)</h4>
              <p className="text-green-800 text-sm">Waits for requests, processes them, and sends back a response.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 6: Topics vs Services */}
        <LectureSlide id="slide-6" title="Topics vs Services" subtitle="When to Use Which" icon={ArrowLeftRight}>
          <div className="overflow-x-auto">
            <table className="w-full text-left border-collapse">
              <thead>
                <tr className="bg-zinc-100">
                  <th className="p-4 font-bold text-zinc-900 border-b-2"></th>
                  <th className="p-4 font-bold text-blue-900 border-b-2">Topics</th>
                  <th className="p-4 font-bold text-green-900 border-b-2">Services</th>
                </tr>
              </thead>
              <tbody className="text-sm">
                <tr className="border-b">
                  <td className="p-4 font-medium">Pattern</td>
                  <td className="p-4">Publish-Subscribe</td>
                  <td className="p-4">Request-Response</td>
                </tr>
                <tr className="border-b bg-zinc-50">
                  <td className="p-4 font-medium">Communication</td>
                  <td className="p-4">One-way (broadcast)</td>
                  <td className="p-4">Two-way (ask &amp; answer)</td>
                </tr>
                <tr className="border-b">
                  <td className="p-4 font-medium">Timing</td>
                  <td className="p-4">Continuous stream</td>
                  <td className="p-4">On-demand</td>
                </tr>
                <tr className="border-b bg-zinc-50">
                  <td className="p-4 font-medium">Receivers</td>
                  <td className="p-4">Many subscribers</td>
                  <td className="p-4">One server, many clients</td>
                </tr>
                <tr className="border-b">
                  <td className="p-4 font-medium">Use when</td>
                  <td className="p-4">Sensor data, status updates</td>
                  <td className="p-4">Queries, commands, actions</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div className="mt-8 grid md:grid-cols-2 gap-6">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">📻 Use Topics For:</h4>
              <ul className="text-blue-800 text-sm space-y-1">
                <li>• Battery level every second</li>
                <li>• Speed sensor readings</li>
                <li>• Camera images</li>
              </ul>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">📞 Use Services For:</h4>
              <ul className="text-green-800 text-sm space-y-1">
                <li>• &quot;Where is the robot?&quot;</li>
                <li>• &quot;Start a delivery task&quot;</li>
                <li>• &quot;Reset the system&quot;</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 7: Service Structure */}
        <LectureSlide id="slide-7" title="Service = Name + Interface" subtitle="Two Things Define a Service" icon={MessageSquare}>
          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 border-2 border-purple-200 bg-purple-50/30 rounded-xl">
              <h4 className="font-bold text-purple-900 text-lg mb-4">1. Service Name</h4>
              <p className="text-zinc-700 mb-4">A unique identifier for the service.</p>
              <div className="space-y-2 font-mono text-sm">
                <div className="p-2 bg-white rounded">/get_location</div>
                <div className="p-2 bg-white rounded">/assign_task</div>
                <div className="p-2 bg-white rounded">/get_robot_status</div>
              </div>
            </div>
            
            <div className="p-6 border-2 border-blue-200 bg-blue-50/30 rounded-xl">
              <h4 className="font-bold text-blue-900 text-lg mb-4">2. Service Interface</h4>
              <p className="text-zinc-700 mb-4">Defines what goes in the request AND response.</p>
              <div className="p-4 bg-white rounded font-mono text-sm">
                <div className="text-zinc-500"># Request</div>
                <div>string destination</div>
                <div className="text-zinc-400 my-2">---</div>
                <div className="text-zinc-500"># Response</div>
                <div>bool success</div>
                <div>string message</div>
              </div>
            </div>
          </div>

          <div className="p-6 bg-amber-50 border border-amber-200 rounded-xl">
            <h4 className="font-bold text-amber-900 mb-2">📝 Key Difference from Topics</h4>
            <p className="text-amber-800">
              Topic interfaces have ONE message type. Service interfaces have TWO parts: a <strong>Request</strong> and a <strong>Response</strong>, separated by <code className="bg-white px-2 py-1 rounded">---</code>
            </p>
          </div>
        </LectureSlide>

        {/* Slide 8: What We're Building */}
        <LectureSlide id="slide-8" title="What We're Building Today" subtitle="The Problem We're Solving" icon={Target}>
          <div className="mb-8">
            <h3 className="text-2xl font-bold text-zinc-900 mb-4">🎯 Our Goal: Control Robot Speed Safely</h3>
            <p className="text-lg text-zinc-600">
              Remember our battery_monitor from Lecture 2? It just publishes battery levels. But what if we want to <strong>control</strong> the robot based on battery?
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <h4 className="font-bold text-red-900 mb-4">❌ The Problem with Topics</h4>
              <p className="text-red-800 mb-4">If you publish &quot;set speed to 2.0 m/s&quot; on a topic:</p>
              <ul className="space-y-2 text-red-800 text-sm">
                <li>• Did the robot receive it?</li>
                <li>• Did it actually set the speed?</li>
                <li>• What if battery is too low?</li>
                <li>• Was the speed valid (not negative)?</li>
                <li>• <strong>You get no feedback!</strong></li>
              </ul>
            </div>

            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-4">✅ What We Need: Request-Response</h4>
              <p className="text-green-800 mb-4">With a service:</p>
              <ul className="space-y-2 text-green-800 text-sm">
                <li>• Send: &quot;Set speed to 2.0 m/s&quot;</li>
                <li>• Robot checks battery level</li>
                <li>• Robot validates speed range</li>
                <li>• Get back: &quot;Success!&quot; or &quot;Failed: battery low&quot;</li>
                <li>• <strong>Guaranteed response!</strong></li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">📋 What We'll Build:</h4>
            <ol className="space-y-2 text-blue-800">
              <li><strong>1.</strong> Update <code className="bg-white px-2 py-1 rounded">battery_monitor</code> to be a service server</li>
              <li><strong>2.</strong> Create <code className="bg-white px-2 py-1 rounded">SetTargetSpeed</code> service (with request data!)</li>
              <li><strong>3.</strong> Build a <code className="bg-white px-2 py-1 rounded">speed_controller</code> client to call it</li>
              <li><strong>4.</strong> See validation in action (reject bad speeds, low battery)</li>
            </ol>
          </div>
        </LectureSlide>

        {/* Slide 9: Creating Interfaces Package */}
        <LectureSlide id="slide-9" title="Creating an Interfaces Package" subtitle="One-Time CMake Setup" icon={Package}>
          <div className="mb-6 p-6 bg-amber-50 border border-amber-200 rounded-xl">
            <h4 className="font-bold text-amber-900 mb-2">⚠️ Important: This is the ONLY CMake in our course!</h4>
            <p className="text-amber-800">
              ROS 2 requires custom interfaces to be in a <strong>CMake package</strong> (C++) because they must be compiled. This is a one-time setup. <strong>All our robot code stays in Python!</strong>
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Navigate to your workspace src folder</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws/src"
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: Create a new CMake package for interfaces</h4>
              <TerminalBlock 
                command="ros2 pkg create my_robot_interfaces"
                output={`going to create a new package
package name: my_robot_interfaces
destination directory: /home/user/ros2_ws/src
package format: 3
...
creating ./my_robot_interfaces/package.xml
creating ./my_robot_interfaces/CMakeLists.txt`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 3: Create folders for message and service interfaces</h4>
              <TerminalBlock 
                command="cd my_robot_interfaces && mkdir msg srv"
                title="Terminal"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-blue-50 border border-blue-100 rounded-lg">
            <h4 className="font-bold text-blue-900 mb-2">📁 Package Structure</h4>
            <div className="font-mono text-sm text-blue-800">
              <div>my_robot_interfaces/</div>
              <div className="pl-4">├── msg/       <span className="text-zinc-500"># Topic interfaces go here</span></div>
              <div className="pl-4">├── srv/       <span className="text-zinc-500"># Service interfaces go here</span></div>
              <div className="pl-4">├── CMakeLists.txt <span className="text-zinc-500"># C++ build config (we&apos;ll edit once)</span></div>
              <div className="pl-4">└── package.xml    <span className="text-zinc-500"># Package metadata</span></div>
            </div>
          </div>

          <div className="mt-6 p-4 bg-purple-50 border border-purple-200 rounded-lg">
            <h4 className="font-bold text-purple-900 mb-2">💡 Remember</h4>
            <p className="text-purple-800 text-sm">
              This interfaces package is just a <strong>container for definitions</strong>. All our actual robot logic stays in Python in <code className="bg-white px-1 rounded">my_robot_pkg</code>!
            </p>
          </div>
        </LectureSlide>

        {/* Slide 10: Package Configuration */}
        <LectureSlide id="slide-10" title="Package Configuration" subtitle="Setting Up package.xml and CMakeLists.txt" icon={Settings}>
          <div className="mb-6 p-6 bg-amber-50 border border-amber-200 rounded-xl">
            <h4 className="font-bold text-amber-900 mb-2">📝 This is a template you&apos;ll reuse</h4>
            <p className="text-amber-800">
              Set this up once. When you need new interfaces later, just add the .srv file and update CMakeLists.txt. <strong>That&apos;s it - no more C++!</strong>
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">package.xml - Add these dependencies:</h4>
              <CodeBlock 
                filename="my_robot_interfaces/package.xml"
                code={packageXmlCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">CMakeLists.txt - Configure interface generation:</h4>
              <CodeBlock 
                filename="my_robot_interfaces/CMakeLists.txt"
                code={cmakeListsCode}
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">⚠️ Important</h4>
            <p className="text-amber-800 text-sm">
              The <code className="bg-white px-1 rounded">rosidl_generate_interfaces</code> function lists all your interface files. Add each new interface file here.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 11: Custom Service Interface */}
        <LectureSlide id="slide-11" title="Custom Service Interface" subtitle="Defining SetTargetSpeed.srv" icon={FileCode}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Let&apos;s create a service interface for controlling the robot&apos;s speed with validation.
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Create the service interface file:</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws/src/my_robot_interfaces/srv && touch SetTargetSpeed.srv"
                title="Terminal"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Write the interface definition:</h4>
              <CodeBlock 
                filename="srv/SetTargetSpeed.srv"
                code={setTargetSpeedSrvCode}
              />
            </div>
          </div>

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">Request (above ---)</h4>
              <p className="text-blue-800 text-sm">Client sends the desired speed. The server will validate it.</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">Response (below ---)</h4>
              <p className="text-green-800 text-sm">Server returns success/failure, a message explaining why, and the actual speed that was set.</p>
            </div>
          </div>

          <div className="mt-6 p-4 bg-purple-50 border border-purple-200 rounded-lg">
            <h4 className="font-bold text-purple-900 mb-2">📝 Naming Rules for .srv Files</h4>
            <ul className="text-purple-800 text-sm space-y-1">
              <li>• Use PascalCase: <code className="bg-white px-1 rounded">SetTargetSpeed</code>, not <code className="bg-white px-1 rounded">set_target_speed</code></li>
              <li>• Use verbs: <code className="bg-white px-1 rounded">SetTargetSpeed</code>, <code className="bg-white px-1 rounded">GetBattery</code>, <code className="bg-white px-1 rounded">ResetRobot</code></li>
              <li>• Field names use snake_case: <code className="bg-white px-1 rounded">target_speed</code>, <code className="bg-white px-1 rounded">battery_level</code></li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 12: Building the Interface */}
        <LectureSlide id="slide-12" title="Building the Interface" subtitle="Compile and Verify" icon={Hammer}>
          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Build the interfaces package</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws && colcon build --packages-select my_robot_interfaces"
                output={`Starting >>> my_robot_interfaces
Finished <<< my_robot_interfaces [5.2s]

Summary: 1 package finished [5.8s]`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: Source the workspace</h4>
              <TerminalBlock 
                command="source ~/ros2_ws/install/setup.bash"
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 3: Verify the interface exists</h4>
              <TerminalBlock 
                command="ros2 interface show my_robot_interfaces/srv/SetTargetSpeed"
                output={`float64 target_speed
---
bool success
string message
float64 actual_speed`}
                title="Terminal"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800">
              <strong>✅ Success!</strong> If you see the interface definition, you can now use it in your Python nodes.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 13: Service Server Code */}
        <LectureSlide id="slide-13" title="Service Server Code" subtitle="Updating battery_monitor" icon={Code}>
          <div className="mb-4">
            <p className="text-zinc-600">
              Let&apos;s update our existing <code className="bg-zinc-100 px-2 py-1 rounded">battery_monitor.py</code> from Lecture 2 to be a service server!
            </p>
          </div>

          <CodeBlock 
            filename="battery_monitor.py (updated)"
            code={batteryMonitorServerCode}
          />
        </LectureSlide>

        {/* Slide 14: Understanding the Server */}
        <LectureSlide id="slide-14" title="Understanding the Server" subtitle="Key Parts Explained" icon={Search}>
          <div className="space-y-6">
            <div className="p-4 bg-purple-50 border-l-4 border-purple-500 rounded-r-lg">
              <h4 className="font-bold text-purple-900 mb-2">from my_robot_interfaces.srv import SetTargetSpeed</h4>
              <p className="text-purple-800 text-sm">Import the custom service interface we created.</p>
            </div>

            <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
              <h4 className="font-bold text-blue-900 mb-2">self.create_service(SetTargetSpeed, &apos;set_target_speed&apos;, self.callback)</h4>
              <p className="text-blue-800 text-sm">Create the service server with three arguments:</p>
              <ul className="text-blue-800 text-sm mt-2 space-y-1">
                <li>• <strong>Service type:</strong> SetTargetSpeed (the interface)</li>
                <li>• <strong>Service name:</strong> &apos;set_target_speed&apos; (what clients call)</li>
                <li>• <strong>Callback:</strong> Function that handles requests</li>
              </ul>
            </div>

            <div className="p-4 bg-green-50 border-l-4 border-green-500 rounded-r-lg">
              <h4 className="font-bold text-green-900 mb-2">request.target_speed</h4>
              <p className="text-green-800 text-sm">Access the data the client sent. We can read <code className="bg-white px-1 rounded">request.target_speed</code> to see what speed was requested.</p>
            </div>

            <div className="p-4 bg-yellow-50 border-l-4 border-yellow-500 rounded-r-lg">
              <h4 className="font-bold text-yellow-900 mb-2">response.success = True / False</h4>
              <p className="text-yellow-800 text-sm">Fill in the response fields. Set success, message, and actual_speed based on validation logic.</p>
            </div>

            <div className="p-4 bg-orange-50 border-l-4 border-orange-500 rounded-r-lg">
              <h4 className="font-bold text-orange-900 mb-2">return response</h4>
              <p className="text-orange-800 text-sm">You MUST return the response object. This sends the data back to the client.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 15: Setup and Build */}
        <LectureSlide id="slide-15" title="Setup and Build" subtitle="Preparing to Run" icon={Wrench}>
          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Add dependency to package.xml</h4>
              <div className="p-4 bg-white rounded font-mono text-sm">
                <div className="text-zinc-500">&lt;!-- In my_robot_pkg/package.xml --&gt;</div>
                <div>&lt;depend&gt;my_robot_interfaces&lt;/depend&gt;</div>
              </div>
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: Add entry point to setup.py</h4>
              <CodeBlock 
                filename="my_robot_pkg/setup.py (entry_points section)"
                code={setupPyAdditionCode}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 3: Build and source</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws && colcon build --packages-select my_robot_pkg && source install/setup.bash"
                output={`Starting >>> my_robot_pkg
Finished <<< my_robot_pkg [1.2s]

Summary: 1 package finished [1.8s]`}
                title="Terminal"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 16: Testing the Server */}
        <LectureSlide id="slide-16" title="Testing the Server" subtitle="Using CLI Before Writing a Client" icon={Play}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Before writing a client node, we can test the service using command line tools.
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 1: Start the server</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg battery_monitor"
                output={`[INFO] [battery_monitor]: Battery Monitor with speed control ready!
[INFO] [battery_monitor]: Battery: 100.0% | Speed: 0.00 m/s
[INFO] [battery_monitor]: Battery: 99.5% | Speed: 0.00 m/s`}
                title="Terminal 1"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: List available services</h4>
              <TerminalBlock 
                command="ros2 service list"
                output={`/set_target_speed
/battery_monitor/describe_parameters
/battery_monitor/get_parameter_types
...`}
                title="Terminal 2"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Call the service with valid speed</h4>
              <TerminalBlock 
                command={`ros2 service call /set_target_speed my_robot_interfaces/srv/SetTargetSpeed "{target_speed: 1.5}"`}
                output={`requester: making request: my_robot_interfaces.srv.SetTargetSpeed_Request(target_speed=1.5)

response:
my_robot_interfaces.srv.SetTargetSpeed_Response(
  success=True, 
  message='Speed set to 1.50 m/s', 
  actual_speed=1.5
)`}
                title="Terminal 2"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Try an invalid speed (too high)</h4>
              <TerminalBlock 
                command={`ros2 service call /set_target_speed my_robot_interfaces/srv/SetTargetSpeed "{target_speed: 3.0}"`}
                output={`response:
my_robot_interfaces.srv.SetTargetSpeed_Response(
  success=False, 
  message='Speed 3.00 m/s out of range [0.0, 2.0]', 
  actual_speed=1.5
)`}
                title="Terminal 2"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800">
              <strong>🎉 It works!</strong> The server validates our requests and gives us clear feedback.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 17: Service Client Code */}
        <LectureSlide id="slide-17" title="Service Client Code" subtitle="The Speed Controller Node" icon={Code}>
          <div className="mb-4">
            <p className="text-zinc-600">
              Now let&apos;s create a client node that calls the service. Create <code className="bg-zinc-100 px-2 py-1 rounded">speed_controller.py</code>
            </p>
          </div>

          <CodeBlock 
            filename="speed_controller.py"
            code={speedControllerClientCode}
          />
        </LectureSlide>

        {/* Slide 18: Understanding the Client */}
        <LectureSlide id="slide-18" title="Understanding the Client" subtitle="Key Parts Explained" icon={Search}>
          <div className="space-y-6">
            <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
              <h4 className="font-bold text-blue-900 mb-2">self.create_client(SetTargetSpeed, &apos;set_target_speed&apos;)</h4>
              <p className="text-blue-800 text-sm">Create a client for the service. Arguments:</p>
              <ul className="text-blue-800 text-sm mt-2 space-y-1">
                <li>• <strong>Service type:</strong> Must match the server&apos;s interface</li>
                <li>• <strong>Service name:</strong> Must match the server&apos;s name</li>
              </ul>
            </div>

            <div className="p-4 bg-purple-50 border-l-4 border-purple-500 rounded-r-lg">
              <h4 className="font-bold text-purple-900 mb-2">self.client.wait_for_service(timeout_sec=1.0)</h4>
              <p className="text-purple-800 text-sm">Wait for the service to be available. Returns True when ready, False on timeout. Always do this before calling!</p>
            </div>

            <div className="p-4 bg-green-50 border-l-4 border-green-500 rounded-r-lg">
              <h4 className="font-bold text-green-900 mb-2">request = SetTargetSpeed.Request()</h4>
              <p className="text-green-800 text-sm">Create a request object and fill in the fields: <code className="bg-white px-1 rounded">request.target_speed = 1.5</code></p>
            </div>

            <div className="p-4 bg-yellow-50 border-l-4 border-yellow-500 rounded-r-lg">
              <h4 className="font-bold text-yellow-900 mb-2">future = self.client.call_async(request)</h4>
              <p className="text-yellow-800 text-sm">Call the service asynchronously. Returns a &quot;future&quot; object.</p>
            </div>

            <div className="p-4 bg-orange-50 border-l-4 border-orange-500 rounded-r-lg">
              <h4 className="font-bold text-orange-900 mb-2">rclpy.spin_until_future_complete(self, future)</h4>
              <p className="text-orange-800 text-sm">Wait until the response arrives. After this, <code>future.result()</code> contains the response with success, message, and actual_speed.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 19: Running Both Nodes */}
        <LectureSlide id="slide-19" title="Running Both Nodes" subtitle="Server and Client Together" icon={Activity}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Don&apos;t forget to add the entry point, build, and source before running!
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 1: Start the server (keep running)</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg battery_monitor"
                output={`[INFO] [battery_monitor]: Battery Monitor with speed control ready!
[INFO] [battery_monitor]: Battery: 100.0% | Speed: 0.00 m/s
[INFO] [battery_monitor]: Speed set to 1.00 m/s
[INFO] [battery_monitor]: Battery: 99.2% | Speed: 1.00 m/s
[INFO] [battery_monitor]: Speed set to 1.50 m/s
[WARN] [battery_monitor]: Speed 3.00 m/s out of range [0.0, 2.0]`}
                title="Terminal 1: Server"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Run the client</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg speed_controller"
                output={`[INFO] [speed_controller]: Speed Controller ready!
[INFO] [speed_controller]: Requesting speed: 1.00 m/s
[INFO] [speed_controller]: ✓ Speed set to 1.00 m/s
[INFO] [speed_controller]: Requesting speed: 1.50 m/s
[INFO] [speed_controller]: ✓ Speed set to 1.50 m/s
[INFO] [speed_controller]: Requesting speed: 0.50 m/s
[INFO] [speed_controller]: ✓ Speed set to 0.50 m/s`}
                title="Terminal 2: Client"
              />
            </div>
          </div>

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">✅ What Happened</h4>
              <ol className="text-green-800 text-sm space-y-1 list-decimal list-inside">
                <li>Client started and found the service</li>
                <li>Client sent multiple speed requests</li>
                <li>Server validated each request</li>
                <li>Client got success/failure for each</li>
              </ol>
            </div>
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">📝 Note</h4>
              <p className="text-blue-800 text-sm">The client tests multiple speeds then exits. The server keeps running, ready for more requests. We updated an existing node (battery_monitor) instead of creating a new one!</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 20: Speed Control Service */}
        <LectureSlide id="slide-20" title="Another Example: Safety Advisor" subtitle="Integrating with Lecture 3 Topics" icon={Bot}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Let&apos;s create a service that combines data from our existing topics. This shows how services and topics work together.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">The Service Interface:</h4>
              <CodeBlock 
                filename="srv/GetSpeedLimit.srv"
                code={getSpeedLimitSrvCode}
              />
            </div>
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">What It Does:</h4>
              <ul className="text-zinc-700 space-y-2 text-sm">
                <li>• <strong>Subscribes</strong> to /battery_level (from Lecture 3)</li>
                <li>• Client asks: &quot;What&apos;s safe speed for battery X%?&quot;</li>
                <li>• <strong>Returns</strong> recommended max speed</li>
                <li>• Provides safety recommendation</li>
                <li>• Flags emergency_stop if critical</li>
              </ul>
              <div className="mt-4 p-3 bg-blue-50 rounded">
                <p className="text-blue-800 text-sm">This combines topics (continuous battery data) with services (on-demand safety advice).</p>
              </div>
            </div>
          </div>

          <div className="mb-6">
            <h4 className="font-bold text-zinc-900 mb-3">The Python Implementation:</h4>
            <CodeBlock 
              filename="safety_advisor.py"
              code={speedLimitServerCode}
            />
          </div>

          <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">📝 Remember to:</h4>
            <ol className="text-amber-800 text-sm space-y-1 list-decimal list-inside">
              <li>Add <code className="bg-white px-1 rounded">GetSpeedLimit.srv</code> to CMakeLists.txt</li>
              <li>Rebuild the interfaces package</li>
              <li>Add safety_advisor entry point to setup.py</li>
              <li>Build my_robot_pkg and source</li>
            </ol>
          </div>

          <div className="mt-6">
            <h4 className="font-bold text-zinc-900 mb-3">Test it with the battery_monitor from Lecture 2:</h4>
            <TerminalBlock 
              command={`# Terminal 1: Run battery monitor (publishes /battery_level)
ros2 run my_robot_pkg battery_monitor

# Terminal 2: Run safety advisor service
ros2 run my_robot_pkg safety_advisor

# Terminal 3: Query safe speed
ros2 service call /get_speed_limit my_robot_interfaces/srv/GetSpeedLimit "{battery_level: 45.0}"`}
              output={`response:
my_robot_interfaces.srv.GetSpeedLimit_Response(
  max_speed=1.0,
  recommendation='Moderate battery - medium speed advised',
  emergency_stop=False
)`}
              title="Integration with Lecture 3"
            />
          </div>
        </LectureSlide>

        {/* Slide 21: CLI Tools */}
        <LectureSlide id="slide-21" title="CLI Tools for Services" subtitle="Commands You Need to Know" icon={Terminal}>
          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">List all services</h4>
              <TerminalBlock 
                command="ros2 service list"
                output={`/set_target_speed
/get_speed_limit
/battery_monitor/describe_parameters
...`}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Get service type</h4>
              <TerminalBlock 
                command="ros2 service type /set_target_speed"
                output={`my_robot_interfaces/srv/SetTargetSpeed`}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Show interface definition</h4>
              <TerminalBlock 
                command="ros2 interface show my_robot_interfaces/srv/SetTargetSpeed"
                output={`float64 target_speed
---
bool success
string message
float64 actual_speed`}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Call a service from terminal</h4>
              <TerminalBlock 
                command={`ros2 service call /set_target_speed my_robot_interfaces/srv/SetTargetSpeed "{target_speed: 1.0}"`}
                output={`response:
my_robot_interfaces.srv.SetTargetSpeed_Response(
  success=True,
  message='Speed set to 1.00 m/s',
  actual_speed=1.0
)`}
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 22: Key Takeaways */}
        <LectureSlide id="slide-22" title="Key Takeaways" subtitle="Remember These" icon={Lightbulb}>
          <div className="grid md:grid-cols-2 gap-8">
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
                <h4 className="font-bold text-blue-900 mb-2">Services vs Topics</h4>
                <p className="text-blue-800 text-sm">Topics = continuous streams. Services = ask and answer.</p>
              </div>
              
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h4 className="font-bold text-green-900 mb-2">Service = Name + Interface</h4>
                <p className="text-green-800 text-sm">Interface has Request and Response, separated by ---</p>
              </div>
              
              <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg">
                <h4 className="font-bold text-purple-900 mb-2">Interfaces Package</h4>
                <p className="text-purple-800 text-sm">Custom interfaces go in a separate CMake package</p>
              </div>
            </div>
            
            <div className="space-y-4">
              <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
                <h4 className="font-bold text-yellow-900 mb-2">Server</h4>
                <p className="text-yellow-800 text-sm">create_service() with callback that returns response</p>
              </div>
              
              <div className="p-4 bg-orange-50 border border-orange-200 rounded-lg">
                <h4 className="font-bold text-orange-900 mb-2">Client</h4>
                <p className="text-orange-800 text-sm">create_client(), wait_for_service(), call_async()</p>
              </div>
              
              <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
                <h4 className="font-bold text-red-900 mb-2">One Server Only</h4>
                <p className="text-red-800 text-sm">Many clients can call, but only one server per service name</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 23: Summary */}
        <LectureSlide id="slide-23" title="What You Learned Today" subtitle="Lecture 4 Summary" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-8">
            <div>
              <h3 className="text-xl font-bold text-zinc-900 mb-6">Concepts Covered:</h3>
              <ul className="space-y-4">
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Services</strong> provide request-response with validation</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Custom interfaces</strong> need CMake package (one-time setup)</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Requests carry data</strong> - not just empty calls</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Updated existing nodes</strong> (battery_monitor became a server)</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Integrated with Lecture 3</strong> topics and services work together</span>
                </li>
              </ul>
            </div>
            <div className="flex flex-col justify-center p-8 bg-zinc-50 rounded-2xl border border-zinc-200">
              <h4 className="text-zinc-400 font-bold uppercase tracking-widest text-sm mb-4">Next Lecture</h4>
              <div className="text-2xl font-bold text-zinc-900 mb-2">
                Parameters &amp; Launch Files
              </div>
              <p className="text-zinc-600">
                Configuring nodes without changing code. Running multiple nodes at once. Building complete robot systems.
              </p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 24: Homework */}
        <LectureSlide id="slide-24" title="Homework" subtitle="Add Emergency Stop Service" icon={Target}>
          <div className="space-y-6">
            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">1</span>
                Create a custom service interface
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• File: <code className="bg-zinc-100 px-2 py-1 rounded">EmergencyStop.srv</code></li>
                <li>• Request fields:
                  <ul className="ml-6 mt-2 space-y-1">
                    <li>- <code className="bg-zinc-100 px-1 rounded">bool activate</code> (true = stop, false = resume)</li>
                    <li>- <code className="bg-zinc-100 px-1 rounded">string reason</code> (why are we stopping?)</li>
                  </ul>
                </li>
                <li>• Response should include:
                  <ul className="ml-6 mt-2 space-y-1">
                    <li>- <code className="bg-zinc-100 px-1 rounded">bool success</code></li>
                    <li>- <code className="bg-zinc-100 px-1 rounded">string message</code></li>
                    <li>- <code className="bg-zinc-100 px-1 rounded">float64 stopped_at_speed</code> (what speed were we going?)</li>
                  </ul>
                </li>
              </ul>
            </div>

            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">2</span>
                Add service to speed_sensor node
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Update the existing <code className="bg-zinc-100 px-2 py-1 rounded">speed_sensor.py</code> from Lecture 3</li>
                <li>• Add a service server for <code className="bg-zinc-100 px-2 py-1 rounded">/emergency_stop</code></li>
                <li>• When activated, stop publishing speed (or publish 0.0)</li>
                <li>• Store the current speed before stopping</li>
                <li>• Return appropriate success/failure response</li>
              </ul>
            </div>

            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">3</span>
                Test using the CLI
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Start your updated speed_sensor node</li>
                <li>• Verify it&apos;s publishing speed data</li>
                <li>• Call the emergency stop service</li>
                <li>• Verify speed stops being published (or is 0.0)</li>
                <li>• Resume and verify it starts publishing again</li>
              </ul>
            </div>

            <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-2">🎯 Expected Behavior</h4>
              <TerminalBlock 
                command={`# Activate emergency stop
ros2 service call /emergency_stop my_robot_interfaces/srv/EmergencyStop "{activate: true, reason: 'Obstacle detected'}"`}
                output={`response:
my_robot_interfaces.srv.EmergencyStop_Response(
  success=True,
  message='Emergency stop activated: Obstacle detected',
  stopped_at_speed=1.25
)

# Speed sensor should stop publishing or publish 0.0

# Resume
ros2 service call /emergency_stop my_robot_interfaces/srv/EmergencyStop "{activate: false, reason: 'Path clear'}"`}
                title="Testing Emergency Stop"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 25: Next Lecture */}
        <LectureSlide id="slide-25" title="Next Lecture" subtitle="Configuration &amp; System Launch" icon={ArrowRight}>
          <div className="min-h-[40vh] flex flex-col items-center justify-center text-center">
            <p className="text-3xl md:text-4xl font-bold text-zinc-900 mb-8 leading-tight">
              You can run nodes.<br/>
              But can you <span className="text-green-600">configure</span> them?
            </p>
            <div className="w-16 h-1 bg-zinc-200 mb-8"></div>
            <p className="text-xl text-zinc-600 mb-8">
              Parameters for runtime configuration<br/>
              Launch files to start entire systems<br/>
              Building complete, configurable robots
            </p>
            
            <div className="mt-8 inline-flex items-center px-6 py-3 bg-zinc-900 text-white rounded-full text-sm font-bold tracking-wide">
              NEXT: Parameters &amp; Launch Files <ArrowRight size={16} className="ml-2" />
            </div>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
