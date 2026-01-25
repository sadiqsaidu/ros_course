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
    { id: 'slide-8', title: 'Creating Interfaces Package', icon: Package },
    { id: 'slide-9', title: 'Package Configuration', icon: Settings },
    { id: 'slide-10', title: 'Custom Service Interface', icon: FileCode },
    { id: 'slide-11', title: 'Building the Interface', icon: Hammer },
    { id: 'slide-12', title: 'Service Server Code', icon: Code },
    { id: 'slide-13', title: 'Understanding the Server', icon: Search },
    { id: 'slide-14', title: 'Setup and Build', icon: Wrench },
    { id: 'slide-15', title: 'Testing the Server', icon: Play },
    { id: 'slide-16', title: 'Service Client Code', icon: Code },
    { id: 'slide-17', title: 'Understanding the Client', icon: Search },
    { id: 'slide-18', title: 'Running Both Nodes', icon: Activity },
    { id: 'slide-19', title: 'Robot Status Service', icon: Bot },
    { id: 'slide-20', title: 'CLI Tools', icon: Terminal },
    { id: 'slide-21', title: 'Key Takeaways', icon: Lightbulb },
    { id: 'slide-22', title: 'Summary', icon: Layout },
    { id: 'slide-23', title: 'Homework', icon: Target },
    { id: 'slide-24', title: 'Next Lecture', icon: ArrowRight },
  ];

  const getLocationSrvCode = `# GetLocation.srv
# Request (what the client sends)
# --- (empty request means "just give me the location")
---
# Response (what the server returns)
float64 x
float64 y
string zone`;

  const localizationServerCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import GetLocation
import random

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        
        # Simulated robot position
        self.x = 0.0
        self.y = 0.0
        self.zone = "warehouse_a"
        
        # Create the service server
        self.service = self.create_service(
            GetLocation,           # Service type
            'get_location',        # Service name
            self.get_location_callback  # Callback function
        )
        
        # Simulate robot moving
        self.timer = self.create_timer(1.0, self.update_position)
        
        self.get_logger().info("Localization service ready!")
    
    def update_position(self):
        # Simulate robot moving around
        self.x += random.uniform(-0.5, 0.5)
        self.y += random.uniform(-0.5, 0.5)
    
    def get_location_callback(self, request, response):
        # This runs when a client calls the service
        response.x = self.x
        response.y = self.y
        response.zone = self.zone
        
        self.get_logger().info(
            f"Location requested: ({self.x:.2f}, {self.y:.2f}) in {self.zone}"
        )
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const locationClientCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import GetLocation

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        
        # Create service client
        self.client = self.create_client(GetLocation, 'get_location')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for localization service...')
        
        self.get_logger().info("Task Planner ready!")
    
    def get_robot_location(self):
        # Create an empty request
        request = GetLocation.Request()
        
        # Call the service and wait for response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        # Get the response
        response = future.result()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    
    # Call the service
    location = node.get_robot_location()
    
    node.get_logger().info(
        f"Robot is at ({location.x:.2f}, {location.y:.2f}) in {location.zone}"
    )
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const getRobotStatusSrvCode = `# GetRobotStatus.srv
# Request
---
# Response
float64 battery_level
float64 speed
float64 x
float64 y
string status
string message`;

  const robotStatusServerCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import GetRobotStatus
from std_msgs.msg import Float32

class StatusServer(Node):
    def __init__(self):
        super().__init__('status_server')
        
        # Store latest sensor values
        self.battery = 100.0
        self.speed = 0.0
        self.x = 0.0
        self.y = 0.0
        
        # Subscribe to sensor topics
        self.create_subscription(
            Float32, 'battery_level', self.battery_callback, 10
        )
        self.create_subscription(
            Float32, 'robot/speed_filtered', self.speed_callback, 10
        )
        
        # Create service
        self.service = self.create_service(
            GetRobotStatus,
            'get_robot_status',
            self.status_callback
        )
        
        self.get_logger().info("Robot Status Service ready!")
    
    def battery_callback(self, msg):
        self.battery = msg.data
    
    def speed_callback(self, msg):
        self.speed = msg.data
    
    def status_callback(self, request, response):
        # Fill in the response with current data
        response.battery_level = self.battery
        response.speed = self.speed
        response.x = self.x
        response.y = self.y
        
        # Determine status
        if self.battery < 20:
            response.status = "CRITICAL"
            response.message = "Battery critically low!"
        elif self.battery < 50:
            response.status = "WARNING"
            response.message = "Battery getting low"
        else:
            response.status = "OK"
            response.message = "All systems normal"
        
        self.get_logger().info(f"Status requested: {response.status}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StatusServer()
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
  "srv/GetLocation.srv"
  "srv/GetRobotStatus.srv"
)

ament_package()`;

  const setupPyAdditionCode = `entry_points={
    'console_scripts': [
        'battery_monitor = my_robot_pkg.battery_monitor:main',
        'speed_sensor = my_robot_pkg.speed_sensor:main',
        'dashboard = my_robot_pkg.dashboard:main',
        'sensor_filter = my_robot_pkg.sensor_filter:main',
        'localization = my_robot_pkg.localization:main',
        'task_planner = my_robot_pkg.task_planner:main',
        'status_server = my_robot_pkg.status_server:main',
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

        {/* Slide 8: Creating Interfaces Package */}
        <LectureSlide id="slide-8" title="Creating an Interfaces Package" subtitle="Where Custom Interfaces Live" icon={Package}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Custom interfaces (for topics AND services) go in a separate C++ package. This is required because interfaces need to be compiled.
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
              <div className="pl-4">├── CMakeLists.txt</div>
              <div className="pl-4">└── package.xml</div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 9: Package Configuration */}
        <LectureSlide id="slide-9" title="Package Configuration" subtitle="Setting Up package.xml and CMakeLists.txt" icon={Settings}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Interface packages need special configuration to generate the code that ROS 2 uses.
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

        {/* Slide 10: Custom Service Interface */}
        <LectureSlide id="slide-10" title="Custom Service Interface" subtitle="Defining GetLocation.srv" icon={FileCode}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Let&apos;s create a service interface for getting the robot&apos;s location.
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Create the service interface file:</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws/src/my_robot_interfaces/srv && touch GetLocation.srv"
                title="Terminal"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Write the interface definition:</h4>
              <CodeBlock 
                filename="srv/GetLocation.srv"
                code={getLocationSrvCode}
              />
            </div>
          </div>

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">Request (above ---)</h4>
              <p className="text-blue-800 text-sm">Empty in this case. The client just asks &quot;give me the location&quot; without sending any data.</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">Response (below ---)</h4>
              <p className="text-green-800 text-sm">Server returns x, y coordinates and which zone the robot is in.</p>
            </div>
          </div>

          <div className="mt-6 p-4 bg-purple-50 border border-purple-200 rounded-lg">
            <h4 className="font-bold text-purple-900 mb-2">📝 Naming Rules for .srv Files</h4>
            <ul className="text-purple-800 text-sm space-y-1">
              <li>• Use PascalCase: <code className="bg-white px-1 rounded">GetLocation</code>, not <code className="bg-white px-1 rounded">get_location</code></li>
              <li>• Use verbs: <code className="bg-white px-1 rounded">GetLocation</code>, <code className="bg-white px-1 rounded">AssignTask</code>, <code className="bg-white px-1 rounded">ResetRobot</code></li>
              <li>• Field names use snake_case: <code className="bg-white px-1 rounded">battery_level</code>, <code className="bg-white px-1 rounded">zone_name</code></li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 11: Building the Interface */}
        <LectureSlide id="slide-11" title="Building the Interface" subtitle="Compile and Verify" icon={Hammer}>
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
                command="ros2 interface show my_robot_interfaces/srv/GetLocation"
                output={`---
float64 x
float64 y
string zone`}
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

        {/* Slide 12: Service Server Code */}
        <LectureSlide id="slide-12" title="Service Server Code" subtitle="The Localization Node" icon={Code}>
          <div className="mb-4">
            <p className="text-zinc-600">
              Create a new file <code className="bg-zinc-100 px-2 py-1 rounded">localization.py</code> in your <code className="bg-zinc-100 px-2 py-1 rounded">my_robot_pkg</code> package.
            </p>
          </div>

          <CodeBlock 
            filename="localization.py"
            code={localizationServerCode}
          />
        </LectureSlide>

        {/* Slide 13: Understanding the Server */}
        <LectureSlide id="slide-13" title="Understanding the Server" subtitle="Key Parts Explained" icon={Search}>
          <div className="space-y-6">
            <div className="p-4 bg-purple-50 border-l-4 border-purple-500 rounded-r-lg">
              <h4 className="font-bold text-purple-900 mb-2">from my_robot_interfaces.srv import GetLocation</h4>
              <p className="text-purple-800 text-sm">Import the custom service interface we created. Note: <code>.srv</code> not <code>.msg</code></p>
            </div>

            <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
              <h4 className="font-bold text-blue-900 mb-2">self.create_service(GetLocation, &apos;get_location&apos;, self.callback)</h4>
              <p className="text-blue-800 text-sm">Create the service server with three arguments:</p>
              <ul className="text-blue-800 text-sm mt-2 space-y-1">
                <li>• <strong>Service type:</strong> GetLocation (the interface)</li>
                <li>• <strong>Service name:</strong> &apos;get_location&apos; (what clients call)</li>
                <li>• <strong>Callback:</strong> Function that handles requests</li>
              </ul>
            </div>

            <div className="p-4 bg-green-50 border-l-4 border-green-500 rounded-r-lg">
              <h4 className="font-bold text-green-900 mb-2">def get_location_callback(self, request, response):</h4>
              <p className="text-green-800 text-sm">The callback receives TWO parameters:</p>
              <ul className="text-green-800 text-sm mt-2 space-y-1">
                <li>• <strong>request:</strong> Data from the client (empty in this case)</li>
                <li>• <strong>response:</strong> Object to fill with data and return</li>
              </ul>
            </div>

            <div className="p-4 bg-yellow-50 border-l-4 border-yellow-500 rounded-r-lg">
              <h4 className="font-bold text-yellow-900 mb-2">return response</h4>
              <p className="text-yellow-800 text-sm">You MUST return the response object. This sends the data back to the client.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 14: Setup and Build */}
        <LectureSlide id="slide-14" title="Setup and Build" subtitle="Preparing to Run" icon={Wrench}>
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

        {/* Slide 15: Testing the Server */}
        <LectureSlide id="slide-15" title="Testing the Server" subtitle="Using CLI Before Writing a Client" icon={Play}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Before writing a client node, we can test the service using command line tools.
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 1: Start the server</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg localization"
                output={`[INFO] [localization]: Localization service ready!`}
                title="Terminal 1"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: List available services</h4>
              <TerminalBlock 
                command="ros2 service list"
                output={`/get_location
/localization/describe_parameters
/localization/get_parameter_types
...`}
                title="Terminal 2"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Call the service</h4>
              <TerminalBlock 
                command={`ros2 service call /get_location my_robot_interfaces/srv/GetLocation "{}"`}
                output={`requester: making request: my_robot_interfaces.srv.GetLocation_Request()

response:
my_robot_interfaces.srv.GetLocation_Response(x=0.32, y=-0.15, zone='warehouse_a')`}
                title="Terminal 2"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800">
              <strong>🎉 It works!</strong> The server received our request and responded with the robot&apos;s location.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 16: Service Client Code */}
        <LectureSlide id="slide-16" title="Service Client Code" subtitle="The Task Planner Node" icon={Code}>
          <div className="mb-4">
            <p className="text-zinc-600">
              Now let&apos;s create a client node that calls the service. Create <code className="bg-zinc-100 px-2 py-1 rounded">task_planner.py</code>
            </p>
          </div>

          <CodeBlock 
            filename="task_planner.py"
            code={locationClientCode}
          />
        </LectureSlide>

        {/* Slide 17: Understanding the Client */}
        <LectureSlide id="slide-17" title="Understanding the Client" subtitle="Key Parts Explained" icon={Search}>
          <div className="space-y-6">
            <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
              <h4 className="font-bold text-blue-900 mb-2">self.create_client(GetLocation, &apos;get_location&apos;)</h4>
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
              <h4 className="font-bold text-green-900 mb-2">request = GetLocation.Request()</h4>
              <p className="text-green-800 text-sm">Create a request object. If the request has fields, fill them here: <code>request.some_field = value</code></p>
            </div>

            <div className="p-4 bg-yellow-50 border-l-4 border-yellow-500 rounded-r-lg">
              <h4 className="font-bold text-yellow-900 mb-2">future = self.client.call_async(request)</h4>
              <p className="text-yellow-800 text-sm">Call the service asynchronously. Returns a &quot;future&quot; object.</p>
            </div>

            <div className="p-4 bg-orange-50 border-l-4 border-orange-500 rounded-r-lg">
              <h4 className="font-bold text-orange-900 mb-2">rclpy.spin_until_future_complete(self, future)</h4>
              <p className="text-orange-800 text-sm">Wait until the response arrives. After this, <code>future.result()</code> contains the response.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 18: Running Both Nodes */}
        <LectureSlide id="slide-18" title="Running Both Nodes" subtitle="Server and Client Together" icon={Activity}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Don&apos;t forget to add the entry point, build, and source before running!
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 1: Start the server (keep running)</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg localization"
                output={`[INFO] [localization]: Localization service ready!
[INFO] [localization]: Location requested: (0.32, -0.15) in warehouse_a
[INFO] [localization]: Location requested: (0.45, 0.22) in warehouse_a`}
                title="Terminal 1: Server"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Run the client</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg task_planner"
                output={`[INFO] [task_planner]: Task Planner ready!
[INFO] [task_planner]: Robot is at (0.32, -0.15) in warehouse_a`}
                title="Terminal 2: Client"
              />
            </div>
          </div>

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">✅ What Happened</h4>
              <ol className="text-green-800 text-sm space-y-1 list-decimal list-inside">
                <li>Client started and found the service</li>
                <li>Client sent a request</li>
                <li>Server processed and responded</li>
                <li>Client received the location</li>
              </ol>
            </div>
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">📝 Note</h4>
              <p className="text-blue-800 text-sm">The client runs once and exits. The server keeps running, ready for more requests.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 19: Robot Status Service */}
        <LectureSlide id="slide-19" title="Another Example: Robot Status" subtitle="Integrating with Our Existing System" icon={Bot}>
          <div className="mb-6">
            <p className="text-lg text-zinc-600">
              Let&apos;s create a service that combines data from our existing topics. This shows how services and topics work together.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">The Service Interface:</h4>
              <CodeBlock 
                filename="srv/GetRobotStatus.srv"
                code={getRobotStatusSrvCode}
              />
            </div>
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">What It Does:</h4>
              <ul className="text-zinc-700 space-y-2 text-sm">
                <li>• <strong>Subscribes</strong> to battery and speed topics</li>
                <li>• <strong>Stores</strong> the latest values</li>
                <li>• When called, <strong>returns</strong> a complete status</li>
                <li>• Determines if status is OK/WARNING/CRITICAL</li>
              </ul>
              <div className="mt-4 p-3 bg-blue-50 rounded">
                <p className="text-blue-800 text-sm">This is a common pattern: a service that aggregates data from multiple topics.</p>
              </div>
            </div>
          </div>

          <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">📝 Remember to:</h4>
            <ol className="text-amber-800 text-sm space-y-1 list-decimal list-inside">
              <li>Add <code className="bg-white px-1 rounded">GetRobotStatus.srv</code> to CMakeLists.txt</li>
              <li>Rebuild the interfaces package</li>
              <li>Source the workspace</li>
            </ol>
          </div>
        </LectureSlide>

        {/* Slide 20: CLI Tools */}
        <LectureSlide id="slide-20" title="CLI Tools for Services" subtitle="Commands You Need to Know" icon={Terminal}>
          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">List all services</h4>
              <TerminalBlock 
                command="ros2 service list"
                output={`/get_location
/get_robot_status
/localization/describe_parameters
...`}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Get service type</h4>
              <TerminalBlock 
                command="ros2 service type /get_location"
                output={`my_robot_interfaces/srv/GetLocation`}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Show interface definition</h4>
              <TerminalBlock 
                command="ros2 interface show my_robot_interfaces/srv/GetLocation"
                output={`---
float64 x
float64 y
string zone`}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">Call a service from terminal</h4>
              <TerminalBlock 
                command={`ros2 service call /get_location my_robot_interfaces/srv/GetLocation "{}"`}
                output={`response:
my_robot_interfaces.srv.GetLocation_Response(x=0.5, y=1.2, zone='warehouse_a')`}
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 21: Key Takeaways */}
        <LectureSlide id="slide-21" title="Key Takeaways" subtitle="Remember These" icon={Lightbulb}>
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

        {/* Slide 22: Summary */}
        <LectureSlide id="slide-22" title="What You Learned Today" subtitle="Lecture 4 Summary" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-8">
            <div>
              <h3 className="text-xl font-bold text-zinc-900 mb-6">Concepts Covered:</h3>
              <ul className="space-y-4">
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Services</strong> are for request-response communication</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Custom interfaces</strong> require a dedicated package</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Service servers</strong> handle incoming requests</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Service clients</strong> send requests and receive responses</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>CLI tools</strong> help test services without writing code</span>
                </li>
              </ul>
            </div>
            <div className="flex flex-col justify-center p-8 bg-zinc-50 rounded-2xl border border-zinc-200">
              <h4 className="text-zinc-400 font-bold uppercase tracking-widest text-sm mb-4">Next Lecture</h4>
              <div className="text-2xl font-bold text-zinc-900 mb-2">
                Building Service-Based Control
              </div>
              <p className="text-zinc-600">
                Task management with multiple services. Validation, error handling, and real-world patterns.
              </p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 23: Homework */}
        <LectureSlide id="slide-23" title="Homework" subtitle="Create a Task Query Service" icon={Target}>
          <div className="space-y-6">
            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">1</span>
                Create a custom service interface
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• File: <code className="bg-zinc-100 px-2 py-1 rounded">GetTaskInfo.srv</code></li>
                <li>• Request: empty (no fields needed)</li>
                <li>• Response should include:
                  <ul className="ml-6 mt-2 space-y-1">
                    <li>- <code className="bg-zinc-100 px-1 rounded">string task_id</code></li>
                    <li>- <code className="bg-zinc-100 px-1 rounded">string destination</code></li>
                    <li>- <code className="bg-zinc-100 px-1 rounded">string status</code> (idle, active, completed)</li>
                    <li>- <code className="bg-zinc-100 px-1 rounded">float64 progress</code> (0.0 to 100.0)</li>
                  </ul>
                </li>
              </ul>
            </div>

            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">2</span>
                Create a task_manager server node
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Service name: <code className="bg-zinc-100 px-2 py-1 rounded">/get_task_info</code></li>
                <li>• Store a simulated task (hardcoded values are fine)</li>
                <li>• Return the task info when called</li>
              </ul>
            </div>

            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">3</span>
                Test using the CLI
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Start your server node</li>
                <li>• Use <code className="bg-zinc-100 px-2 py-1 rounded">ros2 service call</code> to test it</li>
                <li>• Verify you get the expected response</li>
              </ul>
            </div>

            <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-2">🎯 Expected Output</h4>
              <TerminalBlock 
                command={`ros2 service call /get_task_info my_robot_interfaces/srv/GetTaskInfo "{}"`}
                output={`response:
my_robot_interfaces.srv.GetTaskInfo_Response(
  task_id='TASK-001',
  destination='zone_b',
  status='active',
  progress=45.0
)`}
                title="When you call the service"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 24: Next Lecture */}
        <LectureSlide id="slide-24" title="Next Lecture" subtitle="More Powerful Services" icon={ArrowRight}>
          <div className="min-h-[40vh] flex flex-col items-center justify-center text-center">
            <p className="text-3xl md:text-4xl font-bold text-zinc-900 mb-8 leading-tight">
              One service is useful.<br/>
              <span className="text-green-600">Multiple services</span> make a control system.
            </p>
            <div className="w-16 h-1 bg-zinc-200 mb-8"></div>
            <p className="text-xl text-zinc-600 mb-8">
              assign_task, cancel_task, get_status<br/>
              Validation, error handling, state management
            </p>
            
            <div className="mt-8 inline-flex items-center px-6 py-3 bg-zinc-900 text-white rounded-full text-sm font-bold tracking-wide">
              NEXT: Service-Based Control Systems <ArrowRight size={16} className="ml-2" />
            </div>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
