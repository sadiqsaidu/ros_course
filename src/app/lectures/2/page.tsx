'use client';

import { 
  Terminal, Battery, Cpu, Server, Play, Check, ChevronRight, 
  BookOpen, Layout, Settings, Code, ArrowRight, AlertTriangle, 
  Network, Search, Award, Users, Wrench, Package, FolderTree,
  Lightbulb, Zap, Target, Layers, HelpCircle, Bot, Cog,
  FileCode, GitBranch, Box, Rocket, Radio, Send, Inbox,
  MessageSquare, RefreshCw, Eye, Activity, Tag
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture2() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Lecture 2: Topics', icon: BookOpen },
    { id: 'slide-2', title: 'Homework Solution', icon: Check },
    { id: 'slide-3', title: 'Code Walkthrough', icon: Code },
    { id: 'slide-4', title: 'Review: The Big Picture', icon: Layers },
    { id: 'slide-5', title: 'Review: What is a Node?', icon: Server },
    { id: 'slide-6', title: 'The Problem', icon: HelpCircle },
    { id: 'slide-7', title: 'The Solution: Topics', icon: Radio },
    { id: 'slide-8', title: 'Topic = Name + Interface', icon: Tag },
    { id: 'slide-9', title: 'Publisher and Subscriber', icon: Send },
    { id: 'slide-10', title: 'Our First Publisher', icon: Rocket },
    { id: 'slide-11', title: 'Publisher Code', icon: Code },
    { id: 'slide-12', title: 'Understanding the Code', icon: Search },
    { id: 'slide-13', title: 'Running the Publisher', icon: Play },
    { id: 'slide-14', title: 'Our First Subscriber', icon: Inbox },
    { id: 'slide-15', title: 'Subscriber Code', icon: Code },
    { id: 'slide-16', title: 'Understanding Callbacks', icon: RefreshCw },
    { id: 'slide-17', title: 'Running Both Nodes', icon: Activity },
    { id: 'slide-18', title: 'Multiple Subscribers', icon: Users },
    { id: 'slide-19', title: 'Visualizing with rqt_graph', icon: Network },
    { id: 'slide-20', title: 'Topic CLI Tools', icon: Terminal },
    { id: 'slide-21', title: 'Key Takeaways', icon: Lightbulb },
    { id: 'slide-22', title: 'Summary', icon: Layout },
    { id: 'slide-23', title: 'Homework', icon: Battery },
    { id: 'slide-24', title: 'Next Lecture', icon: ArrowRight },
  ];

  const homeworkSolutionCode = `import rclpy
from rclpy.node import Node

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.get_logger().info("Battery Monitor Active - Current level: 100%")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const batteryPublisherCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Create a publisher
        # Arguments: message type, topic name, queue size
        self.publisher = self.create_publisher(Float32, 'battery_level', 10)
        
        # Create a timer to publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_battery)
        
        # Simulate battery level
        self.battery_level = 100.0
        
        self.get_logger().info("Battery Monitor started!")
    
    def publish_battery(self):
        # Create the message
        msg = Float32()
        msg.data = self.battery_level
        
        # Publish it
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Battery: {self.battery_level}%")
        
        # Simulate battery drain
        self.battery_level -= 0.5

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const dashboardSubscriberCode = `import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard')
        
        # Create a subscriber
        # Arguments: message type, topic name, callback function, queue size
        self.subscription = self.create_subscription(
            Float32,
            'battery_level',
            self.battery_callback,
            10
        )
        
        self.get_logger().info("Dashboard started! Waiting for data...")
    
    def battery_callback(self, msg):
        # This function runs every time a message arrives
        battery = msg.data
        
        if battery > 50:
            status = "OK"
        elif battery > 20:
            status = "LOW"
        else:
            status = "CRITICAL"
        
        self.get_logger().info(f"Battery: {battery:.1f}% [{status}]")

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
            Float32,
            'battery_level',
            self.check_battery,
            10
        )
        
        self.get_logger().info("Safety Monitor active!")
    
    def check_battery(self, msg):
        if msg.data < 20.0:
            self.get_logger().warn(f"WARNING: Battery critically low! {msg.data:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={2} 
        lectureTitle="Topics" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Title */}
        <LectureSlide id="slide-1" title="Topics" subtitle="Sending and Receiving Messages Between Nodes" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Lecture</h4>
                <p className="font-medium text-zinc-900">2 of 12</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Focus</h4>
                <p className="font-medium text-zinc-900">Publishers &amp; Subscribers</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Goal</h4>
                <p className="font-medium text-zinc-900">Make nodes talk to each other</p>
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
                  <div className="text-2xl mb-1">🔄</div>
                  <div className="text-sm font-medium">Nodes Recap</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">📡</div>
                  <div className="text-sm font-medium">Topics</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1">🔧</div>
                  <div className="text-sm font-medium">Hands-On</div>
                </div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 2: Homework Solution */}
        <LectureSlide id="slide-2" title="Homework Solution" subtitle="The battery_monitor Node" icon={Check}>
          <div className="mb-8">
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h3 className="font-bold text-green-900 mb-2">📋 The Assignment Was:</h3>
              <p className="text-green-800">Create a <code className="bg-white px-2 py-1 rounded">battery_monitor</code> node that prints &quot;Battery Monitor Active&quot;</p>
            </div>
          </div>

          <div className="grid md:grid-cols-2 gap-8">
            <div>
              <h4 className="font-bold text-zinc-900 mb-4">What You Should Have:</h4>
              <ul className="space-y-3">
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>A Python file: <code className="bg-zinc-100 px-2 py-1 rounded">battery_monitor.py</code></span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>A class <code className="bg-zinc-100 px-2 py-1 rounded">BatteryMonitor</code> that extends <code className="bg-zinc-100 px-2 py-1 rounded">Node</code></span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>Node registered with name <code className="bg-zinc-100 px-2 py-1 rounded">battery_monitor</code></span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>A log message when the node starts</span>
                </li>
              </ul>
            </div>
            <div className="p-6 bg-zinc-50 rounded-xl border border-zinc-200">
              <h4 className="font-bold text-zinc-900 mb-4">Expected Output:</h4>
              <TerminalBlock 
                command="python3 battery_monitor.py"
                output="[INFO] [battery_monitor]: Battery Monitor Active - Current level: 100%"
                title="Terminal"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 3: Code Walkthrough */}
        <LectureSlide id="slide-3" title="Homework Code" subtitle="Let&apos;s Walk Through It" icon={Code}>
          <CodeBlock 
            filename="battery_monitor.py"
            code={homeworkSolutionCode}
          />
          
          <div className="mt-8 p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">🔍 The Pattern You Should Recognize:</h4>
            <div className="grid md:grid-cols-4 gap-4 text-sm">
              <div className="p-3 bg-white rounded-lg text-center">
                <div className="font-bold text-blue-900">1. Import</div>
                <div className="text-blue-700">rclpy and Node</div>
              </div>
              <div className="p-3 bg-white rounded-lg text-center">
                <div className="font-bold text-blue-900">2. Class</div>
                <div className="text-blue-700">Extends Node</div>
              </div>
              <div className="p-3 bg-white rounded-lg text-center">
                <div className="font-bold text-blue-900">3. Constructor</div>
                <div className="text-blue-700">super().__init__(&apos;name&apos;)</div>
              </div>
              <div className="p-3 bg-white rounded-lg text-center">
                <div className="font-bold text-blue-900">4. Main</div>
                <div className="text-blue-700">init → spin → shutdown</div>
              </div>
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <p className="text-amber-800">
              <strong>Question:</strong> This node logs a message once and then just sits there. How can we make it <em>continuously</em> report the battery level? And how can <em>other nodes</em> know about it?
            </p>
          </div>
        </LectureSlide>

        {/* Slide 4: Review - Big Picture */}
        <LectureSlide id="slide-4" title="Review: The Big Picture" subtitle="Where We Are in ROS 2" icon={Layers}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Our Delivery Robot Needs:</h3>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl font-mono text-sm mb-8">
            <div className="text-zinc-400 mb-4"># Project Structure</div>
            <div className="space-y-1">
              <div>ros2_ws/                    <span className="text-blue-400"># Workspace</span></div>
              <div className="pl-4">└── src/</div>
              <div className="pl-8">└── my_robot_pkg/       <span className="text-green-400"># Package</span></div>
              <div className="pl-12">└── my_robot_pkg/</div>
              <div className="pl-16">├── battery_monitor.py  <span className="text-yellow-400"># Node</span></div>
              <div className="pl-16">├── dashboard.py        <span className="text-yellow-400"># Node</span></div>
              <div className="pl-16">├── motor_controller.py <span className="text-yellow-400"># Node</span></div>
              <div className="pl-16">└── navigation.py       <span className="text-yellow-400"># Node</span></div>
            </div>
          </div>

          <div className="grid md:grid-cols-3 gap-6">
            <div className="p-4 border-2 border-blue-200 bg-blue-50/30 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-2">Workspace</h4>
              <p className="text-zinc-600 text-sm">The folder where all your ROS 2 code lives. You build from here.</p>
            </div>
            <div className="p-4 border-2 border-green-200 bg-green-50/30 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-2">Package</h4>
              <p className="text-zinc-600 text-sm">A container for related nodes. Has its own dependencies and build config.</p>
            </div>
            <div className="p-4 border-2 border-yellow-200 bg-yellow-50/30 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-2">Node</h4>
              <p className="text-zinc-600 text-sm">A single running program with one responsibility. The building block.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 5: Review - What is a Node */}
        <LectureSlide id="slide-5" title="Review: What is a Node?" subtitle="This is Important" icon={Server}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-2xl font-medium">
              A <span className="text-yellow-400 font-bold">node</span> is a single running program with <span className="text-blue-400 font-bold">one responsibility</span>.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div>
              <h4 className="font-bold text-zinc-900 mb-4">Each Node Does ONE Thing:</h4>
              <div className="space-y-3">
                <div className="flex items-center p-3 bg-zinc-50 rounded-lg">
                  <div className="w-10 h-10 bg-yellow-100 rounded-full flex items-center justify-center mr-3">🔋</div>
                  <div>
                    <div className="font-medium">battery_monitor</div>
                    <div className="text-sm text-zinc-500">Tracks battery level</div>
                  </div>
                </div>
                <div className="flex items-center p-3 bg-zinc-50 rounded-lg">
                  <div className="w-10 h-10 bg-blue-100 rounded-full flex items-center justify-center mr-3">📊</div>
                  <div>
                    <div className="font-medium">dashboard</div>
                    <div className="text-sm text-zinc-500">Displays robot status</div>
                  </div>
                </div>
                <div className="flex items-center p-3 bg-zinc-50 rounded-lg">
                  <div className="w-10 h-10 bg-green-100 rounded-full flex items-center justify-center mr-3">⚙️</div>
                  <div>
                    <div className="font-medium">motor_controller</div>
                    <div className="text-sm text-zinc-500">Controls wheel motors</div>
                  </div>
                </div>
              </div>
            </div>
            <div>
              <h4 className="font-bold text-zinc-900 mb-4">When a Node is Running:</h4>
              <ul className="space-y-3">
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>It has a <strong>unique name</strong> in the ROS 2 network</span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>It is <strong>discoverable</strong> by other nodes</span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>It <strong>spins</strong>, waiting for things to happen</span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>It can <strong>communicate</strong> with other nodes</span>
                </li>
              </ul>
              
              <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
                <p className="text-amber-800 font-medium">But HOW do nodes communicate?</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 6: The Problem */}
        <LectureSlide id="slide-6" title="The Problem" subtitle="Nodes Need to Share Data" icon={HelpCircle}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Our Delivery Robot Scenario:</h3>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">The battery_monitor node:</h4>
              <ul className="space-y-2 text-zinc-700">
                <li>• Knows the current battery level</li>
                <li>• Updates this value constantly</li>
                <li>• Has important data other nodes need</li>
              </ul>
            </div>
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">Other nodes that need battery data:</h4>
              <ul className="space-y-2 text-zinc-700">
                <li>• <strong>dashboard</strong> wants to display it</li>
                <li>• <strong>safety_monitor</strong> wants to check if it&apos;s too low</li>
                <li>• <strong>navigation</strong> wants to know if there&apos;s enough power</li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-red-50 border border-red-200 rounded-xl mb-8">
            <h4 className="font-bold text-red-900 mb-4">❌ The Challenge:</h4>
            <div className="grid md:grid-cols-3 gap-4">
              <div className="p-4 bg-white rounded-lg">
                <p className="text-red-800 text-sm">Each node runs as a <strong>separate program</strong></p>
              </div>
              <div className="p-4 bg-white rounded-lg">
                <p className="text-red-800 text-sm">They don&apos;t share memory</p>
              </div>
              <div className="p-4 bg-white rounded-lg">
                <p className="text-red-800 text-sm">They might run on <strong>different computers</strong></p>
              </div>
            </div>
          </div>

          <div className="text-center p-6 bg-zinc-900 text-white rounded-xl">
            <p className="text-xl">How can <span className="text-yellow-400 font-bold">battery_monitor</span> share its data with <span className="text-blue-400 font-bold">multiple other nodes</span> at the same time?</p>
          </div>
        </LectureSlide>

        {/* Slide 7: The Solution - Topics */}
        <LectureSlide id="slide-7" title="The Solution: Topics" subtitle="A Channel for Data" icon={Radio}>
          <div className="mb-8 p-6 bg-green-50 border border-green-200 rounded-xl">
            <h3 className="text-xl font-bold text-green-900 mb-2">✅ Topics</h3>
            <p className="text-green-800 text-lg">A <strong>topic</strong> is a named channel where nodes can send and receive messages.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="space-y-4">
              <h4 className="font-bold text-zinc-900">How it works:</h4>
              <div className="space-y-3">
                <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                  <span className="w-8 h-8 rounded-full bg-blue-500 text-white flex items-center justify-center font-bold mr-3">1</span>
                  <div>
                    <strong>A node publishes data</strong>
                    <p className="text-sm text-zinc-600">battery_monitor sends battery level to a topic</p>
                  </div>
                </div>
                <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                  <span className="w-8 h-8 rounded-full bg-blue-500 text-white flex items-center justify-center font-bold mr-3">2</span>
                  <div>
                    <strong>Other nodes subscribe</strong>
                    <p className="text-sm text-zinc-600">dashboard and safety_monitor listen to that topic</p>
                  </div>
                </div>
                <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                  <span className="w-8 h-8 rounded-full bg-blue-500 text-white flex items-center justify-center font-bold mr-3">3</span>
                  <div>
                    <strong>Subscribers receive automatically</strong>
                    <p className="text-sm text-zinc-600">Every time data is published, all subscribers get it</p>
                  </div>
                </div>
              </div>
            </div>
            
            <div className="p-6 bg-zinc-900 rounded-xl text-center flex flex-col justify-center">
              <div className="text-4xl mb-4">📡</div>
              <div className="text-white mb-2">Topic: <span className="text-yellow-400 font-mono">/battery_level</span></div>
              <div className="flex justify-center items-center gap-4 mt-4">
                <div className="p-2 bg-blue-600 rounded text-white text-xs">battery_monitor<br/>(publishes)</div>
                <ArrowRight className="text-zinc-500" />
                <div className="p-2 bg-zinc-700 rounded text-white text-xs">dashboard<br/>(subscribes)</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 8: Topic = Name + Interface */}
        <LectureSlide id="slide-8" title="Topic = Name + Interface" subtitle="Two Things Define a Topic" icon={MessageSquare}>
          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 border-2 border-purple-200 bg-purple-50/30 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-purple-500 text-white flex items-center justify-center mr-3">1</span>
                Topic Name
              </h3>
              <p className="text-zinc-600 mb-4">A unique identifier for the channel</p>
              <div className="space-y-2 font-mono text-sm">
                <div className="p-2 bg-white rounded">/battery_level</div>
                <div className="p-2 bg-white rounded">/robot/speed</div>
                <div className="p-2 bg-white rounded">/camera/image</div>
              </div>
              <p className="text-purple-700 text-xs mt-4">Names start with / and use lowercase with underscores</p>
            </div>
            
            <div className="p-6 border-2 border-blue-200 bg-blue-50/30 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-blue-500 text-white flex items-center justify-center mr-3">2</span>
                Message Interface
              </h3>
              <p className="text-zinc-600 mb-4">The data type being sent</p>
              <div className="space-y-2 font-mono text-sm">
                <div className="p-2 bg-white rounded">std_msgs/msg/Float32</div>
                <div className="p-2 bg-white rounded">std_msgs/msg/String</div>
                <div className="p-2 bg-white rounded">sensor_msgs/msg/Image</div>
              </div>
              <p className="text-blue-700 text-xs mt-4">Both publisher and subscriber must use the SAME interface</p>
            </div>
          </div>

          <div className="p-6 bg-amber-50 border border-amber-200 rounded-xl">
            <h4 className="font-bold text-amber-900 mb-2">💡 Common Message Types</h4>
            <div className="grid md:grid-cols-4 gap-4 text-sm">
              <div className="p-3 bg-white rounded-lg">
                <div className="font-bold text-amber-900">Float32</div>
                <div className="text-amber-700">Single decimal number</div>
              </div>
              <div className="p-3 bg-white rounded-lg">
                <div className="font-bold text-amber-900">Int64</div>
                <div className="text-amber-700">Whole number</div>
              </div>
              <div className="p-3 bg-white rounded-lg">
                <div className="font-bold text-amber-900">String</div>
                <div className="text-amber-700">Text</div>
              </div>
              <div className="p-3 bg-white rounded-lg">
                <div className="font-bold text-amber-900">Bool</div>
                <div className="text-amber-700">True/False</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 9: Publisher and Subscriber */}
        <LectureSlide id="slide-9" title="Publisher and Subscriber" subtitle="The Two Roles" icon={Send}>
          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 border-2 border-green-200 bg-green-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <div className="w-12 h-12 bg-green-100 rounded-full flex items-center justify-center mr-4">
                  <Send size={24} className="text-green-600" />
                </div>
                <h3 className="text-xl font-bold text-zinc-900">Publisher</h3>
              </div>
              <ul className="space-y-3 text-zinc-700">
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-green-500" />
                  <span><strong>Sends</strong> data to a topic</span>
                </li>
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-green-500" />
                  <span>Does not know who is listening</span>
                </li>
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-green-500" />
                  <span>Publishes at its own pace</span>
                </li>
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-green-500" />
                  <span>Created with <code className="bg-white px-1 rounded">create_publisher()</code></span>
                </li>
              </ul>
            </div>
            
            <div className="p-6 border-2 border-blue-200 bg-blue-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <div className="w-12 h-12 bg-blue-100 rounded-full flex items-center justify-center mr-4">
                  <Inbox size={24} className="text-blue-600" />
                </div>
                <h3 className="text-xl font-bold text-zinc-900">Subscriber</h3>
              </div>
              <ul className="space-y-3 text-zinc-700">
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-blue-500" />
                  <span><strong>Receives</strong> data from a topic</span>
                </li>
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-blue-500" />
                  <span>Does not know who is publishing</span>
                </li>
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-blue-500" />
                  <span>Reacts when new data arrives</span>
                </li>
                <li className="flex items-start">
                  <ChevronRight size={16} className="mr-2 mt-1 text-blue-500" />
                  <span>Created with <code className="bg-white px-1 rounded">create_subscription()</code></span>
                </li>
              </ul>
            </div>
          </div>

          <div className="mt-8 p-6 bg-zinc-900 text-white rounded-xl">
            <h4 className="font-bold text-yellow-400 mb-4">🔑 Key Point: Anonymous Communication</h4>
            <p className="text-lg">Publishers and subscribers don&apos;t know about each other. They only know the <strong>topic name</strong>. This makes the system <strong>flexible</strong> and <strong>modular</strong>.</p>
          </div>
        </LectureSlide>

        {/* Slide 10: Our First Publisher */}
        <LectureSlide id="slide-10" title="Our First Publisher" subtitle="Upgrading battery_monitor" icon={Rocket}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Let&apos;s Upgrade Our Homework</h3>
            <p className="text-lg text-zinc-600">Instead of just logging the battery level, we&apos;ll <strong>publish</strong> it to a topic so other nodes can use it.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <h4 className="font-bold text-red-900 mb-4">❌ Before (Homework)</h4>
              <ul className="space-y-2 text-red-800">
                <li>• Logs battery level once</li>
                <li>• No other node can access the data</li>
                <li>• Data stays inside the node</li>
              </ul>
            </div>
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-4">✅ After (Publisher)</h4>
              <ul className="space-y-2 text-green-800">
                <li>• Publishes battery level every second</li>
                <li>• Any node can subscribe and receive it</li>
                <li>• Data flows through the system</li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">What We Need:</h4>
            <div className="grid md:grid-cols-3 gap-4">
              <div className="p-4 bg-white rounded-lg text-center">
                <div className="font-bold">Topic Name</div>
                <code className="text-blue-600">battery_level</code>
              </div>
              <div className="p-4 bg-white rounded-lg text-center">
                <div className="font-bold">Message Type</div>
                <code className="text-blue-600">std_msgs/msg/Float32</code>
              </div>
              <div className="p-4 bg-white rounded-lg text-center">
                <div className="font-bold">Publish Rate</div>
                <code className="text-blue-600">Every 1 second</code>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 11: Publisher Code */}
        <LectureSlide id="slide-11" title="Publisher Code" subtitle="battery_monitor with Publishing" icon={Code}>
          <CodeBlock 
            filename="battery_monitor.py"
            code={batteryPublisherCode}
          />
        </LectureSlide>

        {/* Slide 12: Understanding the Code */}
        <LectureSlide id="slide-12" title="Understanding the Code" subtitle="Line by Line" icon={Search}>
          <div className="space-y-6">
            <div className="p-4 bg-purple-50 border-l-4 border-purple-500 rounded-r-lg">
              <h4 className="font-bold text-purple-900 mb-2">from std_msgs.msg import Float32</h4>
              <p className="text-purple-800 text-sm">Import the message type. <code>std_msgs</code> is a package with standard message types. <code>Float32</code> is a message containing a single decimal number.</p>
            </div>

            <div className="p-4 bg-green-50 border-l-4 border-green-500 rounded-r-lg">
              <h4 className="font-bold text-green-900 mb-2">self.create_publisher(Float32, &apos;battery_level&apos;, 10)</h4>
              <p className="text-green-800 text-sm mb-2">Create a publisher with three arguments:</p>
              <ul className="text-green-800 text-sm space-y-1 ml-4">
                <li>• <strong>Float32</strong>: The message type</li>
                <li>• <strong>&apos;battery_level&apos;</strong>: The topic name</li>
                <li>• <strong>10</strong>: Queue size (buffer for messages)</li>
              </ul>
            </div>

            <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
              <h4 className="font-bold text-blue-900 mb-2">self.create_timer(1.0, self.publish_battery)</h4>
              <p className="text-blue-800 text-sm">Create a timer that calls <code>publish_battery()</code> every 1.0 seconds. This is how we publish repeatedly.</p>
            </div>

            <div className="p-4 bg-yellow-50 border-l-4 border-yellow-500 rounded-r-lg">
              <h4 className="font-bold text-yellow-900 mb-2">msg = Float32() → msg.data = value → publisher.publish(msg)</h4>
              <p className="text-yellow-800 text-sm">Create a message object, fill in the data field, then publish it. This pattern is the same for all message types.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 13: Running the Publisher */}
        <LectureSlide id="slide-13" title="Running the Publisher" subtitle="Testing Our Code" icon={Play}>
          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Run the node</h4>
              <TerminalBlock 
                command="python3 battery_monitor.py"
                output={`[INFO] [battery_monitor]: Battery Monitor started!
[INFO] [battery_monitor]: Battery: 100.0%
[INFO] [battery_monitor]: Battery: 99.5%
[INFO] [battery_monitor]: Battery: 99.0%`}
                title="Terminal 1"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: In another terminal, check what topics exist</h4>
              <TerminalBlock 
                command="ros2 topic list"
                output={`/battery_level
/parameter_events
/rosout`}
                title="Terminal 2"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 3: Listen to the topic directly from terminal</h4>
              <TerminalBlock 
                command="ros2 topic echo /battery_level"
                output={`data: 95.5
---
data: 95.0
---
data: 94.5
---`}
                title="Terminal 2"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800">
              <strong>🎉 Success!</strong> The data is being published. Anyone can now subscribe to <code>/battery_level</code> and receive updates.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 14: Our First Subscriber */}
        <LectureSlide id="slide-14" title="Our First Subscriber" subtitle="Creating the Dashboard Node" icon={Inbox}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">Now Let&apos;s Create a Node That Receives the Data</h3>
            <p className="text-lg text-zinc-600">The <strong>dashboard</strong> node will subscribe to <code>/battery_level</code> and display the information.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">What the Dashboard Will Do:</h4>
              <ul className="space-y-3">
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>Subscribe to <code className="bg-white px-1 rounded">/battery_level</code></span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>Receive battery data automatically</span>
                </li>
                <li className="flex items-start">
                  <Check size={18} className="text-green-500 mr-3 mt-1" />
                  <span>Display the battery with a status (OK, LOW, CRITICAL)</span>
                </li>
              </ul>
            </div>
            
            <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-4">🔑 The Callback Pattern</h4>
              <p className="text-blue-800 mb-4">When you create a subscriber, you provide a <strong>callback function</strong>. This function runs automatically every time a new message arrives.</p>
              <div className="p-3 bg-white rounded font-mono text-sm">
                def battery_callback(self, msg):<br/>
                &nbsp;&nbsp;&nbsp;&nbsp;# This runs when data arrives<br/>
                &nbsp;&nbsp;&nbsp;&nbsp;print(msg.data)
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 15: Subscriber Code */}
        <LectureSlide id="slide-15" title="Subscriber Code" subtitle="The Dashboard Node" icon={Code}>
          <CodeBlock 
            filename="dashboard.py"
            code={dashboardSubscriberCode}
          />
        </LectureSlide>

        {/* Slide 16: Understanding Callbacks */}
        <LectureSlide id="slide-16" title="Understanding Callbacks" subtitle="How Subscribers Work" icon={RefreshCw}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">The Callback is the Heart of a Subscriber</h3>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
                <h4 className="font-bold text-blue-900 mb-2">create_subscription()</h4>
                <p className="text-blue-800 text-sm">Takes 4 arguments: message type, topic name, callback function, and queue size.</p>
              </div>

              <div className="p-4 bg-green-50 border-l-4 border-green-500 rounded-r-lg">
                <h4 className="font-bold text-green-900 mb-2">The Callback Function</h4>
                <p className="text-green-800 text-sm">A function you define. It receives the message as a parameter. ROS 2 calls this function automatically when data arrives.</p>
              </div>

              <div className="p-4 bg-purple-50 border-l-4 border-purple-500 rounded-r-lg">
                <h4 className="font-bold text-purple-900 mb-2">msg.data</h4>
                <p className="text-purple-800 text-sm">For <code>Float32</code>, the message has a <code>data</code> field containing the number. Other message types have different fields.</p>
              </div>
            </div>

            <div className="p-6 bg-zinc-900 text-white rounded-xl">
              <h4 className="text-yellow-400 font-bold mb-4">Timeline:</h4>
              <div className="space-y-4 font-mono text-sm">
                <div className="flex items-center">
                  <div className="w-24 text-zinc-400">t=0.0s</div>
                  <div>Node starts, waiting...</div>
                </div>
                <div className="flex items-center">
                  <div className="w-24 text-zinc-400">t=1.0s</div>
                  <div className="text-green-400">Message arrives → callback runs</div>
                </div>
                <div className="flex items-center">
                  <div className="w-24 text-zinc-400">t=2.0s</div>
                  <div className="text-green-400">Message arrives → callback runs</div>
                </div>
                <div className="flex items-center">
                  <div className="w-24 text-zinc-400">t=3.0s</div>
                  <div className="text-green-400">Message arrives → callback runs</div>
                </div>
                <div className="text-zinc-400 mt-4">... and so on</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 17: Running Both Nodes */}
        <LectureSlide id="slide-17" title="Running Both Nodes" subtitle="Seeing the Communication" icon={Activity}>
          <div className="mb-6">
            <h4 className="font-bold text-zinc-900 mb-3">Open two terminals and run:</h4>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-8">
            <div>
              <TerminalBlock 
                command="python3 battery_monitor.py"
                output={`[INFO] [battery_monitor]: Battery Monitor started!
[INFO] [battery_monitor]: Battery: 100.0%
[INFO] [battery_monitor]: Battery: 99.5%
[INFO] [battery_monitor]: Battery: 99.0%`}
                title="Terminal 1: Publisher"
              />
            </div>
            <div>
              <TerminalBlock 
                command="python3 dashboard.py"
                output={`[INFO] [dashboard]: Dashboard started! Waiting for data...
[INFO] [dashboard]: Battery: 99.5% [OK]
[INFO] [dashboard]: Battery: 99.0% [OK]
[INFO] [dashboard]: Battery: 98.5% [OK]`}
                title="Terminal 2: Subscriber"
              />
            </div>
          </div>

          <div className="p-6 bg-green-50 border border-green-200 rounded-xl mb-6">
            <h4 className="font-bold text-green-900 mb-2">🎉 They&apos;re Communicating!</h4>
            <p className="text-green-800">The dashboard receives battery updates automatically. You can stop and start the dashboard without affecting the publisher. Try it!</p>
          </div>

          <div className="p-4 bg-zinc-100 rounded-lg">
            <h4 className="font-bold text-zinc-900 mb-2">Try This:</h4>
            <ul className="space-y-1 text-zinc-700">
              <li>• Stop the dashboard, wait 5 seconds, start it again</li>
              <li>• Stop the publisher, see dashboard stop receiving</li>
              <li>• Run multiple dashboards at once (they all receive!)</li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 18: Multiple Subscribers */}
        <LectureSlide id="slide-18" title="Multiple Subscribers" subtitle="One Publisher, Many Listeners" icon={Users}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">The Power of Topics: Any Number of Subscribers</h3>
          </div>

          <div className="grid md:grid-cols-3 gap-6 mb-8">
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl text-center">
              <div className="text-3xl mb-2">🔋</div>
              <div className="font-bold">battery_monitor</div>
              <div className="text-sm text-zinc-500">Publisher</div>
            </div>
            <div className="flex items-center justify-center">
              <div className="text-center">
                <div className="text-2xl mb-1">📡</div>
                <div className="font-mono text-sm">/battery_level</div>
              </div>
            </div>
            <div className="space-y-3">
              <div className="p-3 bg-blue-50 border border-blue-200 rounded-lg text-center">
                <div className="font-bold text-sm">dashboard</div>
                <div className="text-xs text-zinc-500">Subscriber</div>
              </div>
              <div className="p-3 bg-blue-50 border border-blue-200 rounded-lg text-center">
                <div className="font-bold text-sm">safety_monitor</div>
                <div className="text-xs text-zinc-500">Subscriber</div>
              </div>
              <div className="p-3 bg-blue-50 border border-blue-200 rounded-lg text-center">
                <div className="font-bold text-sm">navigation</div>
                <div className="text-xs text-zinc-500">Subscriber</div>
              </div>
            </div>
          </div>

          <CodeBlock 
            filename="safety_monitor.py"
            code={safetyMonitorCode}
          />
        </LectureSlide>

        {/* Slide 19: Visualizing with rqt_graph */}
        <LectureSlide id="slide-19" title="Visualizing with rqt_graph" subtitle="See Your System" icon={Network}>
          <div className="mb-6">
            <h4 className="font-bold text-zinc-900 mb-3">Run rqt_graph to visualize the communication:</h4>
            <TerminalBlock 
              command="rqt_graph"
              title="Terminal"
            />
          </div>

          <div className="p-8 bg-zinc-50 rounded-xl border border-zinc-200 mb-8">
            <div className="flex items-center justify-center gap-8">
              <div className="p-4 bg-white border-2 border-zinc-300 rounded-lg text-center shadow-sm">
                <div className="font-mono text-sm">battery_monitor</div>
              </div>
              <div className="flex flex-col items-center">
                <ArrowRight className="text-zinc-400" />
                <div className="font-mono text-xs text-zinc-500 mt-1">/battery_level</div>
              </div>
              <div className="p-4 bg-white border-2 border-zinc-300 rounded-lg text-center shadow-sm">
                <div className="font-mono text-sm">dashboard</div>
              </div>
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">🔍 Why rqt_graph is Useful:</h4>
            <ul className="space-y-2 text-blue-800">
              <li>• See all nodes and topics in your system</li>
              <li>• Quickly spot if nodes are connected properly</li>
              <li>• Debug when messages aren&apos;t being received</li>
              <li>• Understand complex systems with many nodes</li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 20: Topic CLI Tools */}
        <LectureSlide id="slide-20" title="Topic CLI Tools" subtitle="Commands You Need to Know" icon={Terminal}>
          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">ros2 topic list</h4>
              <p className="text-zinc-600 text-sm mb-2">Show all active topics</p>
              <TerminalBlock command="ros2 topic list" output={`/battery_level
/parameter_events
/rosout`} />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">ros2 topic info &lt;topic&gt;</h4>
              <p className="text-zinc-600 text-sm mb-2">Get details about a topic</p>
              <TerminalBlock command="ros2 topic info /battery_level" output={`Type: std_msgs/msg/Float32
Publisher count: 1
Subscription count: 2`} />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">ros2 topic echo &lt;topic&gt;</h4>
              <p className="text-zinc-600 text-sm mb-2">Subscribe and print messages from terminal</p>
              <TerminalBlock command="ros2 topic echo /battery_level" output={`data: 85.5
---
data: 85.0`} />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-2">ros2 topic pub &lt;topic&gt; &lt;type&gt; &lt;data&gt;</h4>
              <p className="text-zinc-600 text-sm mb-2">Publish a message from terminal (for testing)</p>
              <TerminalBlock command="ros2 topic pub /battery_level std_msgs/msg/Float32 '{data: 50.0}'" />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 21: Key Takeaways */}
        <LectureSlide id="slide-21" title="Key Takeaways" subtitle="The Important Bits" icon={Lightbulb}>
          <div className="grid md:grid-cols-2 gap-8">
            <div className="space-y-4">
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h4 className="font-bold text-green-900 mb-2">✅ Topics are for continuous data</h4>
                <p className="text-green-800 text-sm">Sensor readings, status updates, anything that streams constantly.</p>
              </div>
              
              <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
                <h4 className="font-bold text-blue-900 mb-2">✅ One-way communication</h4>
                <p className="text-blue-800 text-sm">Publisher sends, subscriber receives. No response expected.</p>
              </div>
              
              <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg">
                <h4 className="font-bold text-purple-900 mb-2">✅ Anonymous and flexible</h4>
                <p className="text-purple-800 text-sm">Publishers and subscribers don&apos;t know about each other. Easy to add more.</p>
              </div>
            </div>

            <div className="space-y-4">
              <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
                <h4 className="font-bold text-yellow-900 mb-2">✅ Same interface required</h4>
                <p className="text-yellow-800 text-sm">Publisher and subscriber must use the same message type.</p>
              </div>
              
              <div className="p-4 bg-orange-50 border border-orange-200 rounded-lg">
                <h4 className="font-bold text-orange-900 mb-2">✅ Callbacks handle messages</h4>
                <p className="text-orange-800 text-sm">Your callback function runs automatically when data arrives.</p>
              </div>
              
              <div className="p-4 bg-zinc-100 border border-zinc-200 rounded-lg">
                <h4 className="font-bold text-zinc-900 mb-2">✅ Use CLI tools to debug</h4>
                <p className="text-zinc-700 text-sm">ros2 topic list, echo, info, and rqt_graph are your friends.</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 22: Summary */}
        <LectureSlide id="slide-22" title="What You Learned Today" subtitle="Lecture 2 Summary" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-8">
            <div>
              <h3 className="text-xl font-bold text-zinc-900 mb-6">Concepts Covered:</h3>
              <ul className="space-y-4">
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Topics</strong> are named channels for data</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Publishers</strong> send data to topics</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Subscribers</strong> receive data from topics</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Callbacks</strong> run when messages arrive</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Message types</strong> define data structure</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>CLI tools</strong> help debug and test</span>
                </li>
              </ul>
            </div>
            <div className="flex flex-col justify-center p-8 bg-zinc-50 rounded-2xl border border-zinc-200">
              <h4 className="text-zinc-400 font-bold uppercase tracking-widest text-sm mb-4">Next Lecture</h4>
              <div className="text-2xl font-bold text-zinc-900 mb-2">
                Multiple Topics &amp; Data Processing
              </div>
              <p className="text-zinc-600">
                Filter noisy sensor data, combine multiple topics, and build processing pipelines.
              </p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 23: Homework */}
        <LectureSlide id="slide-23" title="Homework" subtitle="Practice What You Learned" icon={Battery}>
          <div className="space-y-6">
            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">1</span>
                Create a speed_sensor node
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Publishes robot speed on topic <code className="bg-zinc-100 px-2 py-1 rounded">/robot/speed</code></li>
                <li>• Use message type <code className="bg-zinc-100 px-2 py-1 rounded">std_msgs/msg/Float32</code></li>
                <li>• Publish a simulated speed value every 0.5 seconds</li>
              </ul>
            </div>

            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">2</span>
                Update the dashboard node
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Subscribe to BOTH <code className="bg-zinc-100 px-2 py-1 rounded">/battery_level</code> AND <code className="bg-zinc-100 px-2 py-1 rounded">/robot/speed</code></li>
                <li>• Display both values when they update</li>
                <li>• Hint: You can have multiple subscribers in one node!</li>
              </ul>
            </div>

            <div className="p-6 bg-white border-2 border-zinc-200 rounded-xl">
              <h3 className="font-bold text-zinc-900 mb-4 flex items-center">
                <span className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center mr-3">3</span>
                Create a safety_monitor node
              </h3>
              <ul className="space-y-2 text-zinc-700 ml-11">
                <li>• Subscribe to <code className="bg-zinc-100 px-2 py-1 rounded">/battery_level</code></li>
                <li>• Log a WARNING if battery drops below 20%</li>
                <li>• Use <code className="bg-zinc-100 px-2 py-1 rounded">self.get_logger().warn()</code> for warnings</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 24: Closing */}
        <LectureSlide id="slide-24" title="Next Lecture" subtitle="Building on What You Learned" icon={ArrowRight}>
          <div className="min-h-[40vh] flex flex-col items-center justify-center text-center">
            <p className="text-3xl md:text-4xl font-bold text-zinc-900 mb-8 leading-tight">
              Nodes that talk to each other<br/>
              are nodes that work <span className="text-blue-600">together</span>.
            </p>
            <div className="w-16 h-1 bg-zinc-200 mb-8"></div>
            <p className="text-xl text-zinc-600">
              You now have the fundamental skill of ROS 2: <strong>communication</strong>.
            </p>
            
            <div className="mt-16 inline-flex items-center px-6 py-3 bg-zinc-900 text-white rounded-full text-sm font-bold tracking-wide">
              NEXT: Multiple Subscribers &amp; Data Processing <ArrowRight size={16} className="ml-2" />
            </div>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
