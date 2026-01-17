'use client';

import { 
  Terminal, Battery, Cpu, Server, Play, Check, ChevronRight, 
  BookOpen, Layout, Settings, Code, ArrowRight, AlertTriangle, 
  Network, Search, Award, Users, Wrench, Package, FolderTree,
  Lightbulb, Zap, Target, Layers, HelpCircle, Bot, Cog,
  FileCode, GitBranch, Box, Rocket
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture1() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Course Introduction', icon: BookOpen },
    { id: 'slide-2', title: 'What This Course Is About', icon: Award },
    { id: 'slide-3', title: 'Prerequisites', icon: AlertTriangle },
    { id: 'slide-4', title: 'Why ROS Exists', icon: HelpCircle },
    { id: 'slide-5', title: 'What is ROS 2?', icon: Layers },
    { id: 'slide-6', title: 'The Four Pillars of ROS', icon: Package },
    { id: 'slide-7', title: 'When to Use ROS 2', icon: Target },
    { id: 'slide-8', title: 'ROS 2 Distributions', icon: GitBranch },
    { id: 'slide-9', title: 'Installation Overview', icon: Settings },
    { id: 'slide-10', title: 'Sourcing the Environment', icon: Terminal },
    { id: 'slide-11', title: 'The Project Vision', icon: Bot },
    { id: 'slide-12', title: 'Robot Software Architecture', icon: Cpu },
    { id: 'slide-13', title: 'The Core Problem', icon: Search },
    { id: 'slide-14', title: 'ROS 2 Workspaces', icon: FolderTree },
    { id: 'slide-15', title: 'Creating a Workspace', icon: Wrench },
    { id: 'slide-16', title: 'ROS 2 Packages', icon: Box },
    { id: 'slide-17', title: 'Creating a Python Package', icon: FileCode },
    { id: 'slide-18', title: 'Key Concept: Nodes', icon: Server },
    { id: 'slide-19', title: 'The Mental Model', icon: Network },
    { id: 'slide-20', title: 'Our First Node', icon: Play },
    { id: 'slide-21', title: 'Code Deep Dive', icon: Code },
    { id: 'slide-22', title: 'Running the Node', icon: Rocket },
    { id: 'slide-23', title: 'Inspecting the System', icon: Search },
    { id: 'slide-24', title: 'Summary', icon: Layout },
    { id: 'slide-25', title: 'Homework', icon: Battery },
    { id: 'slide-26', title: 'Closing Thought', icon: ArrowRight },
  ];

  const minimalNodeCode = `import rclpy
from rclpy.node import Node

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain')
        self.get_logger().info("Robot Brain is online and ready!")

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const nodeTemplateCode = `import rclpy
from rclpy.node import Node

class MyNodeName(Node):
    def __init__(self):
        # 1. Call parent constructor with node name
        super().__init__('my_node_name')
        
        # 2. Initialize your node (publishers, subscribers, timers, etc.)
        self.get_logger().info("Node has started!")

def main(args=None):
    # 3. Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # 4. Create an instance of your node
    node = MyNodeName()
    
    # 5. Keep the node alive and processing callbacks
    rclpy.spin(node)
    
    # 6. Clean shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={1} 
        lectureTitle="Foundations" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Course Introduction */}
        <LectureSlide id="slide-1" title="ROS 2 Fundamentals" subtitle="Building an Autonomous Delivery Robot from Scratch" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Instructor</h4>
                <p className="font-medium text-zinc-900">Abubakar-Sadiq Saidu</p>
                <p className="text-sm text-zinc-500">AI/ML Lead, RAIN-INN ABU</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Duration</h4>
                <p className="font-medium text-zinc-900">4 Weeks</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Language</h4>
                <p className="font-medium text-zinc-900">Python 3</p>
              </div>
            </div>
            <div className="mt-8 pt-8 border-t border-zinc-200">
              <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-4">What You&apos;ll Build</h4>
              <p className="text-lg text-zinc-700">
                A complete autonomous delivery robot system with sensors, navigation, decision-making, and monitoring using a similar architecture found in industrial robots, self-driving cars, and drones.
              </p>
            </div>
          </div>
          
          <div className="mt-8 p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <p className="text-lg text-blue-900">
              🧠 <strong>By the end of this course</strong>, you won&apos;t just &quot;know ROS 2&quot;. You&apos;ll understand how real robots are structured and how software makes them intelligent.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 2: What This Course Is About */}
        <LectureSlide id="slide-2" title="What This Course Is REALLY About" icon={Award}>
          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 bg-red-50/50 border border-red-100 rounded-xl">
              <h3 className="text-red-900 font-bold mb-4 flex items-center"><span className="mr-2 text-xl">❌</span> This course is NOT:</h3>
              <ul className="space-y-3 text-red-800/80">
                <li className="flex items-start"><span className="mr-2">•</span>Just learning ROS 2 commands</li>
                <li className="flex items-start"><span className="mr-2">•</span>Memorizing APIs</li>
                <li className="flex items-start"><span className="mr-2">•</span>Copy-pasting code without understanding</li>
              </ul>
            </div>
            <div className="p-6 bg-green-50/50 border border-green-100 rounded-xl">
              <h3 className="text-green-900 font-bold mb-4 flex items-center"><span className="mr-2 text-xl">✅</span> This course IS:</h3>
              <ul className="space-y-3 text-green-800/80">
                <li className="flex items-start"><span className="mr-2">•</span>Learning how robot software is <strong>designed</strong></li>
                <li className="flex items-start"><span className="mr-2">•</span>Understanding how programs <strong>cooperate</strong></li>
                <li className="flex items-start"><span className="mr-2">•</span>Thinking like a robotics engineer</li>
                <li className="flex items-start"><span className="mr-2">•</span>Building on proven foundations</li>
              </ul>
            </div>
          </div>
          <div className="mt-12 text-center p-6 bg-zinc-900 text-white rounded-xl shadow-xl">
            <p className="text-xl font-medium">ROS 2 is just the tool. <span className="text-yellow-400 font-bold">Systems thinking</span> is the real skill.</p>
          </div>
        </LectureSlide>

        {/* Slide 3: Prerequisites */}
        <LectureSlide id="slide-3" title="Before We Begin: Prerequisites" subtitle="What You Need to Know" icon={AlertTriangle}>
          <div className="grid md:grid-cols-3 gap-6">
            <div className="p-6 border border-zinc-200 rounded-xl hover:shadow-md transition-shadow">
              <div className="w-12 h-12 bg-zinc-100 rounded-full flex items-center justify-center mb-4"><Terminal size={24} /></div>
              <h3 className="font-bold text-zinc-900 mb-2">Linux Command Line</h3>
              <p className="text-zinc-600 text-sm">Navigate directories, run commands, edit files. You don&apos;t need to be an expert, just know the basics.</p>
              <p className="text-zinc-400 text-xs mt-3 font-medium">REQUIRED</p>
            </div>
            <div className="p-6 border border-zinc-200 rounded-xl hover:shadow-md transition-shadow">
              <div className="w-12 h-12 bg-zinc-100 rounded-full flex items-center justify-center mb-4"><Code size={24} /></div>
              <h3 className="font-bold text-zinc-900 mb-2">Python Programming</h3>
              <p className="text-zinc-600 text-sm">Variables, functions, classes (OOP). ROS 2 uses OOP heavily, so understanding classes is a big plus.</p>
              <p className="text-zinc-400 text-xs mt-3 font-medium">REQUIRED</p>
            </div>
            <div className="p-6 border border-zinc-200 rounded-xl hover:shadow-md transition-shadow">
              <div className="w-12 h-12 bg-zinc-100 rounded-full flex items-center justify-center mb-4"><Settings size={24} /></div>
              <h3 className="font-bold text-zinc-900 mb-2">Ubuntu Linux</h3>
              <p className="text-zinc-600 text-sm">ROS 2 runs on Ubuntu. Dual boot, VM, or native installation all work.</p>
              <p className="text-zinc-400 text-xs mt-3 font-medium">REQUIRED</p>
            </div>
          </div>
          
          <div className="mt-8 p-6 bg-amber-50 border border-amber-200 rounded-xl">
            <h4 className="font-bold text-amber-900 mb-2">⚠️ Important Advice</h4>
            <p className="text-amber-800">
              Learning ROS 2 is already challenging. If you&apos;re starting Linux, Python, AND ROS 2 from scratch simultaneously, it can be overwhelming. Consider spending a few hours on Linux and Python basics first. It will make this course much easier.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 4: Why ROS Exists */}
        <LectureSlide id="slide-4" title="Why Does ROS Exist?" subtitle="The Problem It Solves" icon={HelpCircle}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">🔄 The &quot;Reinventing the Wheel&quot; Problem</h3>
            <p className="text-lg text-zinc-600">Imagine starting a new robotics project...</p>
          </div>

          <div className="relative">
            <div className="absolute left-4 top-0 bottom-0 w-0.5 bg-zinc-200"></div>
            <div className="space-y-6 pl-12">
              <div className="relative">
                <div className="absolute -left-12 w-8 h-8 rounded-full bg-zinc-200 flex items-center justify-center font-bold text-sm">1</div>
                <div className="p-4 bg-zinc-50 rounded-lg">
                  <p><strong>Week 1-4:</strong> Design specifications, realize existing solutions don&apos;t fit</p>
                </div>
              </div>
              <div className="relative">
                <div className="absolute -left-12 w-8 h-8 rounded-full bg-zinc-200 flex items-center justify-center font-bold text-sm">2</div>
                <div className="p-4 bg-zinc-50 rounded-lg">
                  <p><strong>Months 1-6:</strong> Still writing basic wheel control and sensor drivers</p>
                </div>
              </div>
              <div className="relative">
                <div className="absolute -left-12 w-8 h-8 rounded-full bg-zinc-200 flex items-center justify-center font-bold text-sm">3</div>
                <div className="p-4 bg-zinc-50 rounded-lg">
                  <p><strong>Year 1-2:</strong> Finally have a robot running but haven&apos;t started actual research/product yet</p>
                </div>
              </div>
              <div className="relative">
                <div className="absolute -left-12 w-8 h-8 rounded-full bg-red-400 flex items-center justify-center font-bold text-sm text-white">4</div>
                <div className="p-4 bg-red-50 border border-red-100 rounded-lg">
                  <p><strong>Project ends:</strong> Code is too specific, next person starts from scratch</p>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-10 p-6 bg-zinc-900 text-white rounded-xl">
            <p className="text-lg">
              <strong className="text-yellow-400">This happens constantly.</strong> People keep reinventing the wheel. ROS was created to stop this by providing a <strong>standard framework</strong> you can use on any robot.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 5: What is ROS 2 */}
        <LectureSlide id="slide-5" title="What is ROS 2?" subtitle="Clearing Up the Confusion" icon={Layers}>
          <div className="mb-8">
            <div className="p-6 bg-red-50 border border-red-100 rounded-xl mb-6">
              <h3 className="text-red-900 font-bold mb-2">❌ What ROS is NOT</h3>
              <p className="text-red-800">Despite the name, ROS is <strong>NOT an operating system</strong>. It runs on top of Linux (Ubuntu).</p>
            </div>
            
            <div className="p-6 bg-green-50 border border-green-100 rounded-xl">
              <h3 className="text-green-900 font-bold mb-2">✅ What ROS IS</h3>
              <p className="text-green-800">ROS is a <strong>combination of four main parts</strong> that together provide everything you need to build robot software.</p>
            </div>
          </div>

          <div className="text-center py-8 px-6 bg-zinc-900 text-white rounded-2xl shadow-2xl">
            <p className="text-2xl md:text-3xl font-light leading-relaxed">
              &quot;ROS 2 is a <span className="text-blue-400 font-bold">communication framework</span> that lets <br className="hidden md:block"/>independent robot programs work together.&quot;
            </p>
          </div>
        </LectureSlide>

        {/* Slide 6: Four Pillars of ROS */}
        <LectureSlide id="slide-6" title="The Four Pillars of ROS" subtitle="What Makes Up the Ecosystem" icon={Package}>
          <div className="grid md:grid-cols-2 gap-6">
            <div className="p-6 border-2 border-purple-200 bg-purple-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <div className="w-10 h-10 bg-purple-100 rounded-lg flex items-center justify-center mr-3">
                  <Layers size={20} className="text-purple-600" />
                </div>
                <h3 className="font-bold text-zinc-900">1. Framework + Plumbing</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-3">Rules for building applications: packages, nodes, build system. The <strong>communication between nodes is handled for you</strong>.</p>
              <p className="text-purple-700 text-xs font-medium">Like having electrical wiring already done in a house</p>
            </div>
            
            <div className="p-6 border-2 border-blue-200 bg-blue-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <div className="w-10 h-10 bg-blue-100 rounded-lg flex items-center justify-center mr-3">
                  <Wrench size={20} className="text-blue-600" />
                </div>
                <h3 className="font-bold text-zinc-900">2. Developer Tools</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-3">CLI tools, visualization (RViz), simulation (Gazebo), logging, introspection, debugging.</p>
              <p className="text-blue-700 text-xs font-medium">See what your robot is doing in real-time</p>
            </div>
            
            <div className="p-6 border-2 border-green-200 bg-green-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <div className="w-10 h-10 bg-green-100 rounded-lg flex items-center justify-center mr-3">
                  <Zap size={20} className="text-green-600" />
                </div>
                <h3 className="font-bold text-zinc-900">3. Plug-and-Play Capabilities</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-3">Ready-made solutions: navigation stack, motion planning, SLAM. Install, configure, use.</p>
              <p className="text-green-700 text-xs font-medium">Saves months/years of development time</p>
            </div>
            
            <div className="p-6 border-2 border-orange-200 bg-orange-50/30 rounded-xl">
              <div className="flex items-center mb-4">
                <div className="w-10 h-10 bg-orange-100 rounded-lg flex items-center justify-center mr-3">
                  <Users size={20} className="text-orange-600" />
                </div>
                <h3 className="font-bold text-zinc-900">4. Community</h3>
              </div>
              <p className="text-zinc-600 text-sm mb-3">Open source, GitHub repos, Robotics Stack Exchange, ROS Discourse forums.</p>
              <p className="text-orange-700 text-xs font-medium">Get help, contribute, stay updated</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 7: When to Use ROS 2 */}
        <LectureSlide id="slide-7" title="When Should You Use ROS 2?" subtitle="It's Not Always the Answer" icon={Target}>
          <div className="mb-8">
            <p className="text-lg text-zinc-600">A robot system has three categories: <strong>Sensors</strong> (input), <strong>Controllers</strong> (brain), and <strong>Actuators</strong> (output).</p>
          </div>

          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 bg-red-50 border border-red-100 rounded-xl">
              <h3 className="font-bold text-red-900 mb-4">❌ DON&apos;T use ROS 2 for:</h3>
              <ul className="space-y-3 text-red-800 text-sm">
                <li className="flex items-start">
                  <span className="mr-2">•</span>
                  <span><strong>Simple camera trigger</strong> — Take photo on button press, send to server. Just use Python libraries.</span>
                </li>
                <li className="flex items-start">
                  <span className="mr-2">•</span>
                  <span><strong>Servo door opener</strong> — IR sensor triggers motor. Arduino is enough.</span>
                </li>
                <li className="flex items-start">
                  <span className="mr-2">•</span>
                  <span><strong>Line-following robot</strong> — Classic student project. A simple microcontroller works fine.</span>
                </li>
              </ul>
              <p className="mt-4 text-red-600 text-xs italic">Using ROS 2 here = over-engineering</p>
            </div>
            
            <div className="p-6 bg-green-50 border border-green-100 rounded-xl">
              <h3 className="font-bold text-green-900 mb-4">✅ USE ROS 2 for:</h3>
              <ul className="space-y-3 text-green-800 text-sm">
                <li className="flex items-start">
                  <span className="mr-2">•</span>
                  <span><strong>Autonomous mobile robot</strong> — Laser scan, mapping, navigation, motor control, simulation</span>
                </li>
                <li className="flex items-start">
                  <span className="mr-2">•</span>
                  <span><strong>Robotic arm with planning</strong> — 6-axis control, motion planning, collision avoidance</span>
                </li>
                <li className="flex items-start">
                  <span className="mr-2">•</span>
                  <span><strong>Multi-robot systems</strong> — Arms, conveyors, mobile robots working together</span>
                </li>
              </ul>
              <p className="mt-4 text-green-600 text-xs italic">Complex + modular + needs existing plugins = ROS 2</p>
            </div>
          </div>

          <div className="mt-8 p-4 bg-zinc-100 rounded-lg text-center">
            <p className="text-zinc-700">
              <strong>Rule of thumb:</strong> If your system has many sensors, actuators, or you need path planning/SLAM/simulation — ROS 2 will save you <em>months</em>.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 8: ROS 2 Distributions */}
        <LectureSlide id="slide-8" title="ROS 2 Distributions" subtitle="Choosing the Right Version" icon={GitBranch}>
          <div className="mb-8">
            <p className="text-lg text-zinc-600">A <strong>distribution</strong> is a stable release of ROS 2. New ones come out every May (World Turtle Day 🐢).</p>
          </div>

          <div className="overflow-x-auto">
            <table className="w-full text-sm">
              <thead>
                <tr className="bg-zinc-100">
                  <th className="px-4 py-3 text-left font-bold">Distribution</th>
                  <th className="px-4 py-3 text-left font-bold">Ubuntu</th>
                  <th className="px-4 py-3 text-left font-bold">Support</th>
                  <th className="px-4 py-3 text-left font-bold">Status</th>
                </tr>
              </thead>
              <tbody>
                <tr className="border-b border-zinc-100 bg-green-50">
                  <td className="px-4 py-3 font-medium">🐢 Humble Hawksbill</td>
                  <td className="px-4 py-3">22.04</td>
                  <td className="px-4 py-3">5 years (LTS)</td>
                  <td className="px-4 py-3"><span className="px-2 py-1 bg-green-200 text-green-800 rounded text-xs font-bold">RECOMMENDED</span></td>
                </tr>
                <tr className="border-b border-zinc-100">
                  <td className="px-4 py-3 font-medium">🐢 Iron Irwini</td>
                  <td className="px-4 py-3">22.04</td>
                  <td className="px-4 py-3">1.5 years</td>
                  <td className="px-4 py-3"><span className="px-2 py-1 bg-zinc-200 text-zinc-600 rounded text-xs">Non-LTS</span></td>
                </tr>
                <tr className="border-b border-zinc-100">
                  <td className="px-4 py-3 font-medium">🐢 Jazzy Jalisco</td>
                  <td className="px-4 py-3">24.04</td>
                  <td className="px-4 py-3">5 years (LTS)</td>
                  <td className="px-4 py-3"><span className="px-2 py-1 bg-blue-200 text-blue-800 rounded text-xs">Latest LTS</span></td>
                </tr>
              </tbody>
            </table>
          </div>

          <div className="mt-8 grid md:grid-cols-2 gap-6">
            <div className="p-4 bg-blue-50 border border-blue-100 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">📌 LTS = Long Term Support</h4>
              <p className="text-blue-800 text-sm">5 years of updates. Use LTS for learning, teaching, and production.</p>
            </div>
            <div className="p-4 bg-amber-50 border border-amber-100 rounded-lg">
              <h4 className="font-bold text-amber-900 mb-2">⚠️ Ubuntu + ROS Pairing</h4>
              <p className="text-amber-800 text-sm">Each ROS 2 distribution requires a specific Ubuntu version. Humble needs 22.04.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 9: Installation Overview */}
        <LectureSlide id="slide-9" title="Installing ROS 2 Humble" subtitle="Quick Reference Guide" icon={Settings}>
          <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl mb-8">
            <h4 className="font-bold text-blue-900 mb-2">📖 Official Documentation</h4>
            <p className="text-blue-800 text-sm">
              Always refer to: <a href="https://docs.ros.org/en/humble/Installation.html" target="_blank" rel="noopener noreferrer" className="underline font-medium">docs.ros.org/en/humble/Installation</a>
            </p>
          </div>

          <div className="space-y-4">
            <h4 className="font-bold text-zinc-900">High-Level Steps:</h4>
            
            <div className="space-y-3">
              <div className="flex items-center p-3 bg-zinc-50 rounded-lg">
                <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold text-sm mr-4">1</div>
                <span>Set locale to UTF-8</span>
              </div>
              <div className="flex items-center p-3 bg-zinc-50 rounded-lg">
                <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold text-sm mr-4">2</div>
                <span>Add ROS 2 apt repository and GPG key</span>
              </div>
              <div className="flex items-center p-3 bg-zinc-50 rounded-lg">
                <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold text-sm mr-4">3</div>
                <span>Install ROS 2 packages</span>
              </div>
            </div>

            <TerminalBlock 
              command="sudo apt update && sudo apt install ros-humble-desktop"
              title="Install ROS 2 Desktop"
            />

            <div className="flex items-center p-3 bg-zinc-50 rounded-lg">
              <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold text-sm mr-4">4</div>
              <span>Install development tools</span>
            </div>

            <TerminalBlock 
              command="sudo apt install ros-dev-tools"
              title="Development Tools"
            />
          </div>
        </LectureSlide>

        {/* Slide 10: Sourcing */}
        <LectureSlide id="slide-10" title="Sourcing the Environment" subtitle="The Most Important Step" icon={Terminal}>
          <div className="p-6 bg-red-50 border border-red-200 rounded-xl mb-8">
            <h4 className="font-bold text-red-900 mb-2">🚨 Critical Concept</h4>
            <p className="text-red-800 text-lg">
              ROS 2 doesn&apos;t exist until you <strong>source</strong> it. Every terminal, every time.
            </p>
          </div>

          <div className="mb-8">
            <h4 className="font-bold text-zinc-900 mb-4">What does &quot;sourcing&quot; mean?</h4>
            <p className="text-zinc-600 mb-4">It runs a script that adds ROS 2 commands and libraries to your current terminal session.</p>
            
            <TerminalBlock 
              command="source /opt/ros/humble/setup.bash"
              title="Source ROS 2"
            />
          </div>

          <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
            <h4 className="font-bold text-green-900 mb-4">✅ Make it Automatic</h4>
            <p className="text-green-800 mb-4">Add this line to your <code className="bg-white px-2 py-1 rounded">~/.bashrc</code> file so every new terminal is ready:</p>
            
            <TerminalBlock 
              command='echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'
              title="Add to bashrc"
            />
          </div>

          <div className="mt-8">
            <h4 className="font-bold text-zinc-900 mb-4">Verify Installation</h4>
            <TerminalBlock 
              command="ros2 run turtlesim turtlesim_node"
              title="Test"
              output="[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim"
            />
            <p className="text-zinc-500 text-sm mt-2">If you see a window with a turtle, ROS 2 is working! Press Ctrl+C to stop.</p>
          </div>
        </LectureSlide>

        {/* Slide 11: Project Vision */}
        <LectureSlide id="slide-11" title="The Robot We Will Build" subtitle="Our Course Project" icon={Bot}>
          <div className="bg-zinc-50 rounded-2xl p-8 mb-8 border border-zinc-100">
            <h3 className="text-2xl font-bold mb-6">🤖 Autonomous Delivery Robot</h3>
            <p className="mb-6 text-lg text-zinc-600">Imagine a robot in a warehouse that can:</p>
            <div className="grid grid-cols-2 md:grid-cols-5 gap-4 text-center">
              <div className="p-4 bg-white rounded-lg shadow-sm border border-zinc-100">
                <div className="text-2xl mb-2">📦</div>
                <div className="text-xs font-bold uppercase text-zinc-500">Receive Orders</div>
              </div>
              <div className="hidden md:flex items-center justify-center text-zinc-300"><ArrowRight /></div>
              <div className="p-4 bg-white rounded-lg shadow-sm border border-zinc-100">
                <div className="text-2xl mb-2">🗺️</div>
                <div className="text-xs font-bold uppercase text-zinc-500">Plan Path</div>
              </div>
              <div className="hidden md:flex items-center justify-center text-zinc-300"><ArrowRight /></div>
              <div className="p-4 bg-white rounded-lg shadow-sm border border-zinc-100">
                <div className="text-2xl mb-2">🚗</div>
                <div className="text-xs font-bold uppercase text-zinc-500">Navigate</div>
              </div>
            </div>
          </div>
          
          <p className="text-zinc-500 text-center">
            This architecture is used in real industrial robots, self-driving cars, drones, and space rovers.
          </p>
        </LectureSlide>

        {/* Slide 12: Robot Software Architecture */}
        <LectureSlide id="slide-12" title="What Happens Inside a Robot?" subtitle="The Software Responsibilities" icon={Cpu}>
          <div className="grid md:grid-cols-2 gap-x-12 gap-y-6">
            <div className="space-y-4">
              <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                <span className="text-2xl mr-4">👁</span>
                <div>
                  <h4 className="font-bold text-zinc-900">Perception</h4>
                  <p className="text-sm text-zinc-600">Cameras, LIDAR, sensors reading the world</p>
                </div>
              </div>
              <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                <span className="text-2xl mr-4">🧠</span>
                <div>
                  <h4 className="font-bold text-zinc-900">Decision Making</h4>
                  <p className="text-sm text-zinc-600">What task? What priority? What action?</p>
                </div>
              </div>
              <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                <span className="text-2xl mr-4">🧭</span>
                <div>
                  <h4 className="font-bold text-zinc-900">Navigation</h4>
                  <p className="text-sm text-zinc-600">Path planning, localization, obstacle avoidance</p>
                </div>
              </div>
              <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                <span className="text-2xl mr-4">⚙️</span>
                <div>
                  <h4 className="font-bold text-zinc-900">Motor Control</h4>
                  <p className="text-sm text-zinc-600">Wheels, arms, actuators</p>
                </div>
              </div>
              <div className="flex items-start p-4 bg-zinc-50 rounded-lg">
                <span className="text-2xl mr-4">📊</span>
                <div>
                  <h4 className="font-bold text-zinc-900">Monitoring</h4>
                  <p className="text-sm text-zinc-600">Battery, temperature, diagnostics</p>
                </div>
              </div>
            </div>
            <div className="flex items-center justify-center">
              <div className="p-8 bg-amber-50 border-2 border-amber-200 rounded-xl text-center">
                <h4 className="text-amber-900 font-bold mb-4 text-lg">💡 Key Insight</h4>
                <p className="text-amber-800">
                  These cannot be <strong>one giant program</strong>. They must be <strong>separate modules</strong> that communicate.
                </p>
                <p className="text-amber-600 text-sm mt-4">
                  This is exactly what ROS 2 enables.
                </p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 13: The Core Problem */}
        <LectureSlide id="slide-13" title="The Core Robotics Problem" subtitle="Why Modular Architecture Matters" icon={Search}>
          <div className="mb-10">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">The Reality:</h3>
            <p className="text-2xl text-zinc-700">Robots are <strong>many programs running simultaneously</strong>.</p>
          </div>
          
          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 bg-zinc-50 rounded-xl border border-zinc-200">
              <h4 className="font-bold text-zinc-900 mb-4">🤔 Questions That Arise:</h4>
              <ul className="space-y-3 text-zinc-700">
                <li className="flex items-start"><ChevronRight size={16} className="mr-2 mt-1 text-zinc-400" />How do programs talk to each other?</li>
                <li className="flex items-start"><ChevronRight size={16} className="mr-2 mt-1 text-zinc-400" />How do they share data safely?</li>
                <li className="flex items-start"><ChevronRight size={16} className="mr-2 mt-1 text-zinc-400" />How do we replace one part without breaking others?</li>
                <li className="flex items-start"><ChevronRight size={16} className="mr-2 mt-1 text-zinc-400" />How do we run across multiple computers?</li>
              </ul>
            </div>
            
            <div className="p-6 bg-green-50 rounded-xl border border-green-200">
              <h4 className="font-bold text-green-900 mb-4">✅ ROS 2 Provides:</h4>
              <ul className="space-y-3 text-green-800">
                <li className="flex items-start"><Check size={16} className="mr-2 mt-1" />Standardized communication (Topics, Services)</li>
                <li className="flex items-start"><Check size={16} className="mr-2 mt-1" />Type-safe message passing</li>
                <li className="flex items-start"><Check size={16} className="mr-2 mt-1" />Modular, replaceable components</li>
                <li className="flex items-start"><Check size={16} className="mr-2 mt-1" />Distributed computing support</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 14: ROS 2 Workspaces */}
        <LectureSlide id="slide-14" title="ROS 2 Workspaces" subtitle="Organizing Your Robot Software" icon={FolderTree}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">What is a Workspace?</h3>
            <p className="text-lg text-zinc-600">A workspace is simply a <strong>folder structure</strong> where you create and build your ROS 2 application.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 bg-zinc-900 text-white rounded-xl font-mono text-sm">
              <div className="text-zinc-400 mb-4"># Workspace Structure</div>
              <div className="space-y-1">
                <div>ros2_ws/</div>
                <div className="pl-4">├── src/           <span className="text-green-400"># Your code goes here</span></div>
                <div className="pl-4">│   ├── my_pkg_1/</div>
                <div className="pl-4">│   └── my_pkg_2/</div>
                <div className="pl-4">├── build/         <span className="text-zinc-500"># Build artifacts</span></div>
                <div className="pl-4">├── install/       <span className="text-blue-400"># Installed packages</span></div>
                <div className="pl-4">└── log/           <span className="text-zinc-500"># Build logs</span></div>
              </div>
            </div>
            
            <div className="space-y-4">
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h4 className="font-bold text-green-900">src/</h4>
                <p className="text-green-800 text-sm">Where you write all your code (packages)</p>
              </div>
              <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
                <h4 className="font-bold text-blue-900">install/</h4>
                <p className="text-blue-800 text-sm">Contains setup.bash you must source</p>
              </div>
              <div className="p-4 bg-zinc-50 border border-zinc-200 rounded-lg">
                <h4 className="font-bold text-zinc-700">build/ &amp; log/</h4>
                <p className="text-zinc-600 text-sm">Generated automatically, don&apos;t edit</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 15: Creating a Workspace */}
        <LectureSlide id="slide-15" title="Creating Your First Workspace" subtitle="Hands-On Setup" icon={Wrench}>
          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Create the workspace directory</h4>
              <TerminalBlock 
                command="mkdir -p ~/ros2_ws/src"
                title="Create Workspace"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: Build the (empty) workspace</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws && colcon build"
                title="Build"
                output="Starting >>> (no packages)\nSummary: 0 packages finished"
              />
              <p className="text-zinc-500 text-sm mt-2"><code>colcon</code> is the ROS 2 build tool. Run it from the workspace root.</p>
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Step 3: Source your workspace</h4>
              <TerminalBlock 
                command="source ~/ros2_ws/install/setup.bash"
                title="Source Workspace"
              />
            </div>

            <div className="p-6 bg-amber-50 border border-amber-200 rounded-xl">
              <h4 className="font-bold text-amber-900 mb-2">📝 Add Both to .bashrc</h4>
              <p className="text-amber-800 text-sm mb-4">Order matters! ROS 2 first, then your workspace:</p>
              <div className="bg-white p-4 rounded-lg font-mono text-sm">
                <div className="text-zinc-500"># ~/.bashrc</div>
                <div>source /opt/ros/humble/setup.bash</div>
                <div>source ~/ros2_ws/install/setup.bash</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 16: ROS 2 Packages */}
        <LectureSlide id="slide-16" title="ROS 2 Packages" subtitle="Organizing Nodes into Logical Units" icon={Box}>
          <div className="mb-8">
            <h3 className="text-xl font-bold text-zinc-900 mb-4">What is a Package?</h3>
            <p className="text-lg text-zinc-600">A package is a <strong>container for related nodes</strong>. It&apos;s how you organize sub-parts of your application.</p>
          </div>

          <div className="p-6 bg-zinc-50 rounded-xl border border-zinc-200 mb-8">
            <h4 className="font-bold text-zinc-900 mb-4">Example: Delivery Robot Packages</h4>
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
              <div className="p-4 bg-white border border-zinc-200 rounded-lg text-center">
                <div className="text-2xl mb-2">📷</div>
                <div className="font-mono text-sm">camera_pkg</div>
              </div>
              <div className="p-4 bg-white border border-zinc-200 rounded-lg text-center">
                <div className="text-2xl mb-2">🧭</div>
                <div className="font-mono text-sm">navigation_pkg</div>
              </div>
              <div className="p-4 bg-white border border-zinc-200 rounded-lg text-center">
                <div className="text-2xl mb-2">⚙️</div>
                <div className="font-mono text-sm">motor_control_pkg</div>
              </div>
              <div className="p-4 bg-white border border-zinc-200 rounded-lg text-center">
                <div className="text-2xl mb-2">🔋</div>
                <div className="font-mono text-sm">battery_pkg</div>
              </div>
            </div>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h4 className="font-bold text-blue-900 mb-2">🎯 Design Principle</h4>
              <p className="text-blue-800 text-sm">Each package is an <strong>independent unit</strong>, responsible for one sub-part of your application.</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h4 className="font-bold text-green-900 mb-2">✅ Benefits</h4>
              <p className="text-green-800 text-sm">Reusable, testable, shareable. Handles dependencies properly.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 17: Creating a Python Package */}
        <LectureSlide id="slide-17" title="Creating a Python Package" subtitle="Your First Package" icon={FileCode}>
          <div className="mb-6">
            <h4 className="font-bold text-zinc-900 mb-3">Navigate to src and create the package:</h4>
            <TerminalBlock 
              command="cd ~/ros2_ws/src"
              title="Navigate"
            />
            <TerminalBlock 
              command="ros2 pkg create my_robot_pkg --build-type ament_python --dependencies rclpy"
              title="Create Package"
              output="going to create a new package\npackage name: my_robot_pkg\n..."
            />
          </div>

          <div className="grid md:grid-cols-2 gap-8">
            <div className="p-6 bg-zinc-900 text-white rounded-xl font-mono text-sm">
              <div className="text-zinc-400 mb-4"># Package Structure</div>
              <div className="space-y-1">
                <div>my_robot_pkg/</div>
                <div className="pl-4">├── my_robot_pkg/</div>
                <div className="pl-8">├── __init__.py</div>
                <div className="pl-8 text-green-400">└── your_nodes.py  <span className="text-zinc-500"># Add here</span></div>
                <div className="pl-4">├── package.xml     <span className="text-blue-400"># Package info</span></div>
                <div className="pl-4">├── setup.py        <span className="text-yellow-400"># Install config</span></div>
                <div className="pl-4">└── setup.cfg</div>
              </div>
            </div>
            
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
                <h4 className="font-bold text-blue-900">package.xml</h4>
                <p className="text-blue-800 text-sm">Package metadata and dependencies</p>
              </div>
              <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
                <h4 className="font-bold text-yellow-900">setup.py</h4>
                <p className="text-yellow-800 text-sm">Build &amp; install instructions for Python nodes</p>
              </div>
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h4 className="font-bold text-green-900">my_robot_pkg/</h4>
                <p className="text-green-800 text-sm">Your Python nodes go in this inner folder</p>
              </div>
            </div>
          </div>
          
          <div className="mt-6">
            <h4 className="font-bold text-zinc-900 mb-3">Build the package:</h4>
            <TerminalBlock 
              command="cd ~/ros2_ws && colcon build"
              title="Build"
            />
          </div>
        </LectureSlide>

        {/* Slide 18: Key Concept - Nodes */}
        <LectureSlide id="slide-18" title="Key Concept: Nodes" subtitle="The Building Blocks of ROS 2" icon={Server}>
          <div className="mb-8">
            <h3 className="text-2xl font-bold text-zinc-900 mb-4">What is a Node?</h3>
            <p className="text-xl text-zinc-600">A node is a <strong>single running program</strong> with <strong>one clear responsibility</strong>.</p>
          </div>
          
          <div className="grid md:grid-cols-2 gap-8">
            <div className="bg-zinc-50 p-6 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">Example Nodes</h4>
              <div className="grid grid-cols-2 gap-4">
                <div className="p-3 bg-white border border-zinc-200 rounded text-center text-sm font-mono">battery_monitor</div>
                <div className="p-3 bg-white border border-zinc-200 rounded text-center text-sm font-mono">camera_driver</div>
                <div className="p-3 bg-white border border-zinc-200 rounded text-center text-sm font-mono">path_planner</div>
                <div className="p-3 bg-white border border-zinc-200 rounded text-center text-sm font-mono">motor_controller</div>
              </div>
            </div>
            <div className="flex flex-col justify-center space-y-4">
              <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
                <h4 className="font-bold text-blue-900 mb-2">🎯 The Golden Rule</h4>
                <p className="text-blue-800 font-medium text-lg">One node = One responsibility</p>
              </div>
              <div className="p-4 border border-zinc-200 rounded-xl">
                <h4 className="font-bold text-zinc-900 mb-2">Benefits</h4>
                <div className="flex flex-wrap gap-2 text-sm">
                  <span className="px-2 py-1 bg-zinc-100 rounded">Modular</span>
                  <span className="px-2 py-1 bg-zinc-100 rounded">Testable</span>
                  <span className="px-2 py-1 bg-zinc-100 rounded">Replaceable</span>
                  <span className="px-2 py-1 bg-zinc-100 rounded">Debuggable</span>
                </div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 19: Mental Model */}
        <LectureSlide id="slide-19" title="The Mental Model" subtitle="Think of ROS 2 as a City 🏙️" icon={Network}>
          <div className="relative p-8 bg-zinc-50 rounded-2xl border border-zinc-200 overflow-hidden mb-8">
            <div className="grid md:grid-cols-4 gap-6 relative z-10">
              <div className="text-center">
                <div className="w-16 h-16 bg-white border-2 border-zinc-900 rounded-full flex items-center justify-center mx-auto mb-4 text-2xl shadow-sm">👷</div>
                <h4 className="font-bold text-zinc-900">Nodes</h4>
                <p className="text-sm text-zinc-500">Workers</p>
              </div>
              <div className="text-center">
                <div className="w-16 h-16 bg-white border-2 border-zinc-900 rounded-full flex items-center justify-center mx-auto mb-4 text-2xl shadow-sm">🛣️</div>
                <h4 className="font-bold text-zinc-900">Topics</h4>
                <p className="text-sm text-zinc-500">Roads</p>
              </div>
              <div className="text-center">
                <div className="w-16 h-16 bg-white border-2 border-zinc-900 rounded-full flex items-center justify-center mx-auto mb-4 text-2xl shadow-sm">📨</div>
                <h4 className="font-bold text-zinc-900">Messages</h4>
                <p className="text-sm text-zinc-500">Letters</p>
              </div>
              <div className="text-center">
                <div className="w-16 h-16 bg-white border-2 border-zinc-900 rounded-full flex items-center justify-center mx-auto mb-4 text-2xl shadow-sm">📞</div>
                <h4 className="font-bold text-zinc-900">Services</h4>
                <p className="text-sm text-zinc-500">Phone calls</p>
              </div>
            </div>
          </div>
          
          <div className="p-6 bg-zinc-900 text-white rounded-xl">
            <p className="text-lg">
              <strong className="text-yellow-400">Today:</strong> We create our first worker (node).<br/>
              <strong className="text-blue-400">Next lecture:</strong> We teach workers to send letters to each other (topics &amp; messages).
            </p>
          </div>
        </LectureSlide>

        {/* Slide 20: Our First Node */}
        <LectureSlide id="slide-20" title="Our First Node" subtitle="The Simplest Possible ROS 2 Node" icon={Play}>
          <div className="text-center mb-10">
            <h3 className="text-xl font-bold mb-4">What will this node do?</h3>
            <p className="text-3xl mb-4">Almost nothing! 🎉</p>
          </div>
          
          <div className="max-w-xl mx-auto space-y-4">
            <h4 className="font-bold text-zinc-900 border-b border-zinc-200 pb-2">Its only jobs:</h4>
            <ul className="space-y-3">
              <li className="flex items-center"><Check size={20} className="text-green-500 mr-3" /> <strong>Exist</strong> as a ROS 2 node</li>
              <li className="flex items-center"><Check size={20} className="text-green-500 mr-3" /> <strong>Register</strong> itself with a name</li>
              <li className="flex items-center"><Check size={20} className="text-green-500 mr-3" /> <strong>Log</strong> a message so we know it&apos;s alive</li>
              <li className="flex items-center"><Check size={20} className="text-green-500 mr-3" /> <strong>Stay running</strong> until we stop it</li>
            </ul>
          </div>
          
          <div className="mt-10 p-6 bg-blue-50 border border-blue-100 rounded-xl text-center">
            <p className="text-blue-900 font-medium">
              🎯 Today&apos;s goal: <strong>Understand the structure</strong>, not build intelligence.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 21: Code Deep Dive */}
        <LectureSlide id="slide-21" title="The Code: A Minimal ROS 2 Node" subtitle="Every Line Explained" icon={Code}>
          <CodeBlock 
            filename="robot_brain.py"
            code={minimalNodeCode}
          />
          
          <div className="grid md:grid-cols-2 gap-6 mt-8">
            <div className="space-y-4">
              <div className="p-4 bg-purple-50 border-l-4 border-purple-500 rounded-r-lg">
                <h4 className="font-bold text-purple-900">rclpy</h4>
                <p className="text-purple-800 text-sm">ROS 2 Client Library for Python. Your gateway to ROS 2.</p>
              </div>
              <div className="p-4 bg-yellow-50 border-l-4 border-yellow-500 rounded-r-lg">
                <h4 className="font-bold text-yellow-900">Node (base class)</h4>
                <p className="text-yellow-800 text-sm">Inherit from this to get ROS 2 superpowers.</p>
              </div>
              <div className="p-4 bg-zinc-50 border-l-4 border-zinc-400 rounded-r-lg">
                <h4 className="font-bold text-zinc-900">super().__init__(&apos;name&apos;)</h4>
                <p className="text-zinc-700 text-sm">Register this node with a unique name.</p>
              </div>
            </div>
            <div className="space-y-4">
              <div className="p-4 bg-green-50 border-l-4 border-green-500 rounded-r-lg">
                <h4 className="font-bold text-green-900">get_logger().info()</h4>
                <p className="text-green-800 text-sm">Print messages with proper ROS 2 logging (not print()!).</p>
              </div>
              <div className="p-4 bg-blue-50 border-l-4 border-blue-500 rounded-r-lg">
                <h4 className="font-bold text-blue-900">rclpy.spin(node)</h4>
                <p className="text-blue-800 text-sm">Keep the node alive and processing callbacks.</p>
              </div>
              <div className="p-4 bg-red-50 border-l-4 border-red-400 rounded-r-lg">
                <h4 className="font-bold text-red-900">destroy_node() &amp; shutdown()</h4>
                <p className="text-red-800 text-sm">Clean exit. Always clean up!</p>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 22: Running the Node */}
        <LectureSlide id="slide-22" title="Running the Node" subtitle="What Happens Under the Hood" icon={Rocket}>
          <div className="mb-8">
            <h4 className="font-bold text-zinc-900 mb-4">Run your node:</h4>
            <TerminalBlock 
              command="python3 robot_brain.py"
              output="[INFO] [1706123456.789012] [robot_brain]: Robot Brain is online and ready!"
              title="Terminal"
            />
          </div>

          <div className="grid md:grid-cols-2 gap-8">
            <div className="space-y-4">
              <h3 className="font-bold text-zinc-900">What just happened:</h3>
              <ol className="space-y-3">
                <li className="flex items-start">
                  <span className="w-8 h-8 rounded-full bg-zinc-200 flex items-center justify-center font-bold mr-4 shrink-0">1</span>
                  <span><strong>rclpy.init()</strong> — ROS 2 communication layer started</span>
                </li>
                <li className="flex items-start">
                  <span className="w-8 h-8 rounded-full bg-zinc-200 flex items-center justify-center font-bold mr-4 shrink-0">2</span>
                  <span><strong>Node registered</strong> with name &quot;robot_brain&quot;</span>
                </li>
                <li className="flex items-start">
                  <span className="w-8 h-8 rounded-full bg-zinc-200 flex items-center justify-center font-bold mr-4 shrink-0">3</span>
                  <span><strong>Log message</strong> printed with timestamp</span>
                </li>
                <li className="flex items-start">
                  <span className="w-8 h-8 rounded-full bg-zinc-200 flex items-center justify-center font-bold mr-4 shrink-0">4</span>
                  <span><strong>spin()</strong> — Node waiting for work</span>
                </li>
              </ol>
            </div>
            <div className="p-8 bg-zinc-50 rounded-2xl border border-zinc-200 text-center flex flex-col justify-center">
              <div className="text-5xl mb-4">🌐</div>
              <p className="text-lg font-medium text-zinc-900">Even doing &quot;nothing&quot;...</p>
              <p className="text-zinc-600 mt-2">Your node is now <strong>discoverable</strong> by other nodes in the ROS 2 network!</p>
            </div>
          </div>
          
          <div className="mt-6 p-4 bg-zinc-100 rounded-lg text-center">
            <p className="text-zinc-700">Press <kbd className="px-2 py-1 bg-white rounded border">Ctrl+C</kbd> to stop the node.</p>
          </div>
        </LectureSlide>

        {/* Slide 23: Inspecting the System */}
        <LectureSlide id="slide-23" title="Inspecting the System" subtitle="ROS 2 CLI Tools" icon={Search}>
          <p className="mb-6 text-lg">ROS 2 is <strong>observable by design</strong>. You can see exactly what&apos;s running at any moment.</p>
          
          <div className="grid md:grid-cols-2 gap-6 mb-8">
            <div>
              <h4 className="font-bold text-zinc-900 mb-4">Terminal 1: Run your node</h4>
              <TerminalBlock 
                title="Terminal 1"
                command="python3 robot_brain.py"
                output="[INFO] [robot_brain]: Robot Brain is online and ready!"
              />
            </div>
            <div>
              <h4 className="font-bold text-zinc-900 mb-4">Terminal 2: List all nodes</h4>
              <TerminalBlock 
                title="Terminal 2"
                command="ros2 node list"
                output="/robot_brain"
              />
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">🔧 Essential ROS 2 CLI Commands</h4>
            <div className="grid md:grid-cols-2 gap-4 text-sm">
              <div className="flex items-center">
                <code className="bg-white px-2 py-1 rounded mr-2 font-mono">ros2 node list</code>
                <span className="text-blue-800">All running nodes</span>
              </div>
              <div className="flex items-center">
                <code className="bg-white px-2 py-1 rounded mr-2 font-mono">ros2 node info &lt;name&gt;</code>
                <span className="text-blue-800">Node details</span>
              </div>
              <div className="flex items-center">
                <code className="bg-white px-2 py-1 rounded mr-2 font-mono">ros2 topic list</code>
                <span className="text-blue-800">All topics (next lecture!)</span>
              </div>
              <div className="flex items-center">
                <code className="bg-white px-2 py-1 rounded mr-2 font-mono">ros2 run &lt;pkg&gt; &lt;node&gt;</code>
                <span className="text-blue-800">Run from a package</span>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 24: Summary */}
        <LectureSlide id="slide-24" title="What You Learned Today" subtitle="Lecture 1 Summary" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-8">
            <div>
              <h3 className="text-xl font-bold text-zinc-900 mb-6">Key Takeaways:</h3>
              <ul className="space-y-4">
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Why ROS 2 exists</strong> — Stop reinventing the wheel</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>The 4 pillars</strong> — Framework, Tools, Plugins, Community</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Workspaces &amp; Packages</strong> — How to organize code</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Nodes</strong> — One program, one responsibility</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>Your first node</strong> — The pattern you&apos;ll use forever</span>
                </li>
                <li className="flex items-start space-x-3">
                  <Check size={18} className="text-green-500 mt-1" />
                  <span><strong>CLI introspection</strong> — See what&apos;s running</span>
                </li>
              </ul>
            </div>
            <div className="flex flex-col justify-center p-8 bg-zinc-50 rounded-2xl border border-zinc-200">
              <h4 className="text-zinc-400 font-bold uppercase tracking-widest text-sm mb-4">Next Lecture</h4>
              <div className="text-2xl font-bold text-zinc-900 mb-2">
                Topics &amp; Publishers
              </div>
              <p className="text-zinc-600">
                Nodes will learn to talk! We&apos;ll publish sensor data and subscribe to receive it.
              </p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 25: Homework */}
        <LectureSlide id="slide-25" title="Homework" icon={Battery}>
          <div className="max-w-2xl mx-auto bg-white border border-zinc-200 shadow-xl rounded-2xl overflow-hidden">
            <div className="bg-zinc-900 p-4 text-white font-bold flex items-center">
              <Code size={20} className="mr-2"/> Assignment: Create Your Own Node
            </div>
            <div className="p-8">
              <div className="mb-6">
                <span className="text-zinc-500 text-sm uppercase tracking-wide font-bold">Task</span>
                <p className="text-lg font-medium mt-1">Create a node called:</p>
                <code className="block mt-2 bg-zinc-100 p-3 rounded text-zinc-900 font-mono font-bold text-lg">battery_monitor</code>
              </div>
              
              <div className="space-y-4">
                <div className="flex items-start">
                  <div className="w-6 h-6 rounded-full bg-green-100 text-green-600 flex items-center justify-center text-xs font-bold mr-3 mt-0.5">1</div>
                  <div>
                    <p className="font-bold text-zinc-900">Create the file</p>
                    <p className="text-zinc-500 text-sm"><code className="bg-zinc-100 px-1 rounded">battery_monitor.py</code></p>
                  </div>
                </div>
                <div className="flex items-start">
                  <div className="w-6 h-6 rounded-full bg-green-100 text-green-600 flex items-center justify-center text-xs font-bold mr-3 mt-0.5">2</div>
                  <div>
                    <p className="font-bold text-zinc-900">Create class <code className="bg-zinc-100 px-1 rounded">BatteryMonitor</code> extending Node</p>
                  </div>
                </div>
                <div className="flex items-start">
                  <div className="w-6 h-6 rounded-full bg-green-100 text-green-600 flex items-center justify-center text-xs font-bold mr-3 mt-0.5">3</div>
                  <div>
                    <p className="font-bold text-zinc-900">Log message:</p>
                    <p className="text-zinc-500 italic mt-1">&quot;Battery Monitor Active — Current level: 100%&quot;</p>
                  </div>
                </div>
                <div className="flex items-start">
                  <div className="w-6 h-6 rounded-full bg-green-100 text-green-600 flex items-center justify-center text-xs font-bold mr-3 mt-0.5">4</div>
                  <div>
                    <p className="font-bold text-zinc-900">Run it and verify with <code className="bg-zinc-100 px-1 rounded">ros2 node list</code></p>
                  </div>
                </div>
              </div>

              <div className="mt-8 pt-6 border-t border-zinc-100">
                <h4 className="font-bold text-zinc-900 mb-4">🎁 Bonus: Node Template</h4>
                <p className="text-zinc-600 text-sm mb-4">Use this pattern for any new node:</p>
              </div>
            </div>
          </div>
          
          <div className="mt-6">
            <CodeBlock 
              filename="node_template.py"
              code={nodeTemplateCode}
            />
          </div>
        </LectureSlide>

        {/* Slide 26: Closing Thought */}
        <LectureSlide id="slide-26" title="Closing Thought" icon={ArrowRight}>
          <div className="min-h-[40vh] flex flex-col items-center justify-center text-center">
            <p className="text-3xl md:text-5xl font-bold text-zinc-900 mb-8 leading-tight">
              Robotics is not about<br/>writing clever code.
            </p>
            <div className="w-16 h-1 bg-zinc-200 mb-8"></div>
            <p className="text-xl md:text-2xl text-zinc-600 font-light">
              It&apos;s about designing <strong>systems that cooperate</strong>.
            </p>
            
            <div className="mt-16 inline-flex items-center px-6 py-3 bg-zinc-900 text-white rounded-full text-sm font-bold tracking-wide">
              NEXT: Topics &amp; Publishers — Nodes Learn to Talk <ArrowRight size={16} className="ml-2" />
            </div>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
