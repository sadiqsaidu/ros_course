'use client';

import { 
  Terminal, Battery, Cpu, Server, Play, Check, ChevronRight, 
  BookOpen, Layout, Settings, Code, ArrowRight, AlertTriangle, 
  Network, Search, Award, Users, Wrench, Package, FolderTree,
  Lightbulb, Zap, Target, Layers, HelpCircle, Bot, Cog,
  FileCode, GitBranch, Box, Rocket, Radio, Send, Inbox,
  MessageSquare, RefreshCw, Eye, Activity, Filter, Gauge,
  CircuitBoard, Workflow, Combine, BarChart3, Sparkles,
  Sliders, FileJson, Variable, RotateCcw
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture6() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Lecture 6: Parameters', icon: BookOpen },
    { id: 'slide-2', title: 'The Problem', icon: HelpCircle },
    { id: 'slide-3', title: 'What are Parameters?', icon: Sliders },
    { id: 'slide-4', title: 'Declaring Parameters', icon: Code },
    { id: 'slide-5', title: 'Using Parameters', icon: Variable },
    { id: 'slide-6', title: 'Parameters at Runtime', icon: Terminal },
    { id: 'slide-7', title: 'Multiple Nodes Example', icon: Network },
    { id: 'slide-8', title: 'YAML Parameter Files', icon: FileJson },
    { id: 'slide-9', title: 'Loading YAML Files', icon: Play },
    { id: 'slide-10', title: 'CLI Tools', icon: Terminal },
    { id: 'slide-11', title: 'Dynamic Reconfiguration', icon: RotateCcw },
    { id: 'slide-12', title: 'Summary', icon: Layout },
    { id: 'slide-13', title: 'Next Lecture', icon: ArrowRight },
  ];

  const taskManagerWithParamsCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AssignTask, GetTaskStatus, CancelTask

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Declare parameters with default values
        self.declare_parameter('warehouse_x_max', 50.0)
        self.declare_parameter('warehouse_y_max', 30.0)
        self.declare_parameter('max_active_tasks', 1)
        
        # Get parameter values
        self.warehouse_x = (0.0, self.get_parameter('warehouse_x_max').value)
        self.warehouse_y = (0.0, self.get_parameter('warehouse_y_max').value)
        self.max_tasks = self.get_parameter('max_active_tasks').value
        
        # Task state
        self.current_task = None
        self.task_counter = 0
        
        # Create services (same as before)
        self.assign_service = self.create_service(
            AssignTask, 'assign_task', self.assign_task_callback
        )
        
        self.get_logger().info(
            f"Task Manager ready! Warehouse: {self.warehouse_x[1]}x{self.warehouse_y[1]}"
        )`;

  const batteryMonitorWithParamsCode = `class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Declare parameters
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('critical_battery_threshold', 10.0)
        
        # Get parameter values
        update_rate = self.get_parameter('update_rate').value
        self.low_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_threshold = self.get_parameter('critical_battery_threshold').value
        
        # Use parameter in timer
        self.timer = self.create_timer(update_rate, self.update_battery)
        
        self.get_logger().info(
            f"Battery Monitor started (rate: {update_rate}s, low: {self.low_threshold}%)"
        )`;

  const sensorFilterWithParamsCode = `class SensorFilter(Node):
    def __init__(self):
        super().__init__('sensor_filter')
        
        # Declare parameter
        self.declare_parameter('filter_window_size', 5)
        
        # Get parameter value
        self.window_size = self.get_parameter('filter_window_size').value
        
        # Initialize buffer with parameter size
        self.speed_buffer = []
        
        self.get_logger().info(f"Sensor Filter started (window: {self.window_size})")`;

  const yamlParamsCode = `# robot_params.yaml
/task_manager:
  ros__parameters:
    warehouse_x_max: 100.0
    warehouse_y_max: 60.0
    max_active_tasks: 1

/battery_monitor:
  ros__parameters:
    update_rate: 0.5
    low_battery_threshold: 25.0
    critical_battery_threshold: 15.0

/sensor_filter:
  ros__parameters:
    filter_window_size: 10`;

  const warehouseAParams = `# warehouse_a_params.yaml
/task_manager:
  ros__parameters:
    warehouse_x_max: 50.0
    warehouse_y_max: 30.0`;

  const warehouseBParams = `# warehouse_b_params.yaml
/task_manager:
  ros__parameters:
    warehouse_x_max: 100.0
    warehouse_y_max: 80.0`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={6} 
        lectureTitle="Parameters" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Title */}
        <LectureSlide id="slide-1" title="Parameters" subtitle="Making Nodes Configurable" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Lecture</h4>
                <p className="font-medium text-zinc-900">6 of 12</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Focus</h4>
                <p className="font-medium text-zinc-900">Runtime Configuration</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Goal</h4>
                <p className="font-medium text-zinc-900">Flexible, Reusable Nodes</p>
              </div>
            </div>
            <div className="mt-8 pt-8 border-t border-zinc-200">
              <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-4">Today&apos;s Agenda</h4>
              <div className="grid md:grid-cols-4 gap-4">
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Sliders className="inline mb-1" size={24} />
                  <div className="text-sm font-medium">Declare Params</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Terminal className="inline mb-1" size={24} />
                  <div className="text-sm font-medium">Runtime Config</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <FileJson className="inline mb-1" size={24} />
                  <div className="text-sm font-medium">YAML Files</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <RotateCcw className="inline mb-1" size={24} />
                  <div className="text-sm font-medium">Dynamic Update</div>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-8 p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <p className="text-blue-800">
              <strong>Note:</strong> This lecture accompanies a live demo. We&apos;ll cover the essentials here, then see it in action.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 2: The Problem */}
        <LectureSlide id="slide-2" title="The Problem" subtitle="Why We Need Parameters" icon={HelpCircle}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              Look at our task_manager from Lecture 5. The warehouse bounds are hardcoded:
            </p>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl mb-8 font-mono text-sm">
            <div className="text-zinc-500"># Hardcoded in task_manager.py</div>
            <div>self.warehouse_x = (0.0, <span className="text-red-400">50.0</span>)  <span className="text-zinc-500"># What if warehouse is bigger?</span></div>
            <div>self.warehouse_y = (0.0, <span className="text-red-400">30.0</span>)  <span className="text-zinc-500"># What if it&apos;s different shape?</span></div>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-8">
            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <h4 className="font-bold text-red-900 mb-4">Problems with Hardcoding</h4>
              <ul className="space-y-2 text-red-800">
                <li>Different warehouse? Edit code, rebuild.</li>
                <li>Testing with smaller area? Edit code, rebuild.</li>
                <li>Two robots, different warehouses? Duplicate code.</li>
                <li>Customer wants to adjust? Give them source code?</li>
              </ul>
            </div>

            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-4">What We Want</h4>
              <ul className="space-y-2 text-green-800">
                <li>Configure at startup, no code changes</li>
                <li>Same code, different configurations</li>
                <li>Store configs in files, easy to share</li>
                <li>Even change some values while running</li>
              </ul>
            </div>
          </div>

          <div className="p-4 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-lg">
              <span className="text-green-400">Parameters</span> let you configure nodes at runtime without changing code.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 3: What are Parameters? */}
        <LectureSlide id="slide-3" title="What are Parameters?" subtitle="Configuration Values for Nodes" icon={Sliders}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-2xl font-medium">
              A parameter is a <span className="text-blue-400">named value</span> that configures a node&apos;s behavior.
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 mb-8">
            <div className="p-6 bg-zinc-50 rounded-xl text-center">
              <div className="text-3xl mb-3">📝</div>
              <h4 className="font-bold text-zinc-900 mb-2">Has a Name</h4>
              <p className="text-zinc-600 text-sm">Like a variable: <code className="bg-white px-1 rounded">update_rate</code></p>
            </div>
            <div className="p-6 bg-zinc-50 rounded-xl text-center">
              <div className="text-3xl mb-3">🔢</div>
              <h4 className="font-bold text-zinc-900 mb-2">Has a Type</h4>
              <p className="text-zinc-600 text-sm">int, float, string, bool, arrays</p>
            </div>
            <div className="p-6 bg-zinc-50 rounded-xl text-center">
              <div className="text-3xl mb-3">🎯</div>
              <h4 className="font-bold text-zinc-900 mb-2">Per-Node</h4>
              <p className="text-zinc-600 text-sm">Each node has its own parameters</p>
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">Common Parameter Examples</h4>
            <div className="grid md:grid-cols-2 gap-4 text-sm">
              <div>
                <p className="font-medium text-blue-800 mb-2">For a camera driver:</p>
                <ul className="text-blue-700 space-y-1 font-mono">
                  <li>device_name: &quot;/dev/video0&quot;</li>
                  <li>fps: 30</li>
                  <li>resolution_width: 640</li>
                </ul>
              </div>
              <div>
                <p className="font-medium text-blue-800 mb-2">For our task_manager:</p>
                <ul className="text-blue-700 space-y-1 font-mono">
                  <li>warehouse_x_max: 50.0</li>
                  <li>warehouse_y_max: 30.0</li>
                  <li>max_active_tasks: 1</li>
                </ul>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 4: Declaring Parameters */}
        <LectureSlide id="slide-4" title="Declaring Parameters" subtitle="Two Steps: Declare, Then Get" icon={Code}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              To use a parameter, you must first <strong>declare</strong> it (make it exist), then <strong>get</strong> its value.
            </p>
          </div>

          <div className="space-y-6">
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">Step 1: Declare the Parameter</h4>
              <div className="bg-zinc-900 text-white p-4 rounded-lg font-mono text-sm">
                <div className="text-zinc-500"># In __init__, declare with name and default value</div>
                <div>self.declare_parameter(<span className="text-green-400">&apos;warehouse_x_max&apos;</span>, <span className="text-blue-400">50.0</span>)</div>
              </div>
              <p className="mt-3 text-zinc-600 text-sm">
                The default value (50.0) also sets the type (float). If no value is provided at runtime, this default is used.
              </p>
            </div>

            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">Step 2: Get the Value</h4>
              <div className="bg-zinc-900 text-white p-4 rounded-lg font-mono text-sm">
                <div className="text-zinc-500"># Retrieve the value to use in your code</div>
                <div>x_max = self.get_parameter(<span className="text-green-400">&apos;warehouse_x_max&apos;</span>).value</div>
              </div>
              <p className="mt-3 text-zinc-600 text-sm">
                Now <code className="bg-white px-1 rounded">x_max</code> contains either the default (50.0) or whatever was set at runtime.
              </p>
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">Important</h4>
            <p className="text-amber-800 text-sm">
              You must declare a parameter before getting it. Otherwise, you&apos;ll get a <code className="bg-white px-1 rounded">ParameterNotDeclaredException</code>.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 5: Using Parameters - Task Manager Example */}
        <LectureSlide id="slide-5" title="Using Parameters" subtitle="Updated task_manager" icon={Variable}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              Here&apos;s our task_manager with configurable warehouse bounds:
            </p>
          </div>

          <CodeBlock 
            filename="task_manager.py (with parameters)"
            code={taskManagerWithParamsCode}
          />

          <div className="mt-6 grid md:grid-cols-3 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg text-center">
              <div className="font-mono text-sm text-blue-800">warehouse_x_max</div>
              <div className="text-xs text-blue-600 mt-1">float, default: 50.0</div>
            </div>
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg text-center">
              <div className="font-mono text-sm text-blue-800">warehouse_y_max</div>
              <div className="text-xs text-blue-600 mt-1">float, default: 30.0</div>
            </div>
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg text-center">
              <div className="font-mono text-sm text-blue-800">max_active_tasks</div>
              <div className="text-xs text-blue-600 mt-1">int, default: 1</div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 6: Parameters at Runtime */}
        <LectureSlide id="slide-6" title="Parameters at Runtime" subtitle="Setting Values When You Launch" icon={Terminal}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Use <code className="bg-zinc-100 px-2 py-1 rounded">--ros-args -p</code> to set parameters when starting a node:
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Default values (50x30 warehouse)</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg task_manager"
                output={`[INFO] [task_manager]: Task Manager ready! Warehouse: 50.0x30.0`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Custom warehouse size (100x60)</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg task_manager --ros-args -p warehouse_x_max:=100.0 -p warehouse_y_max:=60.0"
                output={`[INFO] [task_manager]: Task Manager ready! Warehouse: 100.0x60.0`}
                title="Terminal"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <h4 className="font-bold text-green-900 mb-2">Same Code, Different Configs</h4>
            <p className="text-green-800 text-sm">
              No rebuilding, no code changes. Just different command-line arguments for different warehouses.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 7: Multiple Nodes Example */}
        <LectureSlide id="slide-7" title="Multiple Nodes Example" subtitle="Parameters Work Everywhere" icon={Network}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Parameters work the same way in any node. Here are examples from our other nodes:
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">battery_monitor - Configurable thresholds</h4>
              <CodeBlock 
                filename="battery_monitor.py"
                code={batteryMonitorWithParamsCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">sensor_filter - Configurable filter size</h4>
              <CodeBlock 
                filename="sensor_filter.py"
                code={sensorFilterWithParamsCode}
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-purple-50 border border-purple-200 rounded-lg">
            <h4 className="font-bold text-purple-900 mb-2">Universal Pattern</h4>
            <p className="text-purple-800 text-sm">
              <code className="bg-white px-1 rounded">declare_parameter()</code> + <code className="bg-white px-1 rounded">get_parameter().value</code> works in any node. Make everything configurable!
            </p>
          </div>
        </LectureSlide>

        {/* Slide 8: YAML Parameter Files */}
        <LectureSlide id="slide-8" title="YAML Parameter Files" subtitle="Store Configurations in Files" icon={FileJson}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              With many parameters, command-line gets unwieldy. Store them in YAML files instead:
            </p>
          </div>

          <CodeBlock 
            filename="config/robot_params.yaml"
            code={yamlParamsCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">Structure</h5>
              <ul className="text-blue-800 text-sm space-y-1">
                <li><code className="bg-white px-1 rounded">/node_name:</code> - Node to configure</li>
                <li><code className="bg-white px-1 rounded">ros__parameters:</code> - Required (two underscores!)</li>
                <li><code className="bg-white px-1 rounded">param: value</code> - Your parameters</li>
              </ul>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">Benefits</h5>
              <ul className="text-green-800 text-sm space-y-1">
                <li>Easy to read and edit</li>
                <li>Version control friendly</li>
                <li>Share configs across team</li>
                <li>Multiple nodes in one file</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 9: Loading YAML Files */}
        <LectureSlide id="slide-9" title="Loading YAML Files" subtitle="Different Configs for Different Scenarios" icon={Play}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Use <code className="bg-zinc-100 px-2 py-1 rounded">--params-file</code> to load parameters from YAML:
            </p>
          </div>

          <div className="space-y-4 mb-8">
            <TerminalBlock 
              command="ros2 run my_robot_pkg task_manager --ros-args --params-file config/robot_params.yaml"
              output={`[INFO] [task_manager]: Task Manager ready! Warehouse: 100.0x60.0`}
              title="Terminal"
            />
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Warehouse A (small)</h4>
              <CodeBlock 
                filename="warehouse_a_params.yaml"
                code={warehouseAParams}
              />
            </div>
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Warehouse B (large)</h4>
              <CodeBlock 
                filename="warehouse_b_params.yaml"
                code={warehouseBParams}
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-zinc-900 text-white rounded-xl text-center font-mono text-sm">
            <div className="text-zinc-400 mb-2"># Deploy to different warehouses with same code:</div>
            <div>ros2 run my_robot_pkg task_manager --ros-args --params-file <span className="text-green-400">warehouse_a_params.yaml</span></div>
            <div>ros2 run my_robot_pkg task_manager --ros-args --params-file <span className="text-blue-400">warehouse_b_params.yaml</span></div>
          </div>
        </LectureSlide>

        {/* Slide 10: CLI Tools */}
        <LectureSlide id="slide-10" title="CLI Tools for Parameters" subtitle="Inspect and Manage at Runtime" icon={Terminal}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              ROS 2 provides <code className="bg-zinc-100 px-2 py-1 rounded">ros2 param</code> commands to work with parameters:
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">List all parameters for a node</h4>
              <TerminalBlock 
                command="ros2 param list /task_manager"
                output={`max_active_tasks
use_sim_time
warehouse_x_max
warehouse_y_max`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Get a specific parameter&apos;s value</h4>
              <TerminalBlock 
                command="ros2 param get /task_manager warehouse_x_max"
                output={`Double value is: 100.0`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Describe a parameter (type, constraints)</h4>
              <TerminalBlock 
                command="ros2 param describe /task_manager warehouse_x_max"
                output={`Parameter name: warehouse_x_max
  Type: double
  Description: ...`}
                title="Terminal"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 11: Dynamic Reconfiguration */}
        <LectureSlide id="slide-11" title="Dynamic Reconfiguration" subtitle="Change Parameters While Running" icon={RotateCcw}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              You can change parameter values while the node is running using <code className="bg-zinc-100 px-2 py-1 rounded">ros2 param set</code>:
            </p>
          </div>

          <div className="p-4 bg-zinc-50 rounded-lg mb-6">
            <h4 className="font-bold text-zinc-900 mb-3">Change warehouse bounds on the fly</h4>
            <TerminalBlock 
              command="ros2 param set /task_manager warehouse_x_max 75.0"
              output={`Set parameter successful`}
              title="Terminal"
            />
          </div>

          <div className="p-6 bg-amber-50 border border-amber-200 rounded-xl mb-6">
            <h4 className="font-bold text-amber-900 mb-3">Important: When Does the Node See Changes?</h4>
            <p className="text-amber-800 mb-4">
              By default, the node reads parameters once at startup. For the node to react to runtime changes, you need <strong>parameter callbacks</strong> (advanced topic).
            </p>
            <div className="grid md:grid-cols-2 gap-4 text-sm">
              <div className="p-3 bg-white rounded-lg">
                <h5 className="font-bold text-amber-900">Without callback:</h5>
                <p className="text-amber-800">Node uses old value until restart</p>
              </div>
              <div className="p-3 bg-white rounded-lg">
                <h5 className="font-bold text-amber-900">With callback:</h5>
                <p className="text-amber-800">Node reacts immediately to changes</p>
              </div>
            </div>
          </div>

          <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <h4 className="font-bold text-blue-900 mb-2">Common Use Cases for Dynamic Params</h4>
            <ul className="text-blue-800 text-sm space-y-1">
              <li>Tuning thresholds during testing</li>
              <li>Adjusting speed limits based on conditions</li>
              <li>Enabling/disabling debug logging</li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 12: Summary */}
        <LectureSlide id="slide-12" title="Summary" subtitle="What You Learned" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 rounded-lg">
                <h4 className="font-bold text-blue-900 mb-2">Declaring Parameters</h4>
                <div className="font-mono text-xs text-blue-800">
                  self.declare_parameter(&apos;name&apos;, default)<br/>
                  value = self.get_parameter(&apos;name&apos;).value
                </div>
              </div>

              <div className="p-4 bg-green-50 rounded-lg">
                <h4 className="font-bold text-green-900 mb-2">Runtime Configuration</h4>
                <div className="font-mono text-xs text-green-800">
                  ros2 run pkg node --ros-args -p name:=value
                </div>
              </div>
            </div>

            <div className="space-y-4">
              <div className="p-4 bg-purple-50 rounded-lg">
                <h4 className="font-bold text-purple-900 mb-2">YAML Files</h4>
                <div className="font-mono text-xs text-purple-800">
                  --params-file config.yaml
                </div>
              </div>

              <div className="p-4 bg-amber-50 rounded-lg">
                <h4 className="font-bold text-amber-900 mb-2">CLI Tools</h4>
                <div className="font-mono text-xs text-amber-800">
                  ros2 param list / get / set
                </div>
              </div>
            </div>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-xl">
              Parameters make your nodes <span className="text-green-400">flexible</span> and <span className="text-blue-400">reusable</span> across different robots, environments, and use cases.
            </p>
          </div>

          <div className="mt-6 p-4 bg-zinc-50 rounded-lg">
            <h4 className="font-bold text-zinc-900 mb-2">Now: Live Demo</h4>
            <p className="text-zinc-600">
              Let&apos;s see parameters in action with a real system running!
            </p>
          </div>
        </LectureSlide>

        {/* Slide 13: Next Lecture */}
        <LectureSlide id="slide-13" title="Next Lecture" subtitle="Actions - Long-Running Tasks" icon={ArrowRight}>
          <div className="p-8 bg-gradient-to-br from-zinc-900 to-zinc-800 text-white rounded-2xl">
            <h3 className="text-2xl font-bold mb-6">Lecture 7: Actions</h3>
            
            <div className="grid md:grid-cols-2 gap-6 mb-8">
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">The Problem</h4>
                <p className="text-zinc-400 text-sm">
                  Topics stream data. Services get instant responses. But what about tasks that take 30 seconds? Like navigating to a destination?
                </p>
              </div>
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">The Solution: Actions</h4>
                <p className="text-zinc-400 text-sm">
                  Actions handle long-running tasks with progress feedback, cancellation support, and final results.
                </p>
              </div>
            </div>

            <div className="space-y-4">
              <h4 className="font-bold text-zinc-300">What You&apos;ll Learn:</h4>
              <div className="grid md:grid-cols-3 gap-4">
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Target className="text-blue-400 mb-2" size={20} />
                  <p className="text-sm">Goals - What to do</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Activity className="text-green-400 mb-2" size={20} />
                  <p className="text-sm">Feedback - Progress updates</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Check className="text-purple-400 mb-2" size={20} />
                  <p className="text-sm">Results - Final outcome</p>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-8 p-6 bg-blue-50 border border-blue-200 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-3">Real-World Example: Navigation</h4>
            <div className="flex items-center space-x-4 text-sm text-blue-800">
              <div className="p-2 bg-white rounded">Goal: Go to (25, 15)</div>
              <ArrowRight className="text-blue-400" size={20} />
              <div className="p-2 bg-white rounded">Feedback: 10m remaining...</div>
              <ArrowRight className="text-blue-400" size={20} />
              <div className="p-2 bg-white rounded">Result: Arrived!</div>
            </div>
          </div>

          <div className="mt-8 text-center text-zinc-500">
            <p className="font-medium text-zinc-700">See you in Lecture 7!</p>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
