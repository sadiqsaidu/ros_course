'use client';

import { 
  Terminal, Battery, Cpu, Server, Play, Check, ChevronRight, 
  BookOpen, Layout, Settings, Code, ArrowRight, AlertTriangle, 
  Network, Search, Award, Users, Wrench, Package, FolderTree,
  Lightbulb, Zap, Target, Layers, HelpCircle, Bot, Cog,
  FileCode, GitBranch, Box, Rocket, Radio, Send, Inbox,
  MessageSquare, RefreshCw, Eye, Activity, Filter, Gauge,
  CircuitBoard, Workflow, Combine, BarChart3, Sparkles,
  Sliders, FileJson, Variable, RotateCcw, XCircle, Timer,
  Route, MapPin, Navigation, CheckCircle, ArrowLeftRight,
  FolderOpen, Hammer, Building2
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture8() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Lecture 8: Launch Files', icon: BookOpen },
    { id: 'slide-2', title: 'The Problem', icon: HelpCircle },
    { id: 'slide-3', title: 'What is a Launch File?', icon: Rocket },
    { id: 'slide-4', title: 'Package Setup', icon: Package },
    { id: 'slide-5', title: 'First Launch File', icon: FileCode },
    { id: 'slide-6', title: 'Install and Run', icon: Play },
    { id: 'slide-7', title: 'Adding More Nodes', icon: Network },
    { id: 'slide-8', title: 'Parameters in Launch', icon: Sliders },
    { id: 'slide-9', title: 'YAML Param Files', icon: FileJson },
    { id: 'slide-10', title: 'Loading YAML in Launch', icon: FolderOpen },
    { id: 'slide-11', title: 'Renaming & Remapping', icon: ArrowLeftRight },
    { id: 'slide-12', title: 'Namespaces', icon: FolderTree },
    { id: 'slide-13', title: 'Full Warehouse Launch', icon: Building2 },
    { id: 'slide-14', title: 'Running the System', icon: Terminal },
    { id: 'slide-15', title: 'Python Launch Files', icon: Code },
    { id: 'slide-16', title: 'XML vs Python', icon: Layers },
    { id: 'slide-17', title: 'Summary', icon: Layout },
    { id: 'slide-18', title: 'Next Lecture', icon: ArrowRight },
  ];

  // ── Package creation ──
  const createPackageCode = `cd ~/ros2_ws/src
ros2 pkg create my_robot_bringup
cd my_robot_bringup

# Remove directories we don't need
rm -rf include src

# Create folders for launch files and config
mkdir launch config`;

  // ── CMakeLists setup ──
  const cmakeListsCode = `# my_robot_bringup/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/\${PROJECT_NAME}
)

# Install config (YAML) files
install(DIRECTORY
  config
  DESTINATION share/\${PROJECT_NAME}
)

ament_package()`;

  // ── Minimal launch file ──
  const minimalLaunchCode = `<!-- my_robot_bringup/launch/task_manager.launch.xml -->
<launch>
    <node pkg="my_robot_pkg" exec="task_manager" />
</launch>`;

  // ── Two nodes launch ──
  const twoNodesLaunchCode = `<!-- my_robot_bringup/launch/warehouse_core.launch.xml -->
<launch>
    <node pkg="my_robot_pkg" exec="task_manager" />
    <node pkg="my_robot_pkg" exec="battery_monitor" />
</launch>`;

  // ── Multi-node launch ──
  const multiNodeLaunchCode = `<!-- my_robot_bringup/launch/warehouse_app.launch.xml -->
<launch>
    <node pkg="my_robot_pkg" exec="task_manager" />
    <node pkg="my_robot_pkg" exec="navigation_server" />
    <node pkg="my_robot_pkg" exec="battery_monitor" />
    <node pkg="my_robot_pkg" exec="sensor_filter" />
</launch>`;

  // ── Inline params ──
  const inlineParamsCode = `<!-- Setting parameters directly in the launch file -->
<launch>
    <node pkg="my_robot_pkg" exec="task_manager">
        <param name="warehouse_x_max" value="100.0" />
        <param name="warehouse_y_max" value="60.0" />
        <param name="max_active_tasks" value="1" />
    </node>

    <node pkg="my_robot_pkg" exec="battery_monitor">
        <param name="update_rate" value="0.5" />
        <param name="low_battery_threshold" value="25.0" />
        <param name="critical_battery_threshold" value="15.0" />
    </node>
</launch>`;

  // ── YAML params file ──
  const yamlParamsCode = `# my_robot_bringup/config/warehouse_params.yaml
/task_manager:
  ros__parameters:
    warehouse_x_max: 100.0
    warehouse_y_max: 60.0
    max_active_tasks: 1

/navigation_server:
  ros__parameters:
    speed: 2.0

/battery_monitor:
  ros__parameters:
    update_rate: 0.5
    low_battery_threshold: 25.0
    critical_battery_threshold: 15.0

/sensor_filter:
  ros__parameters:
    filter_window_size: 10`;

  // ── Loading YAML in launch ──
  const yamlLaunchCode = `<!-- my_robot_bringup/launch/warehouse_app.launch.xml -->
<launch>
    <node pkg="my_robot_pkg" exec="task_manager">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>

    <node pkg="my_robot_pkg" exec="navigation_server">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>

    <node pkg="my_robot_pkg" exec="battery_monitor">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>

    <node pkg="my_robot_pkg" exec="sensor_filter">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>
</launch>`;

  // ── Renaming and remapping ──
  const renamingCode = `<!-- Renaming a node -->
<node pkg="my_robot_pkg" exec="task_manager" name="warehouse_a_manager" />

<!-- Remapping a topic -->
<node pkg="my_robot_pkg" exec="sensor_filter">
    <remap from="robot_speed" to="conveyor_speed" />
</node>`;

  // ── Namespaces ──
  const namespacesCode = `<!-- Adding namespaces to separate two robots -->
<launch>
    <!-- Robot 1 -->
    <node pkg="my_robot_pkg" exec="task_manager" namespace="robot1" />
    <node pkg="my_robot_pkg" exec="navigation_server" namespace="robot1" />

    <!-- Robot 2 -->
    <node pkg="my_robot_pkg" exec="task_manager" namespace="robot2" />
    <node pkg="my_robot_pkg" exec="navigation_server" namespace="robot2" />
</launch>`;

  // ── YAML with namespaces ──
  const yamlNamespacesCode = `# IMPORTANT: YAML must match namespace + node name
/robot1/task_manager:
  ros__parameters:
    warehouse_x_max: 50.0

/robot2/task_manager:
  ros__parameters:
    warehouse_x_max: 100.0`;

  // ── Full warehouse launch ──
  const fullWarehouseLaunchCode = `<!-- my_robot_bringup/launch/warehouse_full.launch.xml -->
<launch>
    <!-- Task Manager: accepts and tracks delivery tasks -->
    <node pkg="my_robot_pkg" exec="task_manager">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>

    <!-- Navigation Server: handles navigate_to_position actions -->
    <node pkg="my_robot_pkg" exec="navigation_server">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>

    <!-- Battery Monitor: publishes battery level on topic -->
    <node pkg="my_robot_pkg" exec="battery_monitor">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>

    <!-- Sensor Filter: filters speed readings -->
    <node pkg="my_robot_pkg" exec="sensor_filter">
        <param from="$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml" />
    </node>
</launch>`;

  // ── package.xml deps ──
  const packageXmlCode = `<!-- my_robot_bringup/package.xml (add these dependencies) -->
<exec_depend>my_robot_pkg</exec_depend>
<exec_depend>my_robot_interfaces</exec_depend>`;

  // ── Python launch file ──
  const pythonLaunchCode = `# my_robot_bringup/launch/warehouse_app.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='task_manager',
            parameters=[{
                'warehouse_x_max': 100.0,
                'warehouse_y_max': 60.0,
            }]
        ),
        Node(
            package='my_robot_pkg',
            executable='battery_monitor',
            parameters=[{
                'update_rate': 0.5,
                'low_battery_threshold': 25.0,
            }]
        ),
    ])`;

  // ── Python launch with YAML ──
  const pythonYamlLaunchCode = `# Loading a YAML file in a Python launch file
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'warehouse_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='task_manager',
            parameters=[config]
        ),
    ])`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={8} 
        lectureTitle="Launch Files" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Title */}
        <LectureSlide id="slide-1" title="Launch Files" subtitle="Starting All Your Nodes at Once" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Lecture</h4>
                <p className="font-medium text-zinc-900">8 of 12</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Focus</h4>
                <p className="font-medium text-zinc-900">System Composition</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Goal</h4>
                <p className="font-medium text-zinc-900">One Command, Full System</p>
              </div>
            </div>
            <div className="mt-8 pt-8 border-t border-zinc-200">
              <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-4">Today&apos;s Agenda</h4>
              <div className="grid md:grid-cols-4 gap-3">
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Package className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Bringup Package</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <FileCode className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">XML Launch Files</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <Sliders className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Params &amp; YAML</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <FolderTree className="inline mb-1" size={20} />
                  <div className="text-xs font-medium">Namespaces</div>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-8 p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <p className="text-blue-800">
              <strong>Note:</strong> Each code example will be followed by a live demo so you can see the launch files in action.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 2: The Problem */}
        <LectureSlide id="slide-2" title="The Problem" subtitle="Too Many Terminals" icon={HelpCircle}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              Look at everything we&apos;ve built so far. To run our warehouse system, we need to open <strong>four separate terminals</strong>:
            </p>
          </div>

          <div className="space-y-3 mb-8">
            <div className="p-3 bg-zinc-900 text-white rounded-lg font-mono text-sm">
              <span className="text-zinc-500">Terminal 1:</span> ros2 run my_robot_pkg task_manager --ros-args -p warehouse_x_max:=100.0 -p warehouse_y_max:=60.0
            </div>
            <div className="p-3 bg-zinc-900 text-white rounded-lg font-mono text-sm">
              <span className="text-zinc-500">Terminal 2:</span> ros2 run my_robot_pkg navigation_server
            </div>
            <div className="p-3 bg-zinc-900 text-white rounded-lg font-mono text-sm">
              <span className="text-zinc-500">Terminal 3:</span> ros2 run my_robot_pkg battery_monitor --ros-args -p update_rate:=0.5
            </div>
            <div className="p-3 bg-zinc-900 text-white rounded-lg font-mono text-sm">
              <span className="text-zinc-500">Terminal 4:</span> ros2 run my_robot_pkg sensor_filter --ros-args -p filter_window_size:=10
            </div>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <h4 className="font-bold text-red-900 mb-3">Problems</h4>
              <ul className="space-y-2 text-red-800 text-sm">
                <li>4 terminals now, real robots often need 15+</li>
                <li>Easy to forget a node or mistype a parameter</li>
                <li>Every time you restart, repeat all commands</li>
                <li>Can&apos;t share your setup with teammates easily</li>
              </ul>
            </div>
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-3">What We Want</h4>
              <ul className="space-y-2 text-green-800 text-sm">
                <li>One command to start everything</li>
                <li>All parameters in one config file</li>
                <li>Reproducible across machines</li>
                <li>Version-controlled and shareable</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 3: What is a Launch File? */}
        <LectureSlide id="slide-3" title="What is a Launch File?" subtitle="One File, Entire System" icon={Rocket}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-2xl font-medium">
              A launch file is a <span className="text-blue-400">description</span> of which nodes to start and how to configure them.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-8">
            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <h4 className="font-bold text-red-900 mb-3">Without launch file</h4>
              <div className="space-y-2 text-sm font-mono text-red-800">
                <div>$ ros2 run ... task_manager --ros-args ...</div>
                <div>$ ros2 run ... navigation_server</div>
                <div>$ ros2 run ... battery_monitor --ros-args ...</div>
                <div>$ ros2 run ... sensor_filter --ros-args ...</div>
              </div>
              <p className="mt-3 text-red-700 text-xs">4 terminals, easy to make mistakes</p>
            </div>
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-3">With launch file</h4>
              <div className="font-mono text-sm text-green-800">
                $ ros2 launch my_robot_bringup warehouse_app.launch.xml
              </div>
              <p className="mt-3 text-green-700 text-xs">1 terminal, 1 command, all nodes start with correct parameters</p>
            </div>
          </div>

          <div className="p-4 bg-zinc-50 rounded-lg">
            <h5 className="font-bold text-zinc-900 mb-2">Launch files can be written in:</h5>
            <div className="grid md:grid-cols-3 gap-3">
              <div className="p-3 bg-blue-50 border border-blue-200 rounded-lg text-center">
                <div className="font-bold text-blue-900">XML</div>
                <div className="text-blue-700 text-xs">.launch.xml — simplest, recommended</div>
              </div>
              <div className="p-3 bg-green-50 border border-green-200 rounded-lg text-center">
                <div className="font-bold text-green-900">Python</div>
                <div className="text-green-700 text-xs">.launch.py — more verbose, advanced cases</div>
              </div>
              <div className="p-3 bg-zinc-100 rounded-lg text-center">
                <div className="font-bold text-zinc-500">YAML</div>
                <div className="text-zinc-400 text-xs">.launch.yaml — rarely used</div>
              </div>
            </div>
            <p className="text-zinc-600 text-sm mt-3">
              We&apos;ll use <strong>XML</strong> as our default. It&apos;s shorter, cleaner, and does the job for most use cases.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 4: Package Setup */}
        <LectureSlide id="slide-4" title="Package Setup" subtitle="Creating a Bringup Package" icon={Package}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Best practice: create a <strong>dedicated package</strong> for launch files. Don&apos;t put them in your node packages — that creates dependency loops.
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">1. Create the package</h4>
              <CodeBlock 
                filename="Terminal"
                code={createPackageCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">2. Configure CMakeLists.txt</h4>
              <CodeBlock 
                filename="my_robot_bringup/CMakeLists.txt"
                code={cmakeListsCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">3. Add dependencies to package.xml</h4>
              <CodeBlock 
                filename="my_robot_bringup/package.xml"
                code={packageXmlCode}
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h5 className="font-bold text-amber-900 mb-2">Naming Convention</h5>
            <p className="text-amber-800 text-sm">
              The package is named <code className="bg-white px-1 rounded">&lt;robot_name&gt;_bringup</code>. This <code className="bg-white px-1 rounded">_bringup</code> suffix is a standard ROS convention. You&apos;ll see it in most ROS 2 projects.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 5: First Launch File */}
        <LectureSlide id="slide-5" title="First Launch File" subtitle="Starting a Single Node" icon={FileCode}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Create a file in the <code className="bg-zinc-100 px-2 py-1 rounded">launch</code> folder with the <code className="bg-zinc-100 px-2 py-1 rounded">.launch.xml</code> extension:
            </p>
          </div>

          <CodeBlock 
            filename="launch/task_manager.launch.xml"
            code={minimalLaunchCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">Structure</h5>
              <ul className="text-blue-800 text-sm space-y-1">
                <li><code className="bg-white px-1 rounded">&lt;launch&gt;</code> — wraps everything</li>
                <li><code className="bg-white px-1 rounded">&lt;node&gt;</code> — starts one node</li>
                <li><code className="bg-white px-1 rounded">pkg</code> — the package name</li>
                <li><code className="bg-white px-1 rounded">exec</code> — the executable name</li>
              </ul>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">Same as running:</h5>
              <div className="font-mono text-xs text-green-800 bg-white p-2 rounded">
                ros2 run my_robot_pkg task_manager
              </div>
              <p className="text-green-700 text-xs mt-2">But now it&apos;s in a file, reproducible and shareable.</p>
            </div>
          </div>

          <div className="mt-6 p-6 bg-zinc-50 rounded-xl">
            <h5 className="font-bold text-zinc-900 mb-3">Adding a second node? Just add another line:</h5>
            <CodeBlock 
              filename="launch/warehouse_core.launch.xml"
              code={twoNodesLaunchCode}
            />
          </div>
        </LectureSlide>

        {/* Slide 6: Install and Run */}
        <LectureSlide id="slide-6" title="Install and Run" subtitle="Build, Source, Launch" icon={Play}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Three steps to go from launch file to running system:
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">1. Build the bringup package</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws && colcon build --packages-select my_robot_bringup"
                output={`Starting >>> my_robot_bringup
Finished <<< my_robot_bringup [0.8s]

Summary: 1 package finished`}
                title="Terminal"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">2. Source your environment</h4>
              <TerminalBlock 
                command="source install/setup.bash"
                output=""
                title="Terminal"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">3. Launch!</h4>
              <TerminalBlock 
                command="ros2 launch my_robot_bringup warehouse_core.launch.xml"
                output={`[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [task_manager-1]: process started with pid [12345]
[INFO] [battery_monitor-2]: process started with pid [12346]
[task_manager-1] [INFO] [task_manager]: Task Manager ready! Warehouse: 50.0x30.0
[battery_monitor-2] [INFO] [battery_monitor]: Battery Monitor started (rate: 1.0s, low: 20.0%)`}
                title="Terminal"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <p className="text-green-800 text-sm">
              <strong>All logs appear in one terminal.</strong> Each line is prefixed with the node name (e.g., <code className="bg-white px-1 rounded">[task_manager-1]</code>) so you can tell them apart. Press <code className="bg-white px-1 rounded">Ctrl+C</code> to stop all nodes at once.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 7: Adding More Nodes */}
        <LectureSlide id="slide-7" title="Adding More Nodes" subtitle="Building Up the System" icon={Network}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Adding nodes is just one line each. Here&apos;s our full warehouse system — all four nodes from Lectures 2-7:
            </p>
          </div>

          <CodeBlock 
            filename="launch/warehouse_app.launch.xml"
            code={multiNodeLaunchCode}
          />

          <div className="mt-6 p-6 bg-zinc-900 text-white rounded-xl">
            <h5 className="font-bold mb-4">Our warehouse system at a glance:</h5>
            <div className="grid md:grid-cols-4 gap-3">
              <div className="p-3 bg-zinc-800 rounded-lg text-center">
                <Cpu className="inline text-blue-400 mb-1" size={20} />
                <div className="text-sm font-medium">task_manager</div>
                <div className="text-xs text-zinc-400">Services (L4-5)</div>
              </div>
              <div className="p-3 bg-zinc-800 rounded-lg text-center">
                <Navigation className="inline text-green-400 mb-1" size={20} />
                <div className="text-sm font-medium">navigation_server</div>
                <div className="text-xs text-zinc-400">Actions (L7)</div>
              </div>
              <div className="p-3 bg-zinc-800 rounded-lg text-center">
                <Battery className="inline text-amber-400 mb-1" size={20} />
                <div className="text-sm font-medium">battery_monitor</div>
                <div className="text-xs text-zinc-400">Topics (L2-3)</div>
              </div>
              <div className="p-3 bg-zinc-800 rounded-lg text-center">
                <Filter className="inline text-purple-400 mb-1" size={20} />
                <div className="text-sm font-medium">sensor_filter</div>
                <div className="text-xs text-zinc-400">Topics (L3)</div>
              </div>
            </div>
          </div>

          <div className="mt-6 p-4 bg-zinc-50 rounded-lg">
            <p className="text-zinc-700 text-sm">
              <strong>But wait</strong> — all these nodes are using default parameter values. What if we want warehouse bounds of 100x60 instead of 50x30? Let&apos;s add parameters.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 8: Parameters in Launch */}
        <LectureSlide id="slide-8" title="Parameters in Launch Files" subtitle="Setting Values Inline" icon={Sliders}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Remember setting parameters from the command line with <code className="bg-zinc-100 px-2 py-1 rounded">--ros-args -p</code>? In a launch file, use <code className="bg-zinc-100 px-2 py-1 rounded">&lt;param&gt;</code> tags inside a <code className="bg-zinc-100 px-2 py-1 rounded">&lt;node&gt;</code>:
            </p>
          </div>

          <CodeBlock 
            filename="launch/warehouse_app.launch.xml"
            code={inlineParamsCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">Syntax</h5>
              <div className="font-mono text-xs text-blue-800 bg-white p-2 rounded">
                &lt;param name=&quot;param_name&quot; value=&quot;param_value&quot; /&gt;
              </div>
              <p className="text-blue-700 text-xs mt-2">Each parameter is one line inside the node tag.</p>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">Same as running:</h5>
              <div className="font-mono text-xs text-green-800 bg-white p-2 rounded">
                ros2 run ... --ros-args -p warehouse_x_max:=100.0
              </div>
              <p className="text-green-700 text-xs mt-2">But now it&apos;s saved in the launch file.</p>
            </div>
          </div>

          <div className="mt-4 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <p className="text-amber-800 text-sm">
              <strong>Tip:</strong> Inline params work well for a few parameters. When you have many nodes with many parameters, use a YAML file instead — that&apos;s what we covered in Lecture 6.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 9: YAML Param Files */}
        <LectureSlide id="slide-9" title="YAML Parameter Files" subtitle="All Config in One Place" icon={FileJson}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              For real projects, put all parameters in a YAML file inside the <code className="bg-zinc-100 px-2 py-1 rounded">config</code> folder of your bringup package:
            </p>
          </div>

          <CodeBlock 
            filename="config/warehouse_params.yaml"
            code={yamlParamsCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
              <h5 className="font-bold text-blue-900 mb-2">Same YAML format from Lecture 6</h5>
              <ul className="text-blue-800 text-sm space-y-1">
                <li><code className="bg-white px-1 rounded">/node_name:</code> — matches the node</li>
                <li><code className="bg-white px-1 rounded">ros__parameters:</code> — required (two underscores)</li>
                <li>All params for all nodes in one file</li>
              </ul>
            </div>
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">Why YAML in bringup?</h5>
              <ul className="text-green-800 text-sm space-y-1">
                <li>Separate config from code</li>
                <li>Easy to create variants (warehouse A, B, C)</li>
                <li>Installed with <code className="bg-white px-1 rounded">colcon build</code></li>
                <li>Version-controlled with git</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 10: Loading YAML in Launch */}
        <LectureSlide id="slide-10" title="Loading YAML in Launch" subtitle="Connecting Config to Nodes" icon={FolderOpen}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Use <code className="bg-zinc-100 px-2 py-1 rounded">&lt;param from=&quot;...&quot; /&gt;</code> to load the YAML file for each node:
            </p>
          </div>

          <CodeBlock 
            filename="launch/warehouse_app.launch.xml"
            code={yamlLaunchCode}
          />

          <div className="mt-6 p-6 bg-zinc-50 rounded-xl">
            <h5 className="font-bold text-zinc-900 mb-3">What does <code className="bg-white px-1 rounded">$(find-pkg-share ...)</code> do?</h5>
            <div className="bg-zinc-900 text-white p-4 rounded-lg font-mono text-sm">
              <div className="text-zinc-500"># It resolves to the installed package path:</div>
              <div>$(find-pkg-share my_robot_bringup)/config/warehouse_params.yaml</div>
              <div className="text-zinc-500 mt-1"># → ~/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/config/warehouse_params.yaml</div>
            </div>
            <p className="text-zinc-600 text-sm mt-3">
              This ensures the path works no matter where the workspace is installed. Each node reads only the parameters that match its name from the YAML file.
            </p>
          </div>

          <div className="mt-4 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <p className="text-amber-800 text-sm">
              <strong>Remember:</strong> The node names in the YAML file must match exactly. If you rename a node or add a namespace, the YAML must be updated too.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 11: Renaming & Remapping */}
        <LectureSlide id="slide-11" title="Renaming & Remapping" subtitle="Customizing Node and Topic Names" icon={ArrowLeftRight}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              You can rename nodes and remap topics/services/actions directly in the launch file:
            </p>
          </div>

          <CodeBlock 
            filename="Renaming and remapping"
            code={renamingCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-6">
            <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-3">Renaming (name=)</h4>
              <p className="text-blue-800 text-sm mb-3">Changes the node&apos;s identity on the ROS graph.</p>
              <div className="bg-white p-3 rounded text-xs font-mono text-blue-800">
                <div><span className="text-zinc-400">Before:</span> /task_manager</div>
                <div><span className="text-zinc-400">After:</span> /warehouse_a_manager</div>
              </div>
              <p className="text-blue-700 text-xs mt-2">Useful when running multiple instances of the same node.</p>
            </div>

            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-3">Remapping (remap)</h4>
              <p className="text-green-800 text-sm mb-3">Redirects a topic/service/action to a different name.</p>
              <div className="bg-white p-3 rounded text-xs font-mono text-green-800">
                <div><span className="text-zinc-400">Before:</span> /robot_speed</div>
                <div><span className="text-zinc-400">After:</span> /conveyor_speed</div>
              </div>
              <p className="text-green-700 text-xs mt-2">Connect nodes that use different topic names without changing code.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 12: Namespaces */}
        <LectureSlide id="slide-12" title="Namespaces" subtitle="Grouping Nodes by Robot or Zone" icon={FolderTree}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Namespaces prefix a node and all its topics/services/actions. This lets you run the <strong>same nodes</strong> for multiple robots without name conflicts.
            </p>
          </div>

          <CodeBlock 
            filename="launch/multi_robot.launch.xml"
            code={namespacesCode}
          />

          <div className="mt-6 p-6 bg-zinc-900 text-white rounded-xl">
            <h5 className="font-bold mb-4">What namespaces create:</h5>
            <div className="grid md:grid-cols-2 gap-4">
              <div className="p-3 bg-zinc-800 rounded-lg">
                <h6 className="font-bold text-blue-400 mb-2">namespace=&quot;robot1&quot;</h6>
                <div className="font-mono text-xs text-zinc-300 space-y-1">
                  <div>/robot1/task_manager</div>
                  <div>/robot1/navigation_server</div>
                  <div>/robot1/assign_task</div>
                  <div>/robot1/navigate_to_position</div>
                </div>
              </div>
              <div className="p-3 bg-zinc-800 rounded-lg">
                <h6 className="font-bold text-green-400 mb-2">namespace=&quot;robot2&quot;</h6>
                <div className="font-mono text-xs text-zinc-300 space-y-1">
                  <div>/robot2/task_manager</div>
                  <div>/robot2/navigation_server</div>
                  <div>/robot2/assign_task</div>
                  <div>/robot2/navigate_to_position</div>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h5 className="font-bold text-amber-900 mb-2">YAML must match namespaces!</h5>
            <CodeBlock 
              filename="YAML with namespaces"
              code={yamlNamespacesCode}
            />
            <p className="text-amber-800 text-xs mt-2">
              If your YAML says <code className="bg-white px-1 rounded">/task_manager</code> but the node is actually <code className="bg-white px-1 rounded">/robot1/task_manager</code>, the parameters won&apos;t load.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 13: Full Warehouse Launch */}
        <LectureSlide id="slide-13" title="Full Warehouse Launch File" subtitle="Everything Together" icon={Building2}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              Here&apos;s our complete warehouse system — all nodes with YAML parameters:
            </p>
          </div>

          <CodeBlock 
            filename="launch/warehouse_full.launch.xml"
            code={fullWarehouseLaunchCode}
          />

          <div className="mt-6 p-6 bg-zinc-50 rounded-xl">
            <h5 className="font-bold text-zinc-900 mb-3">Package structure:</h5>
            <div className="bg-zinc-900 text-white p-4 rounded-lg font-mono text-sm">
              <div>my_robot_bringup/</div>
              <div>├── config/</div>
              <div>│   └── warehouse_params.yaml</div>
              <div>├── launch/</div>
              <div>│   ├── task_manager.launch.xml</div>
              <div>│   ├── warehouse_core.launch.xml</div>
              <div>│   └── warehouse_full.launch.xml</div>
              <div>├── CMakeLists.txt</div>
              <div>└── package.xml</div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 14: Running the System */}
        <LectureSlide id="slide-14" title="Running the Full System" subtitle="One Command, Four Nodes" icon={Terminal}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Build, source, and launch — your entire warehouse is running:
            </p>
          </div>

          <TerminalBlock 
            command="ros2 launch my_robot_bringup warehouse_full.launch.xml"
            output={`[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [task_manager-1]: process started with pid [45001]
[INFO] [navigation_server-2]: process started with pid [45002]
[INFO] [battery_monitor-3]: process started with pid [45003]
[INFO] [sensor_filter-4]: process started with pid [45004]
[task_manager-1] [INFO] [task_manager]: Task Manager ready! Warehouse: 100.0x60.0
[navigation_server-2] [INFO] [navigation_server]: Navigation Server ready at (0.0, 0.0)
[battery_monitor-3] [INFO] [battery_monitor]: Battery Monitor started (rate: 0.5s, low: 25.0%)
[sensor_filter-4] [INFO] [sensor_filter]: Sensor Filter started (window: 10)`}
            title="Terminal"
          />

          <div className="mt-6 grid md:grid-cols-3 gap-4">
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg text-center">
              <CheckCircle className="inline text-green-600 mb-2" size={24} />
              <div className="text-sm font-bold text-green-900">4 nodes started</div>
              <p className="text-green-700 text-xs">All in one terminal</p>
            </div>
            <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg text-center">
              <Sliders className="inline text-blue-600 mb-2" size={24} />
              <div className="text-sm font-bold text-blue-900">All params loaded</div>
              <p className="text-blue-700 text-xs">From YAML config file</p>
            </div>
            <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg text-center">
              <Terminal className="inline text-purple-600 mb-2" size={24} />
              <div className="text-sm font-bold text-purple-900">Ctrl+C stops all</div>
              <p className="text-purple-700 text-xs">Clean shutdown</p>
            </div>
          </div>

          <div className="mt-6 p-4 bg-zinc-50 rounded-lg">
            <h5 className="font-bold text-zinc-900 mb-2">Verify with ros2 tools:</h5>
            <div className="grid md:grid-cols-2 gap-3">
              <div className="bg-zinc-900 text-white p-3 rounded font-mono text-xs">
                <div className="text-zinc-500">$ ros2 node list</div>
                <div>/task_manager</div>
                <div>/navigation_server</div>
                <div>/battery_monitor</div>
                <div>/sensor_filter</div>
              </div>
              <div className="bg-zinc-900 text-white p-3 rounded font-mono text-xs">
                <div className="text-zinc-500">$ ros2 action list</div>
                <div>/navigate_to_position</div>
                <div className="text-zinc-500 mt-1">$ ros2 service list | head -3</div>
                <div>/assign_task</div>
                <div>/get_task_status</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 15: Python Launch Files */}
        <LectureSlide id="slide-15" title="Python Launch Files" subtitle="When You Need More Logic" icon={Code}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              You can also write launch files in Python. The functionality is the same, but the syntax is more verbose.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Python with inline params</h4>
              <CodeBlock 
                filename="warehouse_app.launch.py"
                code={pythonLaunchCode}
              />
            </div>
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Python with YAML file</h4>
              <CodeBlock 
                filename="warehouse_app.launch.py"
                code={pythonYamlLaunchCode}
              />
            </div>
          </div>

          <div className="p-4 bg-zinc-50 rounded-lg">
            <h5 className="font-bold text-zinc-900 mb-2">Key Differences</h5>
            <ul className="text-zinc-700 text-sm space-y-1">
              <li>• Must have a <code className="bg-white px-1 rounded">generate_launch_description()</code> function</li>
              <li>• Uses <code className="bg-white px-1 rounded">Node()</code> objects instead of <code className="bg-white px-1 rounded">&lt;node&gt;</code> tags</li>
              <li>• Parameters are dictionaries in a list</li>
              <li>• Loading YAML requires extra imports and <code className="bg-white px-1 rounded">os.path.join()</code></li>
              <li>• File extension is <code className="bg-white px-1 rounded">.launch.py</code></li>
            </ul>
          </div>

          <div className="mt-4 p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <p className="text-blue-800 text-sm">
              <strong>Good news:</strong> You can include a Python launch file inside an XML one (and vice versa). So use XML by default and Python only when you need programming logic.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 16: XML vs Python */}
        <LectureSlide id="slide-16" title="XML vs Python" subtitle="Which Should You Use?" icon={Layers}>
          <div className="mb-8">
            <p className="text-lg text-zinc-700">
              Both produce the exact same result. The difference is readability and verbosity.
            </p>
          </div>

          <div className="overflow-x-auto mb-8">
            <table className="w-full text-sm">
              <thead>
                <tr className="bg-zinc-900 text-white">
                  <th className="p-3 text-left rounded-tl-lg">Criteria</th>
                  <th className="p-3 text-left">XML</th>
                  <th className="p-3 text-left rounded-tr-lg">Python</th>
                </tr>
              </thead>
              <tbody>
                <tr className="border-b border-zinc-100">
                  <td className="p-3 font-medium">Lines for 2 nodes</td>
                  <td className="p-3 text-green-700 font-bold">4 lines</td>
                  <td className="p-3 text-red-700">~20 lines</td>
                </tr>
                <tr className="border-b border-zinc-100 bg-zinc-50">
                  <td className="p-3 font-medium">Readability</td>
                  <td className="p-3 text-green-700 font-bold">Simple, declarative</td>
                  <td className="p-3">More complex</td>
                </tr>
                <tr className="border-b border-zinc-100">
                  <td className="p-3 font-medium">Programming logic</td>
                  <td className="p-3 text-red-700">Limited</td>
                  <td className="p-3 text-green-700 font-bold">Full Python logic</td>
                </tr>
                <tr className="border-b border-zinc-100 bg-zinc-50">
                  <td className="p-3 font-medium">Conditionals / loops</td>
                  <td className="p-3 text-red-700">Not available</td>
                  <td className="p-3 text-green-700 font-bold">Yes</td>
                </tr>
                <tr>
                  <td className="p-3 font-medium">Recommendation</td>
                  <td className="p-3 text-green-700 font-bold">Default choice</td>
                  <td className="p-3">Only when needed</td>
                </tr>
              </tbody>
            </table>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-lg">
              Use <span className="text-green-400">XML</span> for most launch files. Use <span className="text-blue-400">Python</span> only when you need conditional logic or dynamic configuration.
            </p>
            <p className="text-zinc-400 text-sm mt-2">
              You can always include a Python launch file inside an XML one if you need a specific Python feature.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 17: Summary */}
        <LectureSlide id="slide-17" title="Summary" subtitle="What You Learned" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-6 mb-8">
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 rounded-lg">
                <h4 className="font-bold text-blue-900 mb-2">Bringup Package</h4>
                <div className="font-mono text-xs text-blue-800">
                  ros2 pkg create my_robot_bringup<br />
                  launch/ folder + config/ folder
                </div>
              </div>

              <div className="p-4 bg-green-50 rounded-lg">
                <h4 className="font-bold text-green-900 mb-2">XML Launch File</h4>
                <div className="font-mono text-xs text-green-800">
                  &lt;launch&gt;<br />
                  &nbsp;&nbsp;&lt;node pkg=&quot;...&quot; exec=&quot;...&quot; /&gt;<br />
                  &lt;/launch&gt;
                </div>
              </div>
            </div>

            <div className="space-y-4">
              <div className="p-4 bg-purple-50 rounded-lg">
                <h4 className="font-bold text-purple-900 mb-2">Parameters</h4>
                <div className="font-mono text-xs text-purple-800">
                  Inline: &lt;param name=&quot;...&quot; value=&quot;...&quot; /&gt;<br />
                  YAML: &lt;param from=&quot;path.yaml&quot; /&gt;
                </div>
              </div>

              <div className="p-4 bg-amber-50 rounded-lg">
                <h4 className="font-bold text-amber-900 mb-2">Customization</h4>
                <div className="font-mono text-xs text-amber-800">
                  name=&quot;...&quot; — rename node<br />
                  namespace=&quot;...&quot; — group nodes<br />
                  &lt;remap from=&quot;...&quot; to=&quot;...&quot; /&gt;
                </div>
              </div>
            </div>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl">
            <h5 className="font-bold mb-4">Course Progress: Core ROS 2 Complete!</h5>
            <div className="grid md:grid-cols-5 gap-3 text-center">
              <div className="p-2 bg-zinc-800 rounded-lg">
                <div className="text-blue-400 font-bold text-xs">L2-3</div>
                <div className="text-xs text-zinc-400">Topics</div>
              </div>
              <div className="p-2 bg-zinc-800 rounded-lg">
                <div className="text-green-400 font-bold text-xs">L4-5</div>
                <div className="text-xs text-zinc-400">Services</div>
              </div>
              <div className="p-2 bg-zinc-800 rounded-lg">
                <div className="text-purple-400 font-bold text-xs">L6</div>
                <div className="text-xs text-zinc-400">Parameters</div>
              </div>
              <div className="p-2 bg-zinc-800 rounded-lg">
                <div className="text-amber-400 font-bold text-xs">L7</div>
                <div className="text-xs text-zinc-400">Actions</div>
              </div>
              <div className="p-2 bg-blue-600 rounded-lg">
                <div className="text-white font-bold text-xs">L8</div>
                <div className="text-xs text-blue-200">Launch Files</div>
              </div>
            </div>
            <p className="text-zinc-400 text-sm mt-4 text-center">
              You now know all the core building blocks of ROS 2. Next: tools and visualization.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 18: Next Lecture */}
        <LectureSlide id="slide-18" title="Next Lecture" subtitle="Tools & Debugging" icon={ArrowRight}>
          <div className="p-8 bg-gradient-to-br from-zinc-900 to-zinc-800 text-white rounded-2xl">
            <h3 className="text-2xl font-bold mb-6">Coming Up Next</h3>
            
            <div className="grid md:grid-cols-2 gap-6 mb-8">
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">Where We Are</h4>
                <p className="text-zinc-400 text-sm">
                  We&apos;ve covered all core ROS 2 communication types: topics, services, actions, parameters, and launch files. You can build and run full robot systems.
                </p>
              </div>
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">What&apos;s Next</h4>
                <p className="text-zinc-400 text-sm">
                  We&apos;ll explore ROS 2 tools for visualization, debugging, and understanding your system: rqt_graph, TFs, URDF, and more.
                </p>
              </div>
            </div>

            <div className="space-y-4">
              <h4 className="font-bold text-zinc-300">Upcoming Topics:</h4>
              <div className="grid md:grid-cols-3 gap-4">
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Eye className="text-blue-400 mb-2" size={20} />
                  <p className="text-sm">Visualization Tools</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Search className="text-green-400 mb-2" size={20} />
                  <p className="text-sm">Debugging &amp; Introspection</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Bot className="text-purple-400 mb-2" size={20} />
                  <p className="text-sm">Robot Description</p>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-8 text-center text-zinc-500">
            <p className="font-medium text-zinc-700">See you in the next lecture!</p>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
