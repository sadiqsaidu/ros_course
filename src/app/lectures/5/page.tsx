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
  ArrowLeftRight, CheckCircle, XCircle, FolderOpen, Hammer,
  ShieldCheck, ClipboardList, Truck, RotateCcw, List,
  CheckSquare, AlertCircle, Building2, Route, Timer
} from 'lucide-react';
import { LectureSidebar, LectureSlide, CodeBlock, TerminalBlock, SlideInfo } from '@/components';

export default function Lecture5() {
  const slides: SlideInfo[] = [
    { id: 'slide-1', title: 'Lecture 5: Task Management', icon: BookOpen },
    { id: 'slide-2', title: 'Homework Review', icon: Check },
    { id: 'slide-3', title: 'Review: What Services Do', icon: RefreshCw },
    { id: 'slide-4', title: 'The Challenge', icon: HelpCircle },
    { id: 'slide-5', title: 'Service Design Patterns', icon: Workflow },
    { id: 'slide-6', title: 'Request Design', icon: Send },
    { id: 'slide-7', title: 'Response Design', icon: Inbox },
    { id: 'slide-8', title: 'What We\'re Building', icon: Target },
    { id: 'slide-9', title: 'Task Interface Design', icon: FileCode },
    { id: 'slide-10', title: 'Creating the Interfaces', icon: Package },
    { id: 'slide-11', title: 'Task Manager Node', icon: Code },
    { id: 'slide-12', title: 'Understanding the Code', icon: Search },
    { id: 'slide-13', title: 'Building and Testing', icon: Hammer },
    { id: 'slide-14', title: 'Testing assign_task', icon: Play },
    { id: 'slide-15', title: 'Task Query Service', icon: ClipboardList },
    { id: 'slide-16', title: 'Cancel Task Service', icon: XCircle },
    { id: 'slide-17', title: 'Complete Task Manager', icon: Code },
    { id: 'slide-18', title: 'Testing All Services', icon: Terminal },
    { id: 'slide-19', title: 'Task Client Node', icon: Code },
    { id: 'slide-20', title: 'Running the System', icon: Activity },
    { id: 'slide-21', title: 'Error Handling Patterns', icon: ShieldCheck },
    { id: 'slide-22', title: 'Best Practices', icon: Lightbulb },
    { id: 'slide-23', title: 'Summary', icon: Layout },
    { id: 'slide-24', title: 'Homework', icon: Target },
    { id: 'slide-25', title: 'Next Lecture', icon: ArrowRight },
  ];

  // Service interface definitions
  const assignTaskSrvCode = `# AssignTask.srv
# Request - delivery task details from warehouse
string package_id           # Unique package identifier
float64 destination_x       # Target X coordinate in warehouse
float64 destination_y       # Target Y coordinate in warehouse
int32 priority              # 1=low, 2=normal, 3=high
---
# Response - whether task was accepted
bool success                # True if task was accepted
string message              # Detailed feedback
string task_id              # Assigned task ID (if successful)`;

  const getTaskStatusSrvCode = `# GetTaskStatus.srv
# Request - ask about current task
string task_id              # Optional: specific task, empty = current
---
# Response - task information
bool has_active_task        # Is there an active task?
string task_id              # Current task ID
string package_id           # Package being delivered
string status               # "idle", "active", "completed", "failed"
float64 destination_x       # Where robot is going
float64 destination_y
string message              # Additional details`;

  const cancelTaskSrvCode = `# CancelTask.srv
# Request - which task to cancel
string task_id              # Task to cancel (empty = current task)
string reason               # Why it's being cancelled
---
# Response - cancellation result
bool success                # Was cancellation successful?
string message              # Details about the cancellation`;

  const resetRobotSrvCode = `# ResetRobot.srv
# Request - reset options
bool clear_task_history     # Clear completed task list?
bool return_to_origin       # Reset position to (0,0)?
---
# Response - reset confirmation
bool success
string message
float64 position_x          # New position after reset
float64 position_y`;

  // Task Manager Node code
  const taskManagerCodePart1 = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AssignTask, GetTaskStatus, CancelTask

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Task state
        self.current_task = None
        self.task_counter = 0
        self.task_history = []
        
        # Warehouse bounds for validation
        self.warehouse_x = (0.0, 50.0)  # min, max X
        self.warehouse_y = (0.0, 30.0)  # min, max Y
        
        # Create all services
        self.assign_service = self.create_service(
            AssignTask,
            'assign_task',
            self.assign_task_callback
        )
        
        self.status_service = self.create_service(
            GetTaskStatus,
            'get_task_status',
            self.get_status_callback
        )
        
        self.cancel_service = self.create_service(
            CancelTask,
            'cancel_task',
            self.cancel_task_callback
        )
        
        self.get_logger().info("Task Manager ready! Waiting for delivery tasks...")`;

  const assignTaskCallbackCode = `    def assign_task_callback(self, request, response):
        """Handle new task assignment requests."""
        
        # Check if robot is already busy
        if self.current_task is not None:
            response.success = False
            response.message = f"Robot busy with task {self.current_task['task_id']}"
            response.task_id = ""
            self.get_logger().warn(response.message)
            return response
        
        # Validate destination is within warehouse
        x, y = request.destination_x, request.destination_y
        
        if not (self.warehouse_x[0] <= x <= self.warehouse_x[1]):
            response.success = False
            response.message = f"X coordinate {x:.1f} outside warehouse bounds {self.warehouse_x}"
            response.task_id = ""
            self.get_logger().warn(response.message)
            return response
        
        if not (self.warehouse_y[0] <= y <= self.warehouse_y[1]):
            response.success = False
            response.message = f"Y coordinate {y:.1f} outside warehouse bounds {self.warehouse_y}"
            response.task_id = ""
            self.get_logger().warn(response.message)
            return response
        
        # Validate priority
        if request.priority not in [1, 2, 3]:
            response.success = False
            response.message = f"Invalid priority {request.priority}. Must be 1, 2, or 3"
            response.task_id = ""
            return response
        
        # All validation passed - create the task
        self.task_counter += 1
        task_id = f"TASK-{self.task_counter:04d}"
        
        self.current_task = {
            'task_id': task_id,
            'package_id': request.package_id,
            'destination_x': x,
            'destination_y': y,
            'priority': request.priority,
            'status': 'active'
        }
        
        response.success = True
        response.message = f"Task assigned: deliver {request.package_id} to ({x:.1f}, {y:.1f})"
        response.task_id = task_id
        
        self.get_logger().info(response.message)
        return response`;

  const getStatusCallbackCode = `    def get_status_callback(self, request, response):
        """Return current task status."""
        
        if self.current_task is None:
            response.has_active_task = False
            response.task_id = ""
            response.package_id = ""
            response.status = "idle"
            response.destination_x = 0.0
            response.destination_y = 0.0
            response.message = "No active task. Robot is idle."
        else:
            task = self.current_task
            response.has_active_task = True
            response.task_id = task['task_id']
            response.package_id = task['package_id']
            response.status = task['status']
            response.destination_x = task['destination_x']
            response.destination_y = task['destination_y']
            response.message = f"Delivering {task['package_id']} to destination"
        
        return response`;

  const cancelTaskCallbackCode = `    def cancel_task_callback(self, request, response):
        """Cancel current or specified task."""
        
        if self.current_task is None:
            response.success = False
            response.message = "No active task to cancel"
            return response
        
        # Check if cancelling specific task
        if request.task_id and request.task_id != self.current_task['task_id']:
            response.success = False
            response.message = f"Task {request.task_id} not found. Current: {self.current_task['task_id']}"
            return response
        
        # Cancel the task
        cancelled_id = self.current_task['task_id']
        cancelled_pkg = self.current_task['package_id']
        
        # Save to history
        self.current_task['status'] = 'cancelled'
        self.task_history.append(self.current_task)
        self.current_task = None
        
        response.success = True
        response.message = f"Cancelled {cancelled_id} ({cancelled_pkg}). Reason: {request.reason}"
        
        self.get_logger().info(response.message)
        return response`;

  const taskManagerMainCode = `def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const completeTaskManagerCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AssignTask, GetTaskStatus, CancelTask

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Task state
        self.current_task = None
        self.task_counter = 0
        self.task_history = []
        
        # Warehouse bounds for validation
        self.warehouse_x = (0.0, 50.0)
        self.warehouse_y = (0.0, 30.0)
        
        # Create all services
        self.assign_service = self.create_service(
            AssignTask, 'assign_task', self.assign_task_callback
        )
        
        self.status_service = self.create_service(
            GetTaskStatus, 'get_task_status', self.get_status_callback
        )
        
        self.cancel_service = self.create_service(
            CancelTask, 'cancel_task', self.cancel_task_callback
        )
        
        self.get_logger().info("Task Manager ready!")
    
    def assign_task_callback(self, request, response):
        # Check if robot is already busy
        if self.current_task is not None:
            response.success = False
            response.message = f"Robot busy with task {self.current_task['task_id']}"
            response.task_id = ""
            return response
        
        # Validate coordinates
        x, y = request.destination_x, request.destination_y
        if not (self.warehouse_x[0] <= x <= self.warehouse_x[1]):
            response.success = False
            response.message = f"X={x:.1f} outside bounds"
            response.task_id = ""
            return response
        
        if not (self.warehouse_y[0] <= y <= self.warehouse_y[1]):
            response.success = False
            response.message = f"Y={y:.1f} outside bounds"
            response.task_id = ""
            return response
        
        # Create task
        self.task_counter += 1
        task_id = f"TASK-{self.task_counter:04d}"
        
        self.current_task = {
            'task_id': task_id,
            'package_id': request.package_id,
            'destination_x': x,
            'destination_y': y,
            'priority': request.priority,
            'status': 'active'
        }
        
        response.success = True
        response.message = f"Task assigned: {request.package_id} to ({x:.1f}, {y:.1f})"
        response.task_id = task_id
        self.get_logger().info(response.message)
        return response
    
    def get_status_callback(self, request, response):
        if self.current_task is None:
            response.has_active_task = False
            response.status = "idle"
            response.message = "No active task"
        else:
            task = self.current_task
            response.has_active_task = True
            response.task_id = task['task_id']
            response.package_id = task['package_id']
            response.status = task['status']
            response.destination_x = task['destination_x']
            response.destination_y = task['destination_y']
            response.message = f"Delivering {task['package_id']}"
        return response
    
    def cancel_task_callback(self, request, response):
        if self.current_task is None:
            response.success = False
            response.message = "No active task to cancel"
            return response
        
        cancelled_id = self.current_task['task_id']
        self.current_task['status'] = 'cancelled'
        self.task_history.append(self.current_task)
        self.current_task = None
        
        response.success = True
        response.message = f"Cancelled {cancelled_id}. Reason: {request.reason}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const taskClientCode = `import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AssignTask, GetTaskStatus, CancelTask

class TaskClient(Node):
    def __init__(self):
        super().__init__('task_client')
        
        # Create clients for each service
        self.assign_client = self.create_client(AssignTask, 'assign_task')
        self.status_client = self.create_client(GetTaskStatus, 'get_task_status')
        self.cancel_client = self.create_client(CancelTask, 'cancel_task')
        
        # Wait for all services
        self.get_logger().info("Waiting for task manager services...")
        self.assign_client.wait_for_service()
        self.status_client.wait_for_service()
        self.cancel_client.wait_for_service()
        self.get_logger().info("All services available!")
    
    def assign_task(self, package_id, dest_x, dest_y, priority=2):
        request = AssignTask.Request()
        request.package_id = package_id
        request.destination_x = dest_x
        request.destination_y = dest_y
        request.priority = priority
        
        future = self.assign_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def get_status(self):
        request = GetTaskStatus.Request()
        future = self.status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def cancel_task(self, reason):
        request = CancelTask.Request()
        request.reason = reason
        future = self.cancel_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = TaskClient()
    
    # Demo: Full task lifecycle
    client.get_logger().info("=== Task Management Demo ===")
    
    # 1. Check initial status
    status = client.get_status()
    client.get_logger().info(f"Initial status: {status.status}")
    
    # 2. Assign a delivery task
    result = client.assign_task("PKG-001", 25.0, 15.0, priority=2)
    if result.success:
        client.get_logger().info(f"Assigned: {result.task_id}")
    else:
        client.get_logger().warn(f"Failed: {result.message}")
    
    # 3. Check status while active
    status = client.get_status()
    client.get_logger().info(f"Status: {status.status} - {status.message}")
    
    # 4. Try to assign another (should fail - busy)
    result = client.assign_task("PKG-002", 10.0, 5.0)
    client.get_logger().info(f"Second task: {result.message}")
    
    # 5. Cancel current task
    result = client.cancel_task("Demo complete")
    client.get_logger().info(f"Cancel: {result.message}")
    
    # 6. Now assign another
    result = client.assign_task("PKG-002", 10.0, 5.0)
    client.get_logger().info(f"After cancel: {result.message}")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`;

  const cmakeListsUpdatedCode = `cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "srv/SetTargetSpeed.srv"
  "srv/GetSpeedLimit.srv"
  "srv/AssignTask.srv"
  "srv/GetTaskStatus.srv"
  "srv/CancelTask.srv"
  "srv/ResetRobot.srv"
)

ament_package()`;

  const setupPyCode = `entry_points={
    'console_scripts': [
        'battery_monitor = my_robot_pkg.battery_monitor:main',
        'speed_sensor = my_robot_pkg.speed_sensor:main',
        'dashboard = my_robot_pkg.dashboard:main',
        'sensor_filter = my_robot_pkg.sensor_filter:main',
        'speed_controller = my_robot_pkg.speed_controller:main',
        'safety_advisor = my_robot_pkg.safety_advisor:main',
        'task_manager = my_robot_pkg.task_manager:main',
        'task_client = my_robot_pkg.task_client:main',
    ],
},`;

  return (
    <div className="bg-white min-h-screen font-sans selection:bg-zinc-200 selection:text-zinc-900">
      
      <LectureSidebar 
        slides={slides} 
        lectureNumber={5} 
        lectureTitle="Task Management" 
      />

      {/* Main Content */}
      <main className="md:ml-72 transition-all duration-300">
        
        {/* Slide 1: Title */}
        <LectureSlide id="slide-1" title="Building Service-Based Control Systems" subtitle="Task Management for Our Delivery Robot" icon={BookOpen}>
          <div className="mt-12 p-8 bg-zinc-50 rounded-2xl border border-zinc-100">
            <div className="grid gap-6 md:grid-cols-3">
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Lecture</h4>
                <p className="font-medium text-zinc-900">5 of 12</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Focus</h4>
                <p className="font-medium text-zinc-900">Multi-Service Systems</p>
              </div>
              <div>
                <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-2">Goal</h4>
                <p className="font-medium text-zinc-900">Complete Task Management</p>
              </div>
            </div>
            <div className="mt-8 pt-8 border-t border-zinc-200">
              <h4 className="text-sm font-bold text-zinc-400 uppercase tracking-wider mb-4">Today&apos;s Agenda</h4>
              <div className="grid md:grid-cols-4 gap-4">
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1"><Workflow className="inline" size={24} /></div>
                  <div className="text-sm font-medium">Design Patterns</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1"><ClipboardList className="inline" size={24} /></div>
                  <div className="text-sm font-medium">Task Manager</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1"><ShieldCheck className="inline" size={24} /></div>
                  <div className="text-sm font-medium">Validation</div>
                </div>
                <div className="p-3 bg-white rounded-lg border border-zinc-200 text-center">
                  <div className="text-2xl mb-1"><CheckSquare className="inline" size={24} /></div>
                  <div className="text-sm font-medium">Error Handling</div>
                </div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 2: Homework Review */}
        <LectureSlide id="slide-2" title="Homework Review" subtitle="Emergency Stop Service" icon={Check}>
          <div className="mb-8">
            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h3 className="font-bold text-green-900 mb-4">The Assignment Was:</h3>
              <p className="text-green-800 mb-4">Create an <code className="bg-white px-2 py-1 rounded">emergency_stop</code> service that:</p>
              <ul className="space-y-2 text-green-800">
                <li>Immediately sets robot speed to 0</li>
                <li>Returns confirmation with timestamp</li>
                <li>Logs the emergency stop event</li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-blue-50 border border-blue-100 rounded-xl">
            <h4 className="font-bold text-blue-900 mb-4">Key Learning: Services for Critical Commands</h4>
            <p className="text-blue-800">
              Emergency stop is a perfect use case for services. You need to <strong>know</strong> the command was received and executed. Topics cannot provide this guarantee because there is no response mechanism.
            </p>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">Real-World Insight</h4>
            <p className="text-amber-800 text-sm">
              In actual robots, safety-critical commands like emergency stop use dedicated hardware lines in addition to software services. But the software service pattern you learned is used everywhere for validated commands.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 3: Review - What Services Do */}
        <LectureSlide id="slide-3" title="Review: What Services Do" subtitle="Request-Response Communication" icon={RefreshCw}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-2xl font-medium">
              Services provide <span className="text-green-400">guaranteed responses</span> to requests.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">What We Learned in Lecture 4</h4>
              <ul className="space-y-3 text-zinc-700">
                <li className="flex items-start">
                  <CheckCircle className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Services use request-response pattern</span>
                </li>
                <li className="flex items-start">
                  <CheckCircle className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Server waits for requests, processes, returns response</span>
                </li>
                <li className="flex items-start">
                  <CheckCircle className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Client sends request and waits for response</span>
                </li>
                <li className="flex items-start">
                  <CheckCircle className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Interface defines request and response fields</span>
                </li>
              </ul>
            </div>

            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">Today: Multiple Services, One Node</h4>
              <ul className="space-y-3 text-zinc-700">
                <li className="flex items-start">
                  <ArrowRight className="text-blue-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>A node can provide many services</span>
                </li>
                <li className="flex items-start">
                  <ArrowRight className="text-blue-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Design patterns for robust interfaces</span>
                </li>
                <li className="flex items-start">
                  <ArrowRight className="text-blue-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Proper validation and error handling</span>
                </li>
                <li className="flex items-start">
                  <ArrowRight className="text-blue-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Building a complete task management system</span>
                </li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 4: The Challenge */}
        <LectureSlide id="slide-4" title="The Challenge" subtitle="Managing Delivery Tasks" icon={HelpCircle}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              Our delivery robot needs to receive tasks from a warehouse management system.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-zinc-900 text-white rounded-xl">
              <h4 className="font-bold mb-4 flex items-center">
                <Building2 className="mr-2" size={20} />
                Warehouse System Needs To:
              </h4>
              <ul className="space-y-3 text-zinc-300">
                <li>Assign delivery tasks to the robot</li>
                <li>Check what task the robot is doing</li>
                <li>Cancel tasks when priorities change</li>
                <li>Get confirmation of task acceptance</li>
              </ul>
            </div>

            <div className="p-6 bg-zinc-900 text-white rounded-xl">
              <h4 className="font-bold mb-4 flex items-center">
                <Bot className="mr-2" size={20} />
                Robot Needs To:
              </h4>
              <ul className="space-y-3 text-zinc-300">
                <li>Validate that destinations are reachable</li>
                <li>Reject tasks if already busy</li>
                <li>Report current task status</li>
                <li>Allow safe task cancellation</li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-amber-50 border border-amber-200 rounded-xl">
            <h4 className="font-bold text-amber-900 mb-4">Why Not Just Use Topics?</h4>
            <div className="grid md:grid-cols-2 gap-4 text-amber-800">
              <div>
                <p className="font-medium mb-2">Publishing a task on a topic:</p>
                <ul className="text-sm space-y-1">
                  <li>Was the task received?</li>
                  <li>Was the destination valid?</li>
                  <li>Was the robot available?</li>
                  <li>No way to know.</li>
                </ul>
              </div>
              <div>
                <p className="font-medium mb-2">Calling a service:</p>
                <ul className="text-sm space-y-1">
                  <li>Success: &quot;Task TASK-0001 assigned&quot;</li>
                  <li>Failure: &quot;Invalid destination (60,20)&quot;</li>
                  <li>Failure: &quot;Robot busy with TASK-0003&quot;</li>
                  <li>Clear, actionable feedback.</li>
                </ul>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 5: Service Design Patterns */}
        <LectureSlide id="slide-5" title="Service Design Patterns" subtitle="How to Design Good Interfaces" icon={Workflow}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              Before writing code, we design our service interfaces carefully. Good design makes your system easier to use and debug.
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 mb-8">
            <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-4">1. Clear Purpose</h4>
              <p className="text-blue-800 text-sm mb-3">
                Each service should do <strong>one thing well</strong>. Don&apos;t combine unrelated operations.
              </p>
              <div className="text-xs font-mono bg-white p-2 rounded">
                <div className="text-green-600">Good: assign_task</div>
                <div className="text-green-600">Good: cancel_task</div>
                <div className="text-red-600">Bad: do_task_stuff</div>
              </div>
            </div>

            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <h4 className="font-bold text-green-900 mb-4">2. Validation First</h4>
              <p className="text-green-800 text-sm mb-3">
                Always validate input before processing. Return meaningful errors.
              </p>
              <div className="text-xs font-mono bg-white p-2 rounded">
                <div>if x &gt; max_x:</div>
                <div className="pl-2">response.success = False</div>
                <div className="pl-2">response.message = &quot;...&quot;</div>
              </div>
            </div>

            <div className="p-6 bg-purple-50 border border-purple-200 rounded-xl">
              <h4 className="font-bold text-purple-900 mb-4">3. Informative Responses</h4>
              <p className="text-purple-800 text-sm mb-3">
                Always include a success flag and a message explaining the result.
              </p>
              <div className="text-xs font-mono bg-white p-2 rounded">
                <div>bool success</div>
                <div>string message</div>
                <div className="text-zinc-500"># Plus relevant data</div>
              </div>
            </div>
          </div>

          <div className="p-4 bg-zinc-900 text-white rounded-xl">
            <p className="text-center font-mono">
              Design your interface first, then implement the logic. The interface is your contract with other systems.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 6: Request Design */}
        <LectureSlide id="slide-6" title="Request Design" subtitle="What Information to Send" icon={Send}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              The request defines what information a client must provide. Think about what the server needs to do its job.
            </p>
          </div>

          <div className="p-6 bg-zinc-50 rounded-xl mb-8">
            <h4 className="font-bold text-zinc-900 mb-4">Example: Assigning a Delivery Task</h4>
            <p className="text-zinc-600 mb-4">What does the robot need to know to accept a delivery?</p>
            
            <div className="grid md:grid-cols-2 gap-6">
              <div>
                <h5 className="font-medium text-zinc-800 mb-2">Required Information:</h5>
                <ul className="space-y-2 text-zinc-700">
                  <li className="flex items-center">
                    <Package className="mr-2 text-blue-500" size={16} />
                    <span><strong>package_id</strong> - Which package to deliver</span>
                  </li>
                  <li className="flex items-center">
                    <MapPin className="mr-2 text-blue-500" size={16} />
                    <span><strong>destination_x, destination_y</strong> - Where to go</span>
                  </li>
                </ul>
              </div>
              <div>
                <h5 className="font-medium text-zinc-800 mb-2">Optional but Useful:</h5>
                <ul className="space-y-2 text-zinc-700">
                  <li className="flex items-center">
                    <Zap className="mr-2 text-amber-500" size={16} />
                    <span><strong>priority</strong> - How urgent (1, 2, or 3)</span>
                  </li>
                </ul>
              </div>
            </div>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
              <h5 className="font-bold text-green-900 mb-2">Good Request Design</h5>
              <ul className="text-green-800 text-sm space-y-1">
                <li>Include all data needed to complete the task</li>
                <li>Use clear, descriptive field names</li>
                <li>Choose appropriate data types</li>
                <li>Add comments explaining each field</li>
              </ul>
            </div>
            <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
              <h5 className="font-bold text-red-900 mb-2">Common Mistakes</h5>
              <ul className="text-red-800 text-sm space-y-1">
                <li>Forgetting required information</li>
                <li>Using vague field names like &quot;data&quot;</li>
                <li>Wrong data types (string for numbers)</li>
                <li>No documentation</li>
              </ul>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 7: Response Design */}
        <LectureSlide id="slide-7" title="Response Design" subtitle="What to Return to the Client" icon={Inbox}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              The response tells the client what happened. A good response enables the client to take appropriate next steps.
            </p>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl mb-8">
            <h4 className="font-bold mb-4">The Three Essential Response Fields</h4>
            <div className="grid md:grid-cols-3 gap-4">
              <div className="p-4 bg-zinc-800 rounded-lg">
                <div className="font-mono text-green-400 mb-2">bool success</div>
                <p className="text-sm text-zinc-300">Did the operation succeed? Client checks this first.</p>
              </div>
              <div className="p-4 bg-zinc-800 rounded-lg">
                <div className="font-mono text-blue-400 mb-2">string message</div>
                <p className="text-sm text-zinc-300">Human-readable explanation. Useful for logging and debugging.</p>
              </div>
              <div className="p-4 bg-zinc-800 rounded-lg">
                <div className="font-mono text-purple-400 mb-2">[result data]</div>
                <p className="text-sm text-zinc-300">Relevant data the client might need (IDs, values, etc.)</p>
              </div>
            </div>
          </div>

          <div className="space-y-4">
            <h4 className="font-bold text-zinc-900">Example Response Scenarios:</h4>
            
            <div className="grid md:grid-cols-2 gap-4">
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h5 className="font-bold text-green-900 mb-2">Success Case</h5>
                <div className="font-mono text-sm text-green-800">
                  <div>success: True</div>
                  <div>message: &quot;Task assigned: PKG-001 to (25.0, 15.0)&quot;</div>
                  <div>task_id: &quot;TASK-0001&quot;</div>
                </div>
              </div>
              
              <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
                <h5 className="font-bold text-red-900 mb-2">Failure Case</h5>
                <div className="font-mono text-sm text-red-800">
                  <div>success: False</div>
                  <div>message: &quot;X=60.0 outside warehouse bounds (0-50)&quot;</div>
                  <div>task_id: &quot;&quot;</div>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h5 className="font-bold text-amber-900 mb-2">Design Tip</h5>
            <p className="text-amber-800 text-sm">
              The message should explain <strong>why</strong> something failed, not just that it failed. &quot;Invalid coordinates&quot; is less useful than &quot;X coordinate 60.0 exceeds warehouse boundary of 50.0&quot;.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 8: What We're Building */}
        <LectureSlide id="slide-8" title="What We're Building Today" subtitle="Complete Task Management System" icon={Target}>
          <div className="mb-6 p-4 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-lg">A node that provides multiple services to manage delivery tasks.</p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 mb-8">
            <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl">
              <div className="flex items-center mb-4">
                <Truck className="text-blue-600 mr-2" size={24} />
                <h4 className="font-bold text-blue-900">assign_task</h4>
              </div>
              <p className="text-blue-800 text-sm mb-3">Accept new delivery tasks from the warehouse system.</p>
              <ul className="text-blue-700 text-xs space-y-1">
                <li>Validates destination coordinates</li>
                <li>Checks if robot is available</li>
                <li>Assigns unique task ID</li>
              </ul>
            </div>

            <div className="p-6 bg-green-50 border border-green-200 rounded-xl">
              <div className="flex items-center mb-4">
                <ClipboardList className="text-green-600 mr-2" size={24} />
                <h4 className="font-bold text-green-900">get_task_status</h4>
              </div>
              <p className="text-green-800 text-sm mb-3">Query the current task state.</p>
              <ul className="text-green-700 text-xs space-y-1">
                <li>Returns current task details</li>
                <li>Reports idle if no task</li>
                <li>Includes destination info</li>
              </ul>
            </div>

            <div className="p-6 bg-red-50 border border-red-200 rounded-xl">
              <div className="flex items-center mb-4">
                <XCircle className="text-red-600 mr-2" size={24} />
                <h4 className="font-bold text-red-900">cancel_task</h4>
              </div>
              <p className="text-red-800 text-sm mb-3">Cancel the current delivery task.</p>
              <ul className="text-red-700 text-xs space-y-1">
                <li>Stops current delivery</li>
                <li>Records cancellation reason</li>
                <li>Saves to task history</li>
              </ul>
            </div>
          </div>

          <div className="p-6 bg-purple-50 border border-purple-200 rounded-xl">
            <h4 className="font-bold text-purple-900 mb-4">Architecture Overview</h4>
            <div className="flex items-center justify-center space-x-4 text-sm">
              <div className="p-3 bg-white rounded-lg border border-purple-200 text-center">
                <div className="font-bold">Warehouse System</div>
                <div className="text-purple-600">(Client)</div>
              </div>
              <div className="flex flex-col items-center">
                <ArrowRight className="text-purple-400" size={24} />
                <span className="text-xs text-purple-500">requests</span>
              </div>
              <div className="p-3 bg-purple-200 rounded-lg border border-purple-300 text-center">
                <div className="font-bold">Task Manager</div>
                <div className="text-purple-700">(3 Services)</div>
              </div>
              <div className="flex flex-col items-center">
                <ArrowRight className="text-purple-400" size={24} />
                <span className="text-xs text-purple-500">responses</span>
              </div>
              <div className="p-3 bg-white rounded-lg border border-purple-200 text-center">
                <div className="font-bold">Warehouse System</div>
                <div className="text-purple-600">(Client)</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 9: Task Interface Design */}
        <LectureSlide id="slide-9" title="Designing Our Service Interfaces" subtitle="The .srv Files" icon={FileCode}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Let&apos;s design the interfaces for our three services. Remember: the interface is the contract between client and server.
            </p>
          </div>

          <div className="space-y-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">1. AssignTask.srv - Accept new deliveries</h4>
              <CodeBlock 
                filename="my_robot_interfaces/srv/AssignTask.srv"
                code={assignTaskSrvCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">2. GetTaskStatus.srv - Query current state</h4>
              <CodeBlock 
                filename="my_robot_interfaces/srv/GetTaskStatus.srv"
                code={getTaskStatusSrvCode}
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">3. CancelTask.srv - Stop current task</h4>
              <CodeBlock 
                filename="my_robot_interfaces/srv/CancelTask.srv"
                code={cancelTaskSrvCode}
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 10: Creating the Interfaces */}
        <LectureSlide id="slide-10" title="Creating the Interfaces" subtitle="Adding to Our Package" icon={Package}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              We already have <code className="bg-zinc-100 px-2 py-1 rounded">my_robot_interfaces</code> from Lecture 4. Now we add our new service definitions.
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Create the .srv files</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws/src/my_robot_interfaces/srv"
                title="Terminal"
              />
              <p className="text-sm text-zinc-600 mt-2">Create each .srv file shown on the previous slide using your text editor.</p>
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: Update CMakeLists.txt</h4>
              <CodeBlock 
                filename="my_robot_interfaces/CMakeLists.txt"
                code={cmakeListsUpdatedCode}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 3: Build the interfaces package</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws && colcon build --packages-select my_robot_interfaces"
                output={`Starting >>> my_robot_interfaces
Finished <<< my_robot_interfaces [5.2s]

Summary: 1 package finished`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 4: Source and verify</h4>
              <TerminalBlock 
                command="source install/setup.bash && ros2 interface show my_robot_interfaces/srv/AssignTask"
                output={`string package_id
float64 destination_x
float64 destination_y
int32 priority
---
bool success
string message
string task_id`}
                title="Terminal"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 11: Task Manager Node */}
        <LectureSlide id="slide-11" title="Task Manager Node" subtitle="The Service Server" icon={Code}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              Create <code className="bg-zinc-100 px-2 py-1 rounded">task_manager.py</code> in your Python package. This node provides all three services.
            </p>
          </div>

          <div className="mb-6">
            <h4 className="font-bold text-zinc-900 mb-3">Node Initialization and Service Creation:</h4>
            <CodeBlock 
              filename="my_robot_pkg/task_manager.py (Part 1)"
              code={taskManagerCodePart1}
            />
          </div>

          <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <h4 className="font-bold text-blue-900 mb-2">Key Points:</h4>
            <ul className="text-blue-800 text-sm space-y-1">
              <li><strong>One node, three services</strong> - Each service has its own callback</li>
              <li><strong>Shared state</strong> - <code className="bg-white px-1 rounded">current_task</code> is accessed by all callbacks</li>
              <li><strong>Validation bounds</strong> - Warehouse dimensions defined as instance variables</li>
            </ul>
          </div>
        </LectureSlide>

        {/* Slide 12: Understanding the Code */}
        <LectureSlide id="slide-12" title="The assign_task Callback" subtitle="Handling Task Assignment" icon={Search}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              This callback handles the most complex service. It validates input and creates new tasks.
            </p>
          </div>

          <CodeBlock 
            filename="task_manager.py - assign_task_callback"
            code={assignTaskCallbackCode}
          />

          <div className="mt-6 grid md:grid-cols-3 gap-4">
            <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
              <h5 className="font-bold text-amber-900 mb-2">Check 1: Busy?</h5>
              <p className="text-amber-800 text-sm">If robot has an active task, reject the new one.</p>
            </div>
            <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
              <h5 className="font-bold text-amber-900 mb-2">Check 2: Valid Location?</h5>
              <p className="text-amber-800 text-sm">Ensure coordinates are within warehouse bounds.</p>
            </div>
            <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
              <h5 className="font-bold text-amber-900 mb-2">Check 3: Valid Priority?</h5>
              <p className="text-amber-800 text-sm">Priority must be 1, 2, or 3.</p>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 13: Building and Testing */}
        <LectureSlide id="slide-13" title="Building and Testing" subtitle="Getting the Node Running" icon={Hammer}>
          <div className="space-y-4">
            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 1: Add to setup.py</h4>
              <CodeBlock 
                filename="my_robot_pkg/setup.py"
                code={setupPyCode}
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 2: Build</h4>
              <TerminalBlock 
                command="cd ~/ros2_ws && colcon build --packages-select my_robot_pkg && source install/setup.bash"
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 3: Run the node</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg task_manager"
                output={`[INFO] [task_manager]: Task Manager ready! Waiting for delivery tasks...`}
                title="Terminal 1"
              />
            </div>

            <div className="p-4 bg-zinc-50 rounded-lg">
              <h4 className="font-bold text-zinc-900 mb-3">Step 4: Check available services</h4>
              <TerminalBlock 
                command="ros2 service list"
                output={`/assign_task
/cancel_task
/get_task_status
/task_manager/describe_parameters
...`}
                title="Terminal 2"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 14: Testing assign_task */}
        <LectureSlide id="slide-14" title="Testing assign_task" subtitle="Using CLI Before Writing a Client" icon={Play}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Always test your services with CLI tools first. This helps you verify behavior before writing client code.
            </p>
          </div>

          <div className="space-y-4">
            <div className="p-4 bg-green-50 rounded-lg">
              <h4 className="font-bold text-green-900 mb-3">Test 1: Valid task assignment</h4>
              <TerminalBlock 
                command={`ros2 service call /assign_task my_robot_interfaces/srv/AssignTask "{package_id: 'PKG-001', destination_x: 25.0, destination_y: 15.0, priority: 2}"`}
                output={`response:
  success: true
  message: 'Task assigned: deliver PKG-001 to (25.0, 15.0)'
  task_id: 'TASK-0001'`}
                title="Terminal 2"
              />
            </div>

            <div className="p-4 bg-red-50 rounded-lg">
              <h4 className="font-bold text-red-900 mb-3">Test 2: Robot is busy (try again while task active)</h4>
              <TerminalBlock 
                command={`ros2 service call /assign_task my_robot_interfaces/srv/AssignTask "{package_id: 'PKG-002', destination_x: 10.0, destination_y: 5.0, priority: 1}"`}
                output={`response:
  success: false
  message: 'Robot busy with task TASK-0001'
  task_id: ''`}
                title="Terminal 2"
              />
            </div>

            <div className="p-4 bg-red-50 rounded-lg">
              <h4 className="font-bold text-red-900 mb-3">Test 3: Invalid coordinates</h4>
              <TerminalBlock 
                command={`ros2 service call /assign_task my_robot_interfaces/srv/AssignTask "{package_id: 'PKG-003', destination_x: 60.0, destination_y: 15.0, priority: 2}"`}
                output={`response:
  success: false
  message: 'X coordinate 60.0 outside warehouse bounds (0.0, 50.0)'
  task_id: ''`}
                title="Terminal 2"
              />
            </div>
          </div>
        </LectureSlide>

        {/* Slide 15: Task Query Service */}
        <LectureSlide id="slide-15" title="The get_task_status Callback" subtitle="Querying Current State" icon={ClipboardList}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              This is a simpler callback that just reports current state. No validation needed - it always succeeds.
            </p>
          </div>

          <CodeBlock 
            filename="task_manager.py - get_status_callback"
            code={getStatusCallbackCode}
          />

          <div className="mt-6 p-4 bg-zinc-50 rounded-lg">
            <h4 className="font-bold text-zinc-900 mb-3">Test it:</h4>
            <TerminalBlock 
              command="ros2 service call /get_task_status my_robot_interfaces/srv/GetTaskStatus \"{}\""
              output={`response:
  has_active_task: true
  task_id: 'TASK-0001'
  package_id: 'PKG-001'
  status: 'active'
  destination_x: 25.0
  destination_y: 15.0
  message: 'Delivering PKG-001 to destination'`}
              title="Terminal 2"
            />
          </div>

          <div className="mt-4 p-4 bg-blue-50 border border-blue-200 rounded-lg">
            <h4 className="font-bold text-blue-900 mb-2">Note: Empty Request</h4>
            <p className="text-blue-800 text-sm">
              The request has a <code className="bg-white px-1 rounded">task_id</code> field but we&apos;re not using it here. The service returns the current task regardless. You could extend this to query specific tasks from history.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 16: Cancel Task Service */}
        <LectureSlide id="slide-16" title="The cancel_task Callback" subtitle="Stopping Tasks Safely" icon={XCircle}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              Cancellation requires checking if there&apos;s actually a task to cancel, and recording why it was cancelled.
            </p>
          </div>

          <CodeBlock 
            filename="task_manager.py - cancel_task_callback"
            code={cancelTaskCallbackCode}
          />

          <div className="mt-6 grid md:grid-cols-2 gap-4">
            <div className="p-4 bg-green-50 rounded-lg">
              <h4 className="font-bold text-green-900 mb-3">Cancel the active task:</h4>
              <TerminalBlock 
                command={`ros2 service call /cancel_task my_robot_interfaces/srv/CancelTask "{reason: 'Higher priority order'}"`}
                output={`response:
  success: true
  message: 'Cancelled TASK-0001 (PKG-001). Reason: Higher priority order'`}
                title="Terminal"
              />
            </div>

            <div className="p-4 bg-red-50 rounded-lg">
              <h4 className="font-bold text-red-900 mb-3">Try cancelling when idle:</h4>
              <TerminalBlock 
                command={`ros2 service call /cancel_task my_robot_interfaces/srv/CancelTask "{reason: 'Test'}"`}
                output={`response:
  success: false
  message: 'No active task to cancel'`}
                title="Terminal"
              />
            </div>
          </div>

          <div className="mt-4 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">Why Record Cancellation Reason?</h4>
            <p className="text-amber-800 text-sm">
              In real systems, you want to know why tasks were cancelled. This helps with debugging, analytics, and improving the system. Maybe too many tasks are cancelled due to low battery? The history tells you.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 17: Complete Task Manager */}
        <LectureSlide id="slide-17" title="Complete Task Manager" subtitle="Full Code Reference" icon={Code}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              Here&apos;s the complete task manager node with all three services. Copy this to your package.
            </p>
          </div>

          <CodeBlock 
            filename="my_robot_pkg/task_manager.py"
            code={completeTaskManagerCode}
          />
        </LectureSlide>

        {/* Slide 18: Testing All Services */}
        <LectureSlide id="slide-18" title="Testing All Services" subtitle="Complete Workflow Test" icon={Terminal}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Let&apos;s walk through a complete delivery task lifecycle using CLI commands.
            </p>
          </div>

          <div className="space-y-4">
            <div className="flex items-center mb-4">
              <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold mr-3">1</div>
              <h4 className="font-bold text-zinc-900">Check initial status (robot is idle)</h4>
            </div>
            <TerminalBlock 
              command="ros2 service call /get_task_status my_robot_interfaces/srv/GetTaskStatus \"{}\""
              output={`has_active_task: false
status: 'idle'
message: 'No active task. Robot is idle.'`}
              title="Terminal"
            />

            <div className="flex items-center mb-4">
              <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold mr-3">2</div>
              <h4 className="font-bold text-zinc-900">Assign a delivery task</h4>
            </div>
            <TerminalBlock 
              command={`ros2 service call /assign_task my_robot_interfaces/srv/AssignTask "{package_id: 'PKG-001', destination_x: 25.0, destination_y: 15.0, priority: 2}"`}
              output={`success: true
message: 'Task assigned: PKG-001 to (25.0, 15.0)'
task_id: 'TASK-0001'`}
              title="Terminal"
            />

            <div className="flex items-center mb-4">
              <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold mr-3">3</div>
              <h4 className="font-bold text-zinc-900">Check status (now active)</h4>
            </div>
            <TerminalBlock 
              command="ros2 service call /get_task_status my_robot_interfaces/srv/GetTaskStatus \"{}\""
              output={`has_active_task: true
task_id: 'TASK-0001'
status: 'active'`}
              title="Terminal"
            />

            <div className="flex items-center mb-4">
              <div className="w-8 h-8 rounded-full bg-zinc-900 text-white flex items-center justify-center font-bold mr-3">4</div>
              <h4 className="font-bold text-zinc-900">Cancel and assign new task</h4>
            </div>
            <TerminalBlock 
              command={`ros2 service call /cancel_task my_robot_interfaces/srv/CancelTask "{reason: 'Priority change'}"`}
              output={`success: true
message: 'Cancelled TASK-0001 (PKG-001). Reason: Priority change'`}
              title="Terminal"
            />
          </div>
        </LectureSlide>

        {/* Slide 19: Task Client Node */}
        <LectureSlide id="slide-19" title="Task Client Node" subtitle="Programmatic Service Calls" icon={Code}>
          <div className="mb-4">
            <p className="text-lg text-zinc-700">
              In a real system, another node (like a warehouse controller) calls these services programmatically. Here&apos;s a client node that demonstrates the full lifecycle.
            </p>
          </div>

          <CodeBlock 
            filename="my_robot_pkg/task_client.py"
            code={taskClientCode}
          />
        </LectureSlide>

        {/* Slide 20: Running the System */}
        <LectureSlide id="slide-20" title="Running the System" subtitle="Server and Client Together" icon={Activity}>
          <div className="mb-6">
            <p className="text-lg text-zinc-700">
              Run the task manager in one terminal and the client in another to see the full interaction.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 1: Task Manager (Server)</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg task_manager"
                output={`[INFO] [task_manager]: Task Manager ready!
[INFO] [task_manager]: Task assigned: PKG-001 to (25.0, 15.0)
[WARN] [task_manager]: Robot busy with task TASK-0001
[INFO] [task_manager]: Cancelled TASK-0001 (PKG-001). Reason: Demo complete
[INFO] [task_manager]: Task assigned: PKG-002 to (10.0, 5.0)`}
                title="Terminal 1 - Server"
              />
            </div>

            <div>
              <h4 className="font-bold text-zinc-900 mb-3">Terminal 2: Task Client</h4>
              <TerminalBlock 
                command="ros2 run my_robot_pkg task_client"
                output={`[INFO] [task_client]: === Task Management Demo ===
[INFO] [task_client]: Initial status: idle
[INFO] [task_client]: Assigned: TASK-0001
[INFO] [task_client]: Status: active - Delivering PKG-001
[INFO] [task_client]: Second task: Robot busy with task TASK-0001
[INFO] [task_client]: Cancel: Cancelled TASK-0001...
[INFO] [task_client]: After cancel: Task assigned...`}
                title="Terminal 2 - Client"
              />
            </div>
          </div>

          <div className="mt-6 p-4 bg-green-50 border border-green-200 rounded-lg">
            <h4 className="font-bold text-green-900 mb-2">What You Just Built</h4>
            <p className="text-green-800 text-sm">
              A complete task management system where external systems can assign, monitor, and cancel delivery tasks. The robot validates all requests and provides clear feedback. This is exactly how real warehouse robots receive their orders.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 21: Error Handling Patterns */}
        <LectureSlide id="slide-21" title="Error Handling Patterns" subtitle="Building Robust Services" icon={ShieldCheck}>
          <div className="mb-8">
            <p className="text-xl text-zinc-700">
              Good error handling is what separates production-ready code from demos. Here are the patterns to follow.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 mb-8">
            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">1. Validate Early, Fail Fast</h4>
              <p className="text-zinc-700 text-sm mb-4">Check all conditions before doing any work. Return immediately on first failure.</p>
              <div className="font-mono text-xs bg-white p-3 rounded">
                <div className="text-zinc-500"># Check constraints in order</div>
                <div>if busy: return fail(&quot;busy&quot;)</div>
                <div>if invalid_x: return fail(&quot;bad x&quot;)</div>
                <div>if invalid_y: return fail(&quot;bad y&quot;)</div>
                <div className="text-green-600"># All good - do the work</div>
              </div>
            </div>

            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">2. Always Return Valid Response</h4>
              <p className="text-zinc-700 text-sm mb-4">Every code path must set all response fields. Never leave fields uninitialized.</p>
              <div className="font-mono text-xs bg-white p-3 rounded">
                <div className="text-red-600"># Bad: only set on success</div>
                <div>if success: response.task_id = id</div>
                <div className="text-green-600"># Good: always set</div>
                <div>response.task_id = id if success else &quot;&quot;</div>
              </div>
            </div>

            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">3. Log What Happened</h4>
              <p className="text-zinc-700 text-sm mb-4">Use appropriate log levels. Info for success, warn for client errors, error for system problems.</p>
              <div className="font-mono text-xs bg-white p-3 rounded">
                <div>self.get_logger().info(&quot;Task assigned&quot;)</div>
                <div>self.get_logger().warn(&quot;Rejected: busy&quot;)</div>
                <div>self.get_logger().error(&quot;Database fail&quot;)</div>
              </div>
            </div>

            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">4. Include Context in Messages</h4>
              <p className="text-zinc-700 text-sm mb-4">Error messages should include the actual values that caused the problem.</p>
              <div className="font-mono text-xs bg-white p-3 rounded">
                <div className="text-red-600"># Bad</div>
                <div>&quot;Invalid coordinates&quot;</div>
                <div className="text-green-600"># Good</div>
                <div>&quot;X=60.0 outside bounds (0-50)&quot;</div>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 22: Best Practices */}
        <LectureSlide id="slide-22" title="Service Best Practices" subtitle="Guidelines for Production Systems" icon={Lightbulb}>
          <div className="grid md:grid-cols-2 gap-6">
            <div className="space-y-4">
              <div className="p-4 bg-green-50 border border-green-200 rounded-lg">
                <h4 className="font-bold text-green-900 mb-2">Naming Services</h4>
                <ul className="text-green-800 text-sm space-y-1">
                  <li>Use verb_noun format: <code className="bg-white px-1 rounded">assign_task</code></li>
                  <li>Be specific: <code className="bg-white px-1 rounded">get_task_status</code> not <code className="bg-white px-1 rounded">get_status</code></li>
                  <li>Use snake_case consistently</li>
                </ul>
              </div>

              <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
                <h4 className="font-bold text-blue-900 mb-2">Interface Design</h4>
                <ul className="text-blue-800 text-sm space-y-1">
                  <li>Always include <code className="bg-white px-1 rounded">bool success</code></li>
                  <li>Always include <code className="bg-white px-1 rounded">string message</code></li>
                  <li>Document each field with comments</li>
                  <li>Use appropriate types (not strings for numbers)</li>
                </ul>
              </div>

              <div className="p-4 bg-purple-50 border border-purple-200 rounded-lg">
                <h4 className="font-bold text-purple-900 mb-2">State Management</h4>
                <ul className="text-purple-800 text-sm space-y-1">
                  <li>Keep state in instance variables</li>
                  <li>Consider thread safety for complex state</li>
                  <li>Log state changes for debugging</li>
                </ul>
              </div>
            </div>

            <div className="space-y-4">
              <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
                <h4 className="font-bold text-amber-900 mb-2">When to Use Services</h4>
                <ul className="text-amber-800 text-sm space-y-1">
                  <li>Commands that need confirmation</li>
                  <li>Queries that need instant response</li>
                  <li>One-time operations (not streaming)</li>
                  <li>When you need to validate before executing</li>
                </ul>
              </div>

              <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
                <h4 className="font-bold text-red-900 mb-2">When NOT to Use Services</h4>
                <ul className="text-red-800 text-sm space-y-1">
                  <li>Continuous data streams (use topics)</li>
                  <li>Long-running operations (use actions - next lecture)</li>
                  <li>Fire-and-forget messages (use topics)</li>
                </ul>
              </div>

              <div className="p-4 bg-zinc-100 border border-zinc-200 rounded-lg">
                <h4 className="font-bold text-zinc-900 mb-2">Testing Strategy</h4>
                <ul className="text-zinc-700 text-sm space-y-1">
                  <li>Test with CLI before writing clients</li>
                  <li>Test success cases first</li>
                  <li>Test each validation path</li>
                  <li>Test edge cases (empty strings, zero values)</li>
                </ul>
              </div>
            </div>
          </div>
        </LectureSlide>

        {/* Slide 23: Summary */}
        <LectureSlide id="slide-23" title="What You Learned Today" subtitle="Lecture 5 Summary" icon={Layout}>
          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 rounded-lg">
                <h4 className="font-bold text-blue-900 mb-2">Service Design Patterns</h4>
                <ul className="text-blue-800 text-sm space-y-1">
                  <li>Clear purpose per service</li>
                  <li>Validate first, fail fast</li>
                  <li>Informative responses with success + message</li>
                </ul>
              </div>

              <div className="p-4 bg-green-50 rounded-lg">
                <h4 className="font-bold text-green-900 mb-2">Multi-Service Nodes</h4>
                <ul className="text-green-800 text-sm space-y-1">
                  <li>One node can provide multiple services</li>
                  <li>Services share node state</li>
                  <li>Each service has its own callback</li>
                </ul>
              </div>
            </div>

            <div className="space-y-4">
              <div className="p-4 bg-purple-50 rounded-lg">
                <h4 className="font-bold text-purple-900 mb-2">Task Management System</h4>
                <ul className="text-purple-800 text-sm space-y-1">
                  <li>assign_task - Accept new work</li>
                  <li>get_task_status - Query state</li>
                  <li>cancel_task - Stop work safely</li>
                </ul>
              </div>

              <div className="p-4 bg-amber-50 rounded-lg">
                <h4 className="font-bold text-amber-900 mb-2">Error Handling</h4>
                <ul className="text-amber-800 text-sm space-y-1">
                  <li>Check all constraints before executing</li>
                  <li>Return meaningful error messages</li>
                  <li>Log with appropriate severity</li>
                </ul>
              </div>
            </div>
          </div>

          <div className="p-6 bg-zinc-900 text-white rounded-xl text-center">
            <p className="text-xl">
              You now know how to build <span className="text-green-400">robust service-based systems</span> that external systems can reliably interact with.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 24: Homework */}
        <LectureSlide id="slide-24" title="Homework" subtitle="Add reset_robot Service" icon={Target}>
          <div className="mb-8 p-6 bg-zinc-900 text-white rounded-xl">
            <h3 className="text-2xl font-bold mb-4">Assignment: Implement reset_robot Service</h3>
            <p className="text-zinc-300">
              Add a new service to the task manager that resets the robot to its initial state.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 mb-8">
            <div className="p-6 bg-blue-50 border border-blue-200 rounded-xl">
              <h4 className="font-bold text-blue-900 mb-4">Service Interface:</h4>
              <CodeBlock 
                filename="ResetRobot.srv"
                code={resetRobotSrvCode}
              />
            </div>

            <div className="p-6 bg-zinc-50 rounded-xl">
              <h4 className="font-bold text-zinc-900 mb-4">Requirements:</h4>
              <ul className="space-y-3 text-zinc-700">
                <li className="flex items-start">
                  <CheckSquare className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Cancel any active task before resetting</span>
                </li>
                <li className="flex items-start">
                  <CheckSquare className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Optionally clear task history based on request</span>
                </li>
                <li className="flex items-start">
                  <CheckSquare className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Reset task counter to 0</span>
                </li>
                <li className="flex items-start">
                  <CheckSquare className="text-green-500 mr-2 mt-1 flex-shrink-0" size={16} />
                  <span>Return new position (0, 0) if return_to_origin is true</span>
                </li>
              </ul>
            </div>
          </div>

          <div className="space-y-4">
            <h4 className="font-bold text-zinc-900">Steps to Complete:</h4>
            <div className="grid md:grid-cols-4 gap-4">
              <div className="p-4 bg-zinc-100 rounded-lg text-center">
                <div className="text-2xl font-bold text-zinc-900 mb-2">1</div>
                <p className="text-sm text-zinc-600">Create ResetRobot.srv in interfaces package</p>
              </div>
              <div className="p-4 bg-zinc-100 rounded-lg text-center">
                <div className="text-2xl font-bold text-zinc-900 mb-2">2</div>
                <p className="text-sm text-zinc-600">Update CMakeLists.txt and rebuild interfaces</p>
              </div>
              <div className="p-4 bg-zinc-100 rounded-lg text-center">
                <div className="text-2xl font-bold text-zinc-900 mb-2">3</div>
                <p className="text-sm text-zinc-600">Add service and callback to task_manager.py</p>
              </div>
              <div className="p-4 bg-zinc-100 rounded-lg text-center">
                <div className="text-2xl font-bold text-zinc-900 mb-2">4</div>
                <p className="text-sm text-zinc-600">Test with CLI commands</p>
              </div>
            </div>
          </div>

          <div className="mt-6 p-4 bg-amber-50 border border-amber-200 rounded-lg">
            <h4 className="font-bold text-amber-900 mb-2">Bonus Challenge</h4>
            <p className="text-amber-800 text-sm">
              Add a <code className="bg-white px-1 rounded">position_x</code> and <code className="bg-white px-1 rounded">position_y</code> to your task manager state. Update it when tasks are assigned, and reset it when <code className="bg-white px-1 rounded">return_to_origin</code> is True.
            </p>
          </div>
        </LectureSlide>

        {/* Slide 25: Next Lecture */}
        <LectureSlide id="slide-25" title="Next Lecture" subtitle="Parameters - Making Nodes Configurable" icon={ArrowRight}>
          <div className="p-8 bg-gradient-to-br from-zinc-900 to-zinc-800 text-white rounded-2xl">
            <h3 className="text-2xl font-bold mb-6">Lecture 6: Parameters</h3>
            
            <div className="grid md:grid-cols-2 gap-6 mb-8">
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">The Problem</h4>
                <p className="text-zinc-400 text-sm">
                  Right now, our warehouse bounds are hardcoded: (0-50, 0-30). What if we deploy to a different warehouse? We have to change the code. This is not scalable.
                </p>
              </div>
              <div>
                <h4 className="font-bold text-zinc-300 mb-3">The Solution: Parameters</h4>
                <p className="text-zinc-400 text-sm">
                  Parameters let you configure nodes at runtime without changing code. Launch with different settings for different warehouses.
                </p>
              </div>
            </div>

            <div className="space-y-4">
              <h4 className="font-bold text-zinc-300">What You&apos;ll Learn:</h4>
              <div className="grid md:grid-cols-2 gap-4">
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Cog className="text-blue-400 mb-2" size={20} />
                  <p className="text-sm">Declaring and using parameters</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <Settings className="text-green-400 mb-2" size={20} />
                  <p className="text-sm">Setting parameters at runtime</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <FileCode className="text-purple-400 mb-2" size={20} />
                  <p className="text-sm">YAML parameter files</p>
                </div>
                <div className="p-3 bg-zinc-800/50 rounded-lg">
                  <RefreshCw className="text-amber-400 mb-2" size={20} />
                  <p className="text-sm">Dynamic reconfiguration</p>
                </div>
              </div>
            </div>
          </div>

          <div className="mt-8 text-center text-zinc-500">
            <p>Complete your homework before the next session.</p>
            <p className="mt-2 font-medium text-zinc-700">See you in Lecture 6!</p>
          </div>
        </LectureSlide>

      </main>
    </div>
  );
}
